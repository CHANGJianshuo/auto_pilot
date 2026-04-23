#![cfg_attr(not(test), no_std)]

//! Multirotor rate-loop assembly.
//!
//! Wires together the algorithm modules developed across M1 and M2 into a
//! single, reusable [`rate_loop_step`]:
//!
//! ```text
//!   IMU sample
//!       │
//!       ├─► EKF predict           (M1.9c: predict_step)
//!       │      │
//!       │      ▼
//!       │  estimated state + P
//!       │
//!       ├─► LPF (gyro, α)          (M2.0: LowPassFilterVec3)
//!       │
//!       ├─► INDI                   (M2.0: compute_torque_increment)
//!       │      │ virtual torque
//!       │      ▼
//!       └─► control allocation     (M2.1: allocate → motor thrusts)
//! ```

use algo_ekf::{
    BaroMeasurement, Covariance, GpsMeasurement, ImuMeasurement, MagMeasurement, ProcessNoise,
    State, baro_update, gps_update, mag_update, predict_step_with_drag,
};
use algo_fdir::{HealthLevel, SensorRejectionCounter};
use algo_indi::{
    AttitudeGain, IndiInput, Inertia, LowPassFilterVec3, RateCommand, RateGain, attitude_to_rate,
    compute_torque_increment,
};
use algo_nmpc::{PositionGains, Setpoint, position_to_attitude_thrust_pi};
use core_hal::traits::ImuSample;
use nalgebra::{Matrix4, SVector, Vector3};

pub use algo_alloc::{
    MotorGeometry, VirtualCommand, allocate, build_effectiveness, invert_quad_effectiveness,
    saturate, standard_x_quad,
};

/// Per-flight tunable parameters that are static at boot.
#[derive(Clone, Debug)]
pub struct RateLoopConfig {
    pub k_rate: RateGain,
    pub k_attitude: AttitudeGain,
    pub inertia: Inertia,
    pub mass_kg: f32,
    pub process_noise: ProcessNoise,
    pub gyro_lpf: LowPassFilterVec3,
    pub alpha_lpf: LowPassFilterVec3,
    /// Pre-computed effectiveness inverse for the 4-motor configuration.
    pub e_inv: Matrix4<f32>,
    /// Per-motor thrust limits (N).
    pub motor_min_n: f32,
    pub motor_max_n: f32,
    /// Nominal hover thrust (N) — used when no thrust command is given.
    pub hover_thrust_n: f32,
    /// Outer-loop gains (position / velocity).
    pub position_gains: PositionGains,
    /// Feed-forward gain applied to the EKF's estimated wind before it
    /// is added to the commanded acceleration. Roughly `k_drag / mass`
    /// for the airframe — set to zero to disable wind FF.
    pub wind_ff_gain: f32,
    /// Drag-over-mass (s⁻¹) fed into the EKF predict step so the filter's
    /// own velocity propagation accounts for aerodynamic drag. Same
    /// physical parameter as `wind_ff_gain`; typically they're equal.
    pub drag_over_mass_hz: f32,
    /// Commanded descent rate (m/s, positive = downward in NED) while
    /// [`LandingState::Landing`] is active.
    pub landing_descent_mps: f32,
    /// Commanded climb rate (m/s, magnitude — NED conversion handled
    /// internally) while [`TakeoffState::TakingOff`] is active.
    pub takeoff_climb_mps: f32,
    /// RTL cruise altitude (m above home, positive). Vehicle climbs to
    /// this altitude before transiting horizontally to home.
    pub rtl_safe_alt_m: f32,
    /// Tolerance (m) for "we've reached the RTL target xy". Smaller →
    /// tighter station-keeping but more time spent near home; larger →
    /// early hand-off to Landing.
    pub rtl_xy_tolerance_m: f32,
}

/// Reason an arm / takeoff request was refused by the preflight check.
///
/// Kept as a small `#[repr(u8)]` enum so it can be logged, surfaced in a
/// `COMMAND_ACK`, or matched on in firmware without allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PreflightReject {
    /// GPS health is not `Healthy`.
    GpsUnhealthy = 1,
    /// Barometer health is not `Healthy`.
    BaroUnhealthy = 2,
    /// Magnetometer health is not `Healthy`.
    MagUnhealthy = 3,
    /// EKF has not converged — position covariance too large.
    EkfNotConverged = 4,
}

impl PreflightReject {
    /// Short human-readable label for logs / MAVLink STATUSTEXT.
    #[must_use]
    pub const fn reason_str(self) -> &'static str {
        match self {
            Self::GpsUnhealthy => "GPS unhealthy",
            Self::BaroUnhealthy => "baro unhealthy",
            Self::MagUnhealthy => "mag unhealthy",
            Self::EkfNotConverged => "EKF not converged",
        }
    }
}

/// Position covariance (trace of the 3×3 position block in `P`) above
/// this value is treated as "not converged" — a freshly-initialised EKF
/// has `initial_sigma²` on each diagonal which is well above this, and a
/// properly converged filter sits around 0.1–1 m² trace.
pub const PREFLIGHT_POS_TRACE_MAX_M2: f32 = 5.0;

/// Run every safety check that should gate arm / takeoff. Returns the
/// first failing reason, or `Ok(())` if the vehicle is cleared to fly.
///
/// This is intentionally *read-only* — it never mutates `flight` so
/// callers can safely probe before accepting a COMMAND_LONG.
pub fn preflight_check(flight: &FlightState) -> Result<(), PreflightReject> {
    if flight.gps_health.level() != HealthLevel::Healthy {
        return Err(PreflightReject::GpsUnhealthy);
    }
    if flight.baro_health.level() != HealthLevel::Healthy {
        return Err(PreflightReject::BaroUnhealthy);
    }
    if flight.mag_health.level() != HealthLevel::Healthy {
        return Err(PreflightReject::MagUnhealthy);
    }
    let pos_trace = (0..algo_ekf::idx::P_NED_LEN)
        .map(|i| {
            flight
                .covariance
                .fixed_view::<1, 1>(
                    algo_ekf::idx::P_NED_START + i,
                    algo_ekf::idx::P_NED_START + i,
                )
                .to_scalar()
        })
        .sum::<f32>();
    if pos_trace > PREFLIGHT_POS_TRACE_MAX_M2 {
        return Err(PreflightReject::EkfNotConverged);
    }
    Ok(())
}

/// Whether the vehicle is allowed to drive its motors.
///
/// Disarmed is the safe default: every rate-loop step short-circuits to
/// zero motor thrust, even if the controller commands something non-zero.
/// Required by all GCS (QGC, MP, MAVROS) before flight.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ArmState {
    #[default]
    Disarmed,
    Armed,
}

/// Automatic-landing state machine.
///
/// `Idle`: vehicle follows whatever position setpoint the caller passes.
/// `Landing`: `outer_step` overrides the setpoint to descend in place at
/// `landing_descent_mps`, monitors [`TouchdownDetector`], and auto-disarms
/// when touchdown is detected.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LandingState {
    #[default]
    Idle,
    Landing,
}

/// Outcome of [`LandingState::advance`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LandingTransition {
    /// No change — caller continues feeding the landing setpoint.
    Stay,
    /// Touchdown detector fired — caller disarms, resets the
    /// landing state to Idle, and resets the detector's accumulator.
    Complete,
}

impl LandingState {
    /// Pure transition. Given the current vehicle velocity + position
    /// and the detector's accumulator state, returns what `outer_step`
    /// should do next.
    ///
    /// Advancing the detector is an unavoidable side effect (the
    /// accumulator has to grow or reset), so the function takes a
    /// `&mut TouchdownDetector`. The `LandingState` value itself is
    /// handed in by value and never mutated — the return enum carries
    /// the directive instead. Kani can still reason about the
    /// transition shape because the detector's observable result
    /// (touchdown yes/no) is a pure function of its inputs.
    pub fn advance(
        self,
        detector: &mut TouchdownDetector,
        velocity_ned: Vector3<f32>,
        position_z_ned: f32,
        dt_s: f32,
    ) -> LandingTransition {
        if matches!(self, Self::Idle) {
            return LandingTransition::Stay;
        }
        if detector.observe(velocity_ned, position_z_ned, dt_s) {
            LandingTransition::Complete
        } else {
            LandingTransition::Stay
        }
    }
}

/// Automatic return-to-launch state machine.
///
/// A 3-phase sequence driven by a single external trigger
/// (`MAV_CMD_NAV_RETURN_TO_LAUNCH`):
///
///   1. `Climbing` — ascend to `rtl_safe_alt_m` above home if currently below it. Skipped if already at / above safe alt.
///   2. `Returning` — cruise to the home xy while holding `rtl_safe_alt_m`.
///   3. Automatic hand-off to [`LandingState::Landing`] which runs the existing touchdown-detect + auto-disarm flow.
///
/// `Idle` is the default; `outer_step` drops back to `Idle` as soon as
/// the landing hand-off is made, so the caller's setpoint resumes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RtlPhase {
    #[default]
    Idle,
    Climbing,
    Returning,
}

/// Outcome of [`RtlPhase::advance`]. Kept as a small enum (rather than a
/// mutable `&mut RtlPhase`) so Kani can reason about the transition
/// shape without touching the rest of `FlightState`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RtlTransition {
    /// Caller keeps the current phase unchanged.
    Stay,
    /// Caller overwrites the current phase with the contained value.
    Advance(RtlPhase),
    /// Returning phase completed — caller switches the RTL state to
    /// `Idle` and starts [`LandingState::Landing`].
    Handoff,
}

impl RtlPhase {
    /// Pure transition function. Takes current phase + vehicle state +
    /// relevant config, returns what the caller should do. No mutation,
    /// no IO, no floats beyond the well-defined comparison arithmetic
    /// (sqrt was removed by comparing squared distance so Kani can
    /// reason about the whole function).
    #[must_use]
    pub fn advance(
        self,
        home_position_ned: Option<Vector3<f32>>,
        current_position_ned: Vector3<f32>,
        rtl_safe_alt_m: f32,
        rtl_xy_tolerance_m: f32,
    ) -> RtlTransition {
        // Idle is a sink — the machine never advances from Idle
        // autonomously; `Climbing` / `Returning` are caller-set by
        // external commands (MAVLink NAV_RETURN_TO_LAUNCH).
        if matches!(self, Self::Idle) {
            return RtlTransition::Stay;
        }
        // RTL without a latched home is a no-op.
        let Some(home) = home_position_ned else {
            return RtlTransition::Advance(Self::Idle);
        };
        let safe_z = home.z - rtl_safe_alt_m;
        match self {
            Self::Climbing => {
                // Climbing → Returning once we're at or above safe alt.
                // In NED z grows downward, so "high enough" means z is
                // less (more negative) than safe_z + some margin.
                if current_position_ned.z <= safe_z + 0.3 {
                    RtlTransition::Advance(Self::Returning)
                } else {
                    RtlTransition::Stay
                }
            }
            Self::Returning => {
                // Returning → Handoff once we're inside the xy
                // tolerance of home. Compare squared distance to
                // avoid sqrt — Kani has trouble reasoning about
                // `libm::sqrtf` symbolically.
                let dx = current_position_ned.x - home.x;
                let dy = current_position_ned.y - home.y;
                let dist_sq = dx * dx + dy * dy;
                let tol_sq = rtl_xy_tolerance_m * rtl_xy_tolerance_m;
                if dist_sq < tol_sq {
                    RtlTransition::Handoff
                } else {
                    RtlTransition::Stay
                }
            }
            // Unreachable — early-return above — but the compiler
            // needs the arm to make the match exhaustive.
            Self::Idle => RtlTransition::Stay,
        }
    }
}

/// Automatic climb-to-altitude state machine.
///
/// `Idle`: vehicle follows the caller's setpoint as usual.
/// `TakingOff { target_z_ned }`: `outer_step` holds home x/y and pushes
/// the z setpoint toward `target_z_ned`. Completes when the vehicle
/// sits within 0.3 m of target for 1 s, at which point state returns
/// to `Idle` — the caller then resumes providing setpoints.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum TakeoffState {
    #[default]
    Idle,
    TakingOff {
        target_z_ned: f32,
    },
}

/// Outcome of [`TakeoffState::advance`]. Same shape as
/// [`LandingTransition`] — one-way exit via `Reached`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TakeoffTransition {
    /// No change — caller keeps driving the climb setpoint.
    Stay,
    /// Altitude-reached detector fired — caller resets the takeoff
    /// state to `Idle` and resets the detector's accumulator. Caller
    /// decides whether the setpoint shifts to the target or whatever
    /// the outer navigator supplies next.
    Reached,
}

impl TakeoffState {
    /// Pure transition. `Idle` never advances autonomously;
    /// `TakingOff { target_z_ned }` advances to `Reached` once the
    /// altitude-reached detector sees the vehicle settled near the
    /// commanded altitude.
    pub fn advance(
        self,
        detector: &mut AltitudeReachedDetector,
        position_z_ned: f32,
        velocity_z_ned: f32,
        dt_s: f32,
    ) -> TakeoffTransition {
        let target_z_ned = match self {
            Self::Idle => return TakeoffTransition::Stay,
            Self::TakingOff { target_z_ned } => target_z_ned,
        };
        if detector.observe(position_z_ned, velocity_z_ned, target_z_ned, dt_s) {
            TakeoffTransition::Reached
        } else {
            TakeoffTransition::Stay
        }
    }
}

/// Detects that the vehicle has reached and settled at its takeoff
/// altitude. Parallels [`TouchdownDetector`] but checks "close to
/// commanded z" rather than "close to ground".
#[derive(Clone, Copy, Debug, Default)]
pub struct AltitudeReachedDetector {
    settled_s: f32,
}

impl AltitudeReachedDetector {
    pub const Z_TOLERANCE_M: f32 = 0.3;
    pub const VZ_TOLERANCE_MPS: f32 = 0.3;
    pub const SETTLE_TIME_S: f32 = 1.0;

    #[must_use]
    pub const fn new() -> Self {
        Self { settled_s: 0.0 }
    }

    pub fn reset(&mut self) {
        self.settled_s = 0.0;
    }

    pub fn observe(
        &mut self,
        position_z_ned: f32,
        velocity_z_ned: f32,
        target_z_ned: f32,
        dt_s: f32,
    ) -> bool {
        let alt_ok = (position_z_ned - target_z_ned).abs() < Self::Z_TOLERANCE_M;
        let vz_ok = velocity_z_ned.abs() < Self::VZ_TOLERANCE_MPS;
        if alt_ok && vz_ok {
            self.settled_s += dt_s;
        } else {
            self.settled_s = 0.0;
        }
        self.settled_s >= Self::SETTLE_TIME_S
    }
}

/// Detects that the vehicle has come to rest on the ground.
///
/// Requires simultaneously:
///   * `|v_z| < VZ_THRESHOLD_MPS` (not descending any more)
///   * `‖v_xy‖ < VXY_THRESHOLD_MPS` (not sliding)
///   * `p_z_ned > Z_THRESHOLD_M` (near the ground; NED z=0 is ground, z<0 airborne)
///
/// All three held continuously for `SETTLE_TIME_S` ⇒ `observe` returns
/// `true`. Any miss resets the accumulator to zero.
#[derive(Clone, Copy, Debug, Default)]
pub struct TouchdownDetector {
    settled_s: f32,
}

impl TouchdownDetector {
    pub const VZ_THRESHOLD_MPS: f32 = 0.15;
    pub const VXY_THRESHOLD_MPS: f32 = 0.5;
    pub const Z_THRESHOLD_M: f32 = -0.30;
    pub const SETTLE_TIME_S: f32 = 1.0;

    #[must_use]
    pub const fn new() -> Self {
        Self { settled_s: 0.0 }
    }

    pub fn reset(&mut self) {
        self.settled_s = 0.0;
    }

    pub fn observe(&mut self, velocity_ned: Vector3<f32>, position_z_ned: f32, dt_s: f32) -> bool {
        let vertical_quiet = velocity_ned.z.abs() < Self::VZ_THRESHOLD_MPS;
        // Squared comparison avoids libm::sqrtf so the whole method
        // becomes Kani-tractable — see the `sqrtf`-removal rationale in
        // `RtlPhase::advance`.
        let horizontal_sq = velocity_ned.x * velocity_ned.x + velocity_ned.y * velocity_ned.y;
        let horizontal_quiet = horizontal_sq < Self::VXY_THRESHOLD_MPS * Self::VXY_THRESHOLD_MPS;
        let near_ground = position_z_ned > Self::Z_THRESHOLD_M;
        if vertical_quiet && horizontal_quiet && near_ground {
            self.settled_s += dt_s;
        } else {
            self.settled_s = 0.0;
        }
        self.settled_s >= Self::SETTLE_TIME_S
    }
}

/// Dynamic flight state held across iterations.
#[derive(Clone, Copy, Debug)]
pub struct FlightState {
    pub state: State,
    pub covariance: Covariance,
    pub last_gyro_filtered: Vector3<f32>,
    /// Per-sensor rejection counters; `apply_*_measurement` updates them.
    pub gps_health: SensorRejectionCounter,
    pub mag_health: SensorRejectionCounter,
    pub baro_health: SensorRejectionCounter,
    /// Velocity-loop integrator (anti-windup-bounded). Advanced by
    /// [`outer_step`] each call.
    pub vel_integrator: Vector3<f32>,
    /// Arm state; `Disarmed` forces motor output to zero regardless of
    /// controller output.
    pub arm_state: ArmState,
    /// Automatic-landing mode. `Landing` overrides the caller's setpoint
    /// with a descending ramp and auto-disarms on touchdown.
    pub landing_state: LandingState,
    /// Touchdown detector; advanced by `outer_step` while landing.
    pub touchdown_detector: TouchdownDetector,
    /// Automatic-takeoff mode.
    pub takeoff_state: TakeoffState,
    /// Altitude-reached detector, advanced by `outer_step` while in
    /// `TakingOff`.
    pub altitude_reached_detector: AltitudeReachedDetector,
    /// Position latched at first arm. Used as the reference for TAKEOFF
    /// (climb target = home + (0,0,-alt)) and RTL (return target = home
    /// xy). `None` until the vehicle has been armed once.
    pub home_position_ned: Option<Vector3<f32>>,
    /// Return-to-launch phase.
    pub rtl_phase: RtlPhase,
}

impl Default for FlightState {
    fn default() -> Self {
        Self {
            state: State::default(),
            covariance: algo_ekf::initial_covariance(),
            last_gyro_filtered: Vector3::zeros(),
            gps_health: SensorRejectionCounter::new(),
            mag_health: SensorRejectionCounter::new(),
            baro_health: SensorRejectionCounter::new(),
            vel_integrator: Vector3::zeros(),
            arm_state: ArmState::default(),
            landing_state: LandingState::default(),
            touchdown_detector: TouchdownDetector::new(),
            takeoff_state: TakeoffState::default(),
            altitude_reached_detector: AltitudeReachedDetector::new(),
            home_position_ned: None,
            rtl_phase: RtlPhase::default(),
        }
    }
}

impl FlightState {
    /// Roll-up of every sensor stream's health level (worst wins).
    #[must_use]
    pub fn overall_health(&self) -> HealthLevel {
        [
            self.gps_health.level(),
            self.mag_health.level(),
            self.baro_health.level(),
        ]
        .into_iter()
        .max_by_key(|l| l.severity())
        .unwrap_or(HealthLevel::Healthy)
    }
}

/// Output of one rate-loop iteration.
#[derive(Clone, Copy, Debug)]
pub struct RateLoopOutput {
    /// Motor thrusts after saturation (N). Indexing matches the motor
    /// array passed to [`RateLoopConfig::e_inv`] at construction.
    pub motor_thrusts_n: SVector<f32, 4>,
    /// Virtual command that was sent to the allocator.
    pub virtual_cmd: VirtualCommand,
}

/// Single iteration of the inner rate loop.
///
/// Advances the EKF by one IMU step, filters ω and finite-differences α,
/// runs INDI to turn the rate setpoint into a virtual torque, and lets
/// the allocator pick motor thrusts.
pub fn rate_loop_step(
    cfg: &mut RateLoopConfig,
    flight: &mut FlightState,
    imu: ImuSample,
    dt_s: f32,
    rate_cmd: RateCommand,
) -> RateLoopOutput {
    // -- 1. EKF predict using the raw IMU sample --------------------------
    let measurement = ImuMeasurement {
        gyro_rad_s: imu.gyro_rad_s,
        accel_m_s2: imu.accel_m_s2,
    };
    let (next_state, next_cov) = predict_step_with_drag(
        &flight.state,
        &flight.covariance,
        measurement,
        cfg.process_noise,
        dt_s,
        cfg.drag_over_mass_hz,
    );
    flight.state = next_state;
    flight.covariance = next_cov;

    // -- 2. Filter gyro, compute angular acceleration α = dω/dt -----------
    let gyro_filtered = cfg.gyro_lpf.update(imu.gyro_rad_s);
    let omega_dot_raw = if dt_s > 0.0 {
        (gyro_filtered - flight.last_gyro_filtered) / dt_s
    } else {
        Vector3::zeros()
    };
    let omega_dot_filtered = cfg.alpha_lpf.update(omega_dot_raw);
    flight.last_gyro_filtered = gyro_filtered;

    // -- 3. INDI step ------------------------------------------------------
    let indi_input = IndiInput {
        cmd: rate_cmd,
        omega_filtered: gyro_filtered,
        omega_dot_filtered,
        k_rate: &cfg.k_rate,
        inertia: &cfg.inertia,
    };
    let torque = compute_torque_increment(&indi_input);

    // -- 4. Control allocation --------------------------------------------
    let virtual_cmd = VirtualCommand {
        torque: torque.body_torque_nm,
        thrust_n: cfg.hover_thrust_n,
    };
    let raw = allocate(&cfg.e_inv, &virtual_cmd);
    let clipped = saturate(raw, cfg.motor_min_n, cfg.motor_max_n);

    // Arm-state gate: disarmed vehicles emit zero thrust unconditionally.
    let final_thrusts = match flight.arm_state {
        ArmState::Armed => clipped,
        ArmState::Disarmed => SVector::<f32, 4>::zeros(),
    };

    RateLoopOutput {
        motor_thrusts_n: final_thrusts,
        virtual_cmd,
    }
}

/// Build a reasonable default configuration for a 250 g quadrotor.
/// Intended for tests, SITL bootstrapping, and documentation examples.
#[must_use]
pub fn default_config_250g() -> RateLoopConfig {
    let motors = standard_x_quad(0.15, 0.016);
    let e = build_effectiveness(&motors);
    let e_mat: Matrix4<f32> = e.into_owned();
    let e_inv = invert_quad_effectiveness(&e_mat).unwrap_or(Matrix4::identity());
    RateLoopConfig {
        k_rate: Vector3::new(25.0, 25.0, 15.0),
        k_attitude: Vector3::new(8.0, 8.0, 5.0),
        inertia: nalgebra::Matrix3::from_diagonal(&Vector3::new(0.015, 0.015, 0.025)),
        mass_kg: 0.25,
        process_noise: ProcessNoise::default(),
        gyro_lpf: LowPassFilterVec3::new(80.0, 1000.0),
        alpha_lpf: LowPassFilterVec3::new(30.0, 1000.0),
        e_inv,
        motor_min_n: 0.0,
        motor_max_n: 6.0,
        hover_thrust_n: 2.45, // m=0.25 kg × g=9.8 m/s²
        position_gains: PositionGains {
            k_i_vel: Vector3::new(0.5, 0.5, 0.8),
            ..PositionGains::default()
        },
        // k_drag ≈ 0.05 N·s/m, mass 0.25 kg → ~0.2 s⁻¹ scales wind (m/s) to
        // the drag-canceling accel (m/s²). Set 0 to disable.
        wind_ff_gain: 0.2,
        // EKF-side drag. Matching Jacobian lives in
        // `build_transition_jacobian_with_drag` (M4.0). Default still 0
        // — the PI integrator (M3.2) is tuned against a drag-free EKF
        // predict, and enabling drag by default regresses the
        // drag-free SITL hover. End-to-end wind-identification tests
        // set this explicitly.
        drag_over_mass_hz: 0.0,
        // 0.5 m/s descent while landing — conservative, gives the EKF
        // time to notice touchdown before the controller cranks up
        // thrust to chase the ground.
        landing_descent_mps: 0.5,
        // 1.0 m/s climb during automatic takeoff — quick enough to feel
        // responsive, slow enough to stay well within the vehicle's
        // thrust envelope at any payload.
        takeoff_climb_mps: 1.0,
        // 10 m RTL altitude clears typical obstacles (treeline, single-
        // story buildings). Operators should raise this for forested
        // environments.
        rtl_safe_alt_m: 10.0,
        // 1 m "we're home" threshold. Within this → hand off to Landing.
        rtl_xy_tolerance_m: 1.0,
    }
}

// ----------------------------------------------------------------------------
// Measurement-update hooks
// ----------------------------------------------------------------------------

/// Fold a GPS observation into the filter and update the GPS health
/// counter. Returns the computed NIS so the caller can log it.
pub fn apply_gps_measurement(flight: &mut FlightState, measurement: &GpsMeasurement) -> f32 {
    let r = gps_update(&flight.state, &flight.covariance, measurement);
    if r.applied {
        flight.state = r.state;
        flight.covariance = r.covariance;
    }
    flight.gps_health.observe(r.applied);
    r.nis
}

/// Fold a magnetometer observation into the filter.
pub fn apply_mag_measurement(flight: &mut FlightState, measurement: &MagMeasurement) -> f32 {
    let r = mag_update(&flight.state, &flight.covariance, measurement);
    if r.applied {
        flight.state = r.state;
        flight.covariance = r.covariance;
    }
    flight.mag_health.observe(r.applied);
    r.nis
}

/// Fold a barometer observation into the filter.
pub fn apply_baro_measurement(flight: &mut FlightState, measurement: &BaroMeasurement) -> f32 {
    let r = baro_update(&flight.state, &flight.covariance, measurement);
    if r.applied {
        flight.state = r.state;
        flight.covariance = r.covariance;
    }
    flight.baro_health.observe(r.applied);
    r.nis
}

/// Outer-to-inner single-step control.
///
/// Runs `position_to_attitude_thrust` → `attitude_to_rate` →
/// `rate_loop_step` in sequence. The caller passes a position setpoint;
/// this function handles everything down to motor thrusts.
///
/// The commanded thrust from the outer loop overrides the
/// `hover_thrust_n` fallback — `rate_loop_step` sees the outer loop's
/// actual thrust demand so climbing / descending works.
pub fn outer_step(
    cfg: &mut RateLoopConfig,
    flight: &mut FlightState,
    imu: ImuSample,
    dt_s: f32,
    setpoint: &Setpoint,
) -> RateLoopOutput {
    // Home-latch: record the EKF position on first Armed state.
    if flight.arm_state == ArmState::Armed && flight.home_position_ned.is_none() {
        flight.home_position_ned = Some(flight.state.position_ned);
    }

    // RTL phase transition — pure function, apply its directive here.
    match flight.rtl_phase.advance(
        flight.home_position_ned,
        flight.state.position_ned,
        cfg.rtl_safe_alt_m,
        cfg.rtl_xy_tolerance_m,
    ) {
        RtlTransition::Stay => {}
        RtlTransition::Advance(next) => {
            flight.rtl_phase = next;
        }
        RtlTransition::Handoff => {
            flight.rtl_phase = RtlPhase::Idle;
            flight.landing_state = LandingState::Landing;
        }
    }

    // Landing and Takeoff are mutually exclusive mode overrides on the
    // setpoint. Landing dominates (safer — "descend and disarm" should
    // always win over "climb"). Otherwise takeoff drives to target_z.
    // RTL is layered on top: when RTL is active it overrides the caller
    // setpoint (if not already in Landing) to drive to the safe alt +
    // home xy.
    let override_sp;
    let active_sp: &Setpoint = if flight.landing_state == LandingState::Landing {
        let current_z = flight.state.position_ned.z;
        let target_z = (current_z + cfg.landing_descent_mps * dt_s).min(0.0);
        override_sp = Setpoint {
            position_ned: Vector3::new(
                flight.state.position_ned.x,
                flight.state.position_ned.y,
                target_z,
            ),
            velocity_ned: Vector3::new(0.0, 0.0, cfg.landing_descent_mps),
            accel_ned: Vector3::zeros(),
            yaw_rad: setpoint.yaw_rad,
        };
        &override_sp
    } else if let TakeoffState::TakingOff { target_z_ned } = flight.takeoff_state {
        // Hold home xy (or current xy if home not set); drive z toward
        // target_z_ned. Velocity FF at commanded climb rate.
        let home_xy = flight
            .home_position_ned
            .map_or_else(|| flight.state.position_ned, |h| h);
        let climb_v = -cfg.takeoff_climb_mps; // NED: climbing = negative z
        override_sp = Setpoint {
            position_ned: Vector3::new(home_xy.x, home_xy.y, target_z_ned),
            velocity_ned: Vector3::new(0.0, 0.0, climb_v),
            accel_ned: Vector3::zeros(),
            yaw_rad: setpoint.yaw_rad,
        };
        &override_sp
    } else if flight.rtl_phase != RtlPhase::Idle {
        let home = flight
            .home_position_ned
            .unwrap_or(flight.state.position_ned);
        let safe_z = home.z - cfg.rtl_safe_alt_m;
        let target_xy = match flight.rtl_phase {
            RtlPhase::Climbing => (flight.state.position_ned.x, flight.state.position_ned.y),
            _ => (home.x, home.y),
        };
        override_sp = Setpoint {
            position_ned: Vector3::new(target_xy.0, target_xy.1, safe_z),
            velocity_ned: Vector3::zeros(),
            accel_ned: Vector3::zeros(),
            yaw_rad: setpoint.yaw_rad,
        };
        &override_sp
    } else {
        setpoint
    };

    // Wind feed-forward: add k·wind_estimate to the setpoint's accel
    // feedforward channel. Zero-latency disturbance compensation that
    // complements the velocity-loop integrator.
    let wind_ff = Vector3::new(
        flight.state.wind_ne.x * cfg.wind_ff_gain,
        flight.state.wind_ne.y * cfg.wind_ff_gain,
        0.0,
    );
    let adjusted_setpoint = Setpoint {
        accel_ned: active_sp.accel_ned + wind_ff,
        ..*active_sp
    };

    // Outer loop: position → (q_desired, thrust). PI cascade, so we
    // carry the integrator forward through flight.vel_integrator.
    let (att, new_integ) = position_to_attitude_thrust_pi(
        &adjusted_setpoint,
        flight.state.position_ned,
        flight.state.velocity_ned,
        cfg.mass_kg,
        &cfg.position_gains,
        flight.vel_integrator,
        dt_s,
    );
    flight.vel_integrator = new_integ;

    // Inner (middle): q_desired → rate command.
    let rate_cmd = attitude_to_rate(flight.state.attitude, att.q_desired, &cfg.k_attitude);

    // Swap the hover fallback for the live thrust demand for this tick.
    let saved_hover = cfg.hover_thrust_n;
    cfg.hover_thrust_n = att.thrust_n;
    let out = rate_loop_step(cfg, flight, imu, dt_s, rate_cmd);
    cfg.hover_thrust_n = saved_hover;

    // Auto-disarm on touchdown — pure transition fn + side effects.
    match flight.landing_state.advance(
        &mut flight.touchdown_detector,
        flight.state.velocity_ned,
        flight.state.position_ned.z,
        dt_s,
    ) {
        LandingTransition::Stay => {}
        LandingTransition::Complete => {
            flight.arm_state = ArmState::Disarmed;
            flight.landing_state = LandingState::Idle;
            flight.touchdown_detector.reset();
        }
    }

    // Takeoff completion — pure transition fn + side effects.
    match flight.takeoff_state.advance(
        &mut flight.altitude_reached_detector,
        flight.state.position_ned.z,
        flight.state.velocity_ned.z,
        dt_s,
    ) {
        TakeoffTransition::Stay => {}
        TakeoffTransition::Reached => {
            flight.takeoff_state = TakeoffState::Idle;
            flight.altitude_reached_detector.reset();
        }
    }

    out
}

// ----------------------------------------------------------------------------
// Formal verification — RtlPhase transition-shape invariants (M16a)
// ----------------------------------------------------------------------------
//
// Proves the state-machine shape is what we say it is. Crucially these
// harnesses do NOT exercise `sqrt` or any other float-heavy op — the
// `advance` function is designed so squared distance stands in for
// actual distance, keeping every branch Kani-tractable.
#[cfg(kani)]
mod rtl_kani {
    use super::*;

    fn any_phase() -> RtlPhase {
        let t: u8 = kani::any();
        kani::assume(t < 3);
        match t {
            0 => RtlPhase::Idle,
            1 => RtlPhase::Climbing,
            _ => RtlPhase::Returning,
        }
    }

    /// `kani::any::<f32>()` spans every bit pattern, including NaN.
    /// A real EKF + MPC pipeline never feeds NaN into the RTL
    /// transition — Kani's job is to verify logical shape, not
    /// rediscover that `NaN ⊕ NaN = NaN`. Assume finiteness at the
    /// harness boundary so the proofs focus on the state-machine
    /// invariants.
    fn any_finite_f32() -> f32 {
        let x: f32 = kani::any();
        kani::assume(x.is_finite());
        x
    }

    fn any_finite_vec() -> Vector3<f32> {
        Vector3::new(any_finite_f32(), any_finite_f32(), any_finite_f32())
    }

    /// Idle is an absorbing state under `advance`. No vehicle state +
    /// no config can push it to Climbing / Returning / Handoff —
    /// those transitions only come from external commands setting
    /// `flight.rtl_phase` directly.
    #[kani::proof]
    fn idle_is_absorbing() {
        let home_present: bool = kani::any();
        let home = if home_present {
            Some(any_finite_vec())
        } else {
            None
        };
        let pos = any_finite_vec();
        let safe_alt = any_finite_f32();
        let tol = any_finite_f32();
        let r = RtlPhase::Idle.advance(home, pos, safe_alt, tol);
        assert!(matches!(r, RtlTransition::Stay));
    }

    /// RTL with no home latched is always cancelled — no vehicle
    /// position can change that (we can't navigate home if we never
    /// knew where home was).
    #[kani::proof]
    fn no_home_cancels_rtl_to_idle() {
        let phase = any_phase();
        // Only Climbing / Returning are state-changeable here; Idle is
        // covered by `idle_is_absorbing`.
        kani::assume(matches!(phase, RtlPhase::Climbing | RtlPhase::Returning));
        let pos = any_finite_vec();
        let safe_alt = any_finite_f32();
        let tol = any_finite_f32();
        let r = phase.advance(None, pos, safe_alt, tol);
        assert!(matches!(r, RtlTransition::Advance(RtlPhase::Idle)));
    }

    /// Climbing can only `Stay` or `Advance(Returning)`. It never
    /// skips directly to `Handoff` (which would be "teleport to
    /// ground"). Proves `Climbing → Returning → Handoff` is the only
    /// complete path.
    #[kani::proof]
    fn climbing_never_handoffs_directly() {
        let home = any_finite_vec();
        let pos = any_finite_vec();
        let safe_alt = any_finite_f32();
        let tol = any_finite_f32();
        let r = RtlPhase::Climbing.advance(Some(home), pos, safe_alt, tol);
        match r {
            RtlTransition::Stay => {}
            RtlTransition::Advance(next) => {
                // Climbing can only advance *to* Returning, never back
                // to Climbing or to Idle.
                assert!(matches!(next, RtlPhase::Returning));
            }
            RtlTransition::Handoff => {
                // Must never happen from Climbing.
                panic!("Climbing must not produce Handoff");
            }
        }
    }

    /// Returning can only `Stay` or `Handoff`. It never goes back to
    /// Climbing or sideways to Idle — the only exit is "we're home,
    /// start landing".
    // ---- LandingState (M16b) -----------------------------------------

    /// `LandingState::Idle` cannot auto-advance to `Landing` — the only
    /// path in is an explicit MAVLink NAV_LAND or RTL handoff flipping
    /// `flight.landing_state = Landing` externally.
    #[kani::proof]
    fn landing_idle_is_absorbing() {
        let mut det = TouchdownDetector::new();
        // Detector accumulator state is arbitrary too; idle must
        // still be absorbing.
        det.settled_s = any_finite_f32();
        kani::assume(det.settled_s >= 0.0);
        let vel = any_finite_vec();
        let pos_z = any_finite_f32();
        let dt = any_finite_f32();
        let r = LandingState::Idle.advance(&mut det, vel, pos_z, dt);
        assert!(matches!(r, LandingTransition::Stay));
    }

    /// `LandingState::Landing` can only `Stay` or `Complete`. No
    /// state-space magic lets it teleport to something else.
    #[kani::proof]
    fn landing_only_exits_via_complete() {
        let mut det = TouchdownDetector::new();
        det.settled_s = any_finite_f32();
        kani::assume(det.settled_s >= 0.0);
        let vel = any_finite_vec();
        let pos_z = any_finite_f32();
        let dt = any_finite_f32();
        let r = LandingState::Landing.advance(&mut det, vel, pos_z, dt);
        assert!(matches!(
            r,
            LandingTransition::Stay | LandingTransition::Complete
        ));
    }

    /// Idle never advances the detector. The accumulator value the
    /// caller hands in survives unchanged — important because real
    /// flight toggles Idle → Landing → Idle repeatedly, and each new
    /// Landing has to start from a clean detector state via the
    /// outer_step `reset` call on Complete (not silently mid-Idle).
    // ---- TakeoffState (M16c) -----------------------------------------

    /// `TakeoffState::Idle` can't auto-advance. External commands
    /// (MAV_CMD_NAV_TAKEOFF) flip it to `TakingOff` directly.
    #[kani::proof]
    fn takeoff_idle_is_absorbing() {
        let mut det = AltitudeReachedDetector::new();
        det.settled_s = any_finite_f32();
        kani::assume(det.settled_s >= 0.0);
        let pos_z = any_finite_f32();
        let vel_z = any_finite_f32();
        let dt = any_finite_f32();
        let r = TakeoffState::Idle.advance(&mut det, pos_z, vel_z, dt);
        assert!(matches!(r, TakeoffTransition::Stay));
    }

    /// `TakeoffState::TakingOff` exits only via `Reached`. No phantom
    /// transition back to Idle (that's a side effect the caller
    /// performs on `Reached`, not something `advance` itself does).
    #[kani::proof]
    fn takeoff_only_exits_via_reached() {
        let mut det = AltitudeReachedDetector::new();
        det.settled_s = any_finite_f32();
        kani::assume(det.settled_s >= 0.0);
        let target_z = any_finite_f32();
        let pos_z = any_finite_f32();
        let vel_z = any_finite_f32();
        let dt = any_finite_f32();
        let r = TakeoffState::TakingOff { target_z_ned: target_z }
            .advance(&mut det, pos_z, vel_z, dt);
        assert!(matches!(r, TakeoffTransition::Stay | TakeoffTransition::Reached));
    }

    /// Idle leaves the altitude detector untouched. Symmetric to the
    /// Landing Idle-doesn't-accumulate invariant — ensures a stale
    /// detector value from a previous flight doesn't poison a fresh
    /// TakingOff that follows.
    #[kani::proof]
    fn takeoff_idle_does_not_touch_detector() {
        let mut det = AltitudeReachedDetector::new();
        let before = any_finite_f32();
        kani::assume(before >= 0.0);
        det.settled_s = before;
        let pos_z = any_finite_f32();
        let vel_z = any_finite_f32();
        let dt = any_finite_f32();
        let _ = TakeoffState::Idle.advance(&mut det, pos_z, vel_z, dt);
        assert!(det.settled_s == before);
    }

    #[kani::proof]
    fn landing_idle_does_not_touch_detector() {
        let mut det = TouchdownDetector::new();
        let before: f32 = any_finite_f32();
        kani::assume(before >= 0.0);
        det.settled_s = before;
        let vel = any_finite_vec();
        let pos_z = any_finite_f32();
        let dt = any_finite_f32();
        let _ = LandingState::Idle.advance(&mut det, vel, pos_z, dt);
        assert!(det.settled_s == before);
    }

    #[kani::proof]
    fn returning_never_advances_to_climbing_or_idle() {
        let home = any_finite_vec();
        let pos = any_finite_vec();
        let safe_alt = any_finite_f32();
        let tol = any_finite_f32();
        let r = RtlPhase::Returning.advance(Some(home), pos, safe_alt, tol);
        match r {
            RtlTransition::Stay => {}
            RtlTransition::Handoff => {}
            RtlTransition::Advance(next) => {
                // Must never happen from Returning — there's no
                // advance path.
                panic!("Returning must not Advance (got {:?})", next);
            }
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::expect_used,
    clippy::indexing_slicing
)]
mod tests {
    use super::*;

    fn make_imu(ts: u64, gyro: Vector3<f32>, accel: Vector3<f32>) -> ImuSample {
        ImuSample {
            timestamp_us: ts,
            gyro_rad_s: gyro,
            accel_m_s2: accel,
            temperature_c: 20.0,
        }
    }

    fn stationary_imu(ts: u64) -> ImuSample {
        make_imu(
            ts,
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, -algo_ekf::GRAVITY_M_S2),
        )
    }

    #[test]
    fn rate_loop_runs_one_step() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let imu = stationary_imu(0);
        let out = rate_loop_step(&mut cfg, &mut flight, imu, 0.001, RateCommand::default());
        // With zero rate cmd and stationary IMU, motor thrusts should all be
        // close to hover/4 = 0.6125 N.
        for i in 0..4 {
            let t = out.motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar();
            assert!(t >= 0.0 && t <= cfg.motor_max_n);
            assert!((t - cfg.hover_thrust_n / 4.0).abs() < 0.1);
        }
    }

    #[test]
    fn rate_loop_stationary_stays_stable_for_one_second() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
        for i in 0..1000u64 {
            let imu = stationary_imu(i);
            let _ = rate_loop_step(&mut cfg, &mut flight, imu, 0.001, RateCommand::default());
        }
        // EKF: quaternion unit.
        assert!((flight.state.attitude.norm() - 1.0).abs() < 1.0e-4);
        // Covariance diagonals positive.
        for i in 0..algo_ekf::STATE_DIM {
            let v = flight.covariance.fixed_view::<1, 1>(i, i).to_scalar();
            assert!(v > 0.0 && v.is_finite(), "P[{i},{i}]={v}");
        }
    }

    #[test]
    fn gps_measurement_pulls_state_and_updates_health() {
        let mut flight = FlightState::default();
        assert_eq!(flight.overall_health(), HealthLevel::Healthy);
        let m = algo_ekf::GpsMeasurement {
            position_ned: Vector3::new(1.0, -0.5, 0.3),
            sigma: Vector3::new(0.3, 0.3, 0.4),
        };
        let nis = apply_gps_measurement(&mut flight, &m);
        assert!(nis.is_finite());
        // Position must have moved toward measurement.
        assert!(flight.state.position_ned.norm() > 0.0);
        // A first accepted measurement keeps health Healthy.
        assert_eq!(flight.gps_health.level(), HealthLevel::Healthy);
    }

    #[test]
    fn repeated_gps_outliers_degrade_health() {
        let mut flight = FlightState::default();
        let bad = algo_ekf::GpsMeasurement {
            position_ned: Vector3::new(10_000.0, 0.0, 0.0),
            sigma: Vector3::new(0.5, 0.5, 0.5),
        };
        for _ in 0..15 {
            let _ = apply_gps_measurement(&mut flight, &bad);
        }
        assert_ne!(flight.gps_health.level(), HealthLevel::Healthy);
        assert_eq!(flight.overall_health(), flight.gps_health.level());
    }

    #[test]
    fn baro_measurement_updates_health_and_altitude() {
        let mut flight = FlightState::default();
        let m = algo_ekf::BaroMeasurement {
            altitude_m: 2.0,
            sigma_m: 0.1,
        };
        let _ = apply_baro_measurement(&mut flight, &m);
        // position_ned.z is negative-Down, altitude is -z.
        let altitude = -flight.state.position_ned.z;
        assert!(altitude > 0.0, "altitude did not climb: {altitude}");
    }

    #[test]
    fn mag_measurement_does_not_corrupt_attitude_at_alignment() {
        let mut flight = FlightState::default();
        // Body-frame reading = world mag_ned when q = identity.
        let m = algo_ekf::MagMeasurement {
            body_field: flight.state.mag_ned,
            sigma: Vector3::new(0.01, 0.01, 0.01),
        };
        let _ = apply_mag_measurement(&mut flight, &m);
        assert!((flight.state.attitude.norm() - 1.0).abs() < 1.0e-4);
    }

    #[test]
    fn outer_step_hover_at_origin_stays_stable() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
        let setpoint = Setpoint::default();
        for i in 0..1000u64 {
            let imu = stationary_imu(i);
            let _ = outer_step(&mut cfg, &mut flight, imu, 0.001, &setpoint);
        }
        // ‖q‖ unit, position / velocity bounded (synthetic IMU keeps vehicle
        // stationary; controller only has to *not destabilise* it).
        assert!((flight.state.attitude.norm() - 1.0).abs() < 1.0e-4);
        assert!(flight.state.position_ned.norm() < 0.05);
        assert!(flight.state.velocity_ned.norm() < 0.1);
    }

    #[test]
    fn rate_loop_drag_influences_predicted_velocity_when_wind_set() {
        // With a known wind in state.wind_ne and drag enabled, predict
        // should push the estimated velocity along the wind direction,
        // even with level-hover IMU reads.
        let mut cfg_on = default_config_250g();
        cfg_on.drag_over_mass_hz = 0.4;
        let mut cfg_off = default_config_250g();
        cfg_off.drag_over_mass_hz = 0.0;

        let make_state = || {
            let mut f = FlightState::default();
            f.state.wind_ne = nalgebra::Vector2::new(2.0, 0.0); // +x wind 2 m/s
            f
        };
        let mut flight_on = make_state();
        let mut flight_off = make_state();

        for i in 0..100u64 {
            let imu = stationary_imu(i);
            let _ = rate_loop_step(
                &mut cfg_on,
                &mut flight_on,
                imu,
                0.001,
                algo_indi::RateCommand::default(),
            );
            let _ = rate_loop_step(
                &mut cfg_off,
                &mut flight_off,
                imu,
                0.001,
                algo_indi::RateCommand::default(),
            );
        }
        // Drag-on branch should have accumulated some +x velocity; drag-off
        // branch should have (near) zero (gravity only).
        let vx_on = flight_on.state.velocity_ned.x;
        let vx_off = flight_off.state.velocity_ned.x;
        assert!(
            vx_on > vx_off + 0.01,
            "drag FF inactive: vx_on={vx_on} vx_off={vx_off}"
        );
    }

    #[test]
    fn outer_step_wind_feedforward_tilts_against_wind() {
        // With a known horizontal wind, the controller should tilt
        // *into* the wind to cancel drag — even at zero pos/vel error.
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        // Pretend the EKF has identified +3 m/s east wind.
        flight.state.wind_ne = nalgebra::Vector2::new(0.0, 3.0);
        let imu = stationary_imu(0);
        let setpoint = Setpoint::default();
        let out = outer_step(&mut cfg, &mut flight, imu, 0.001, &setpoint);

        // Same scenario but with wind FF disabled.
        let mut cfg_no_ff = default_config_250g();
        cfg_no_ff.wind_ff_gain = 0.0;
        let mut flight_no_ff = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        flight_no_ff.state.wind_ne = nalgebra::Vector2::new(0.0, 3.0);
        let out_no = outer_step(&mut cfg_no_ff, &mut flight_no_ff, imu, 0.001, &setpoint);

        // Without wind FF, controller can't feel the wind → zero east-axis
        // torque. With wind FF, east accel is commanded, so motor thrusts
        // differ by a measurable amount.
        let diff: f32 = (0..4)
            .map(|i| {
                (out.motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar()
                    - out_no.motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar())
                .abs()
            })
            .sum();
        assert!(
            diff > 1.0e-3,
            "wind FF had no effect on motors: diff={diff}"
        );
    }

    #[test]
    fn outer_step_altitude_setpoint_raises_thrust() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        // Climb to 1 m altitude (z = -1 in NED).
        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };
        let imu = stationary_imu(0);
        let out = outer_step(&mut cfg, &mut flight, imu, 0.001, &setpoint);
        // Total commanded thrust > hover baseline (2.45 N).
        let total: f32 = (0..4)
            .map(|i| out.motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar())
            .sum();
        assert!(total > 2.45, "total thrust {total} should exceed hover");
    }

    #[test]
    fn disarmed_vehicle_outputs_zero_thrust_regardless_of_command() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default(); // Disarmed by default
        assert_eq!(flight.arm_state, ArmState::Disarmed);
        // Aggressive setpoint that would normally saturate motors.
        let setpoint = Setpoint {
            position_ned: Vector3::new(100.0, 0.0, -10.0),
            ..Setpoint::default()
        };
        let imu = stationary_imu(0);
        let out = outer_step(&mut cfg, &mut flight, imu, 0.001, &setpoint);
        for i in 0..4 {
            let t = out.motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar();
            assert_eq!(t, 0.0, "disarmed motor {i} should be 0, got {t}");
        }
    }

    #[test]
    fn arming_enables_motor_output() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let imu = stationary_imu(0);
        let out = outer_step(&mut cfg, &mut flight, imu, 0.001, &Setpoint::default());
        // Armed + zero setpoint → hover thrust on each motor.
        let total: f32 = (0..4)
            .map(|i| out.motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar())
            .sum();
        assert!(total > 0.0);
    }

    #[test]
    fn touchdown_detector_fires_after_settle_time() {
        let mut td = TouchdownDetector::new();
        // Below thresholds, near ground — should accumulate.
        let v = Vector3::new(0.05, 0.05, 0.05);
        let z = -0.1_f32;
        let mut fired = false;
        // 1000 × 1 ms = 1 s; SETTLE_TIME_S is 1.0 s, so it fires by then.
        for _ in 0..1100 {
            if td.observe(v, z, 0.001) {
                fired = true;
                break;
            }
        }
        assert!(fired);
    }

    #[test]
    fn touchdown_detector_resets_on_disturbance() {
        let mut td = TouchdownDetector::new();
        let quiet = Vector3::new(0.05, 0.05, 0.05);
        let airborne = Vector3::new(0.0, 0.0, 2.0); // fast descent
        let z = -0.1_f32;
        // Accumulate half the settle time.
        for _ in 0..500 {
            let _ = td.observe(quiet, z, 0.001);
        }
        // One sample of large velocity — should reset.
        let _ = td.observe(airborne, z, 0.001);
        // Another half second shouldn't fire yet.
        for _ in 0..400 {
            assert!(!td.observe(quiet, z, 0.001));
        }
    }

    #[test]
    fn touchdown_detector_does_not_fire_while_airborne() {
        let mut td = TouchdownDetector::new();
        let v = Vector3::new(0.01, 0.01, 0.01);
        let z_airborne = -5.0_f32;
        for _ in 0..5000 {
            assert!(!td.observe(v, z_airborne, 0.001));
        }
    }

    #[test]
    fn home_position_latches_on_first_arm() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
        assert!(flight.home_position_ned.is_none());
        // First call with Disarmed — no latch.
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        assert!(flight.home_position_ned.is_none());
        // Arm and move "vehicle" — latch should happen on next outer_step
        // and capture current estimate.
        flight.arm_state = ArmState::Armed;
        flight.state.position_ned = Vector3::new(1.0, 2.0, -3.0);
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        assert_eq!(flight.home_position_ned, Some(Vector3::new(1.0, 2.0, -3.0)));
        // Moving afterwards must not overwrite.
        flight.state.position_ned = Vector3::new(5.0, 5.0, -5.0);
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        assert_eq!(flight.home_position_ned, Some(Vector3::new(1.0, 2.0, -3.0)));
    }

    #[test]
    fn takeoff_mode_drives_z_toward_target_and_completes() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            takeoff_state: TakeoffState::TakingOff { target_z_ned: -5.0 },
            ..FlightState::default()
        };
        // Vehicle already hovering at target.
        flight.state.position_ned = Vector3::new(0.0, 0.0, -5.0);
        flight.state.velocity_ned = Vector3::zeros();
        let imu = stationary_imu(0);
        for _ in 0..1200 {
            let _ = outer_step(&mut cfg, &mut flight, imu, 0.001, &Setpoint::default());
        }
        assert_eq!(flight.takeoff_state, TakeoffState::Idle);
    }

    #[test]
    fn altitude_reached_detector_fires_and_resets() {
        let mut d = AltitudeReachedDetector::new();
        // Within tolerance + quiet velocity — accumulates.
        for _ in 0..1100 {
            let _ = d.observe(-5.0, 0.0, -5.0, 0.001);
        }
        assert!(d.observe(-5.0, 0.0, -5.0, 0.001));
        d.reset();
        // Too fast → never settles.
        for _ in 0..2000 {
            assert!(!d.observe(-5.0, 1.0, -5.0, 0.001));
        }
    }

    #[test]
    fn preflight_rejects_default_because_ekf_not_converged() {
        let flight = FlightState::default();
        // Initial covariance has σ²=100 per axis → trace=300 ≫ 5.
        let r = preflight_check(&flight);
        assert_eq!(r, Err(PreflightReject::EkfNotConverged));
    }

    #[test]
    fn preflight_passes_when_covariance_shrinks() {
        let mut flight = FlightState::default();
        // Manually shrink position covariance below threshold.
        for i in 0..algo_ekf::idx::P_NED_LEN {
            flight.covariance[(
                algo_ekf::idx::P_NED_START + i,
                algo_ekf::idx::P_NED_START + i,
            )] = 0.5;
        }
        assert!(preflight_check(&flight).is_ok());
    }

    #[test]
    fn preflight_rejects_when_gps_unhealthy() {
        let mut flight = FlightState::default();
        // Shrink covariance first so EKF check passes.
        for i in 0..algo_ekf::idx::P_NED_LEN {
            flight.covariance[(
                algo_ekf::idx::P_NED_START + i,
                algo_ekf::idx::P_NED_START + i,
            )] = 0.5;
        }
        // Feed enough rejections to tip gps_health to Degraded.
        for _ in 0..SensorRejectionCounter::DEFAULT_N_DEGRADE + 1 {
            flight.gps_health.observe(false);
        }
        assert_eq!(preflight_check(&flight), Err(PreflightReject::GpsUnhealthy));
    }

    #[test]
    fn preflight_reason_string_covers_every_variant() {
        // Any future variant added to PreflightReject forces an update
        // here; keeps reason_str() exhaustive.
        for r in [
            PreflightReject::GpsUnhealthy,
            PreflightReject::BaroUnhealthy,
            PreflightReject::MagUnhealthy,
            PreflightReject::EkfNotConverged,
        ] {
            assert!(!r.reason_str().is_empty());
        }
    }

    #[test]
    fn rtl_without_home_is_cleared_to_idle() {
        // Disarmed means the first-arm latch doesn't fire, so
        // home_position_ned stays None. RTL in this state has no target
        // and must clear itself.
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Disarmed,
            rtl_phase: RtlPhase::Climbing,
            home_position_ned: None,
            ..FlightState::default()
        };
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        assert_eq!(flight.rtl_phase, RtlPhase::Idle);
        assert!(flight.home_position_ned.is_none());
    }

    #[test]
    fn rtl_climb_to_safe_alt_then_transit_then_land() {
        let mut cfg = default_config_250g();
        cfg.rtl_safe_alt_m = 5.0;
        cfg.rtl_xy_tolerance_m = 0.5;
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            rtl_phase: RtlPhase::Climbing,
            home_position_ned: Some(Vector3::new(0.0, 0.0, 0.0)),
            ..FlightState::default()
        };
        // Start 20 m from home, at -1 m altitude (below safe alt -5).
        flight.state.position_ned = Vector3::new(15.0, -10.0, -1.0);

        // Step 1: Climbing. Simulate z reaching safe alt.
        flight.state.position_ned.z = -5.0;
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        assert_eq!(flight.rtl_phase, RtlPhase::Returning);

        // Step 2: Returning. Simulate xy reaching home within tolerance.
        flight.state.position_ned = Vector3::new(0.2, -0.1, -5.0);
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        assert_eq!(flight.rtl_phase, RtlPhase::Idle);
        assert_eq!(flight.landing_state, LandingState::Landing);
    }

    #[test]
    fn rtl_already_above_safe_alt_skips_climb() {
        let mut cfg = default_config_250g();
        cfg.rtl_safe_alt_m = 5.0;
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            rtl_phase: RtlPhase::Climbing,
            home_position_ned: Some(Vector3::new(0.0, 0.0, 0.0)),
            ..FlightState::default()
        };
        // Already at -10 m (10 m above home) — above safe alt of 5 m.
        flight.state.position_ned = Vector3::new(5.0, 5.0, -10.0);
        let _ = outer_step(
            &mut cfg,
            &mut flight,
            stationary_imu(0),
            0.001,
            &Setpoint::default(),
        );
        // One tick is enough to skip to Returning.
        assert_eq!(flight.rtl_phase, RtlPhase::Returning);
    }

    #[test]
    fn landing_mode_overrides_setpoint_and_auto_disarms() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            landing_state: LandingState::Landing,
            ..FlightState::default()
        };
        // Vehicle already on the ground, stationary.
        flight.state.position_ned = Vector3::new(0.0, 0.0, -0.05);
        flight.state.velocity_ned = Vector3::zeros();
        let imu = stationary_imu(0);
        // Caller passes a setpoint of (0, 0, -5) — but landing override
        // should ignore it and ramp down in place.
        let sp = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -5.0),
            ..Setpoint::default()
        };
        // Run > SETTLE_TIME_S worth of ticks.
        for _ in 0..1200 {
            let _ = outer_step(&mut cfg, &mut flight, imu, 0.001, &sp);
        }
        assert_eq!(flight.arm_state, ArmState::Disarmed);
        assert_eq!(flight.landing_state, LandingState::Idle);
    }

    #[test]
    fn rate_loop_commands_positive_roll_shifts_motors_asymmetrically() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        // Seed the filters with 10 stationary samples.
        for i in 0..10 {
            let _ = rate_loop_step(
                &mut cfg,
                &mut flight,
                stationary_imu(i),
                0.001,
                RateCommand::default(),
            );
        }
        let cmd = RateCommand {
            body_rate_rad_s: Vector3::new(2.0, 0.0, 0.0),
        };
        let out = rate_loop_step(&mut cfg, &mut flight, stationary_imu(11), 0.001, cmd);
        // Positive roll cmd → y<0 motors (M2, M3) get more thrust than y>0 motors (M1, M4).
        let m1 = out.motor_thrusts_n.fixed_view::<1, 1>(0, 0).to_scalar();
        let m2 = out.motor_thrusts_n.fixed_view::<1, 1>(1, 0).to_scalar();
        let m3 = out.motor_thrusts_n.fixed_view::<1, 1>(2, 0).to_scalar();
        let m4 = out.motor_thrusts_n.fixed_view::<1, 1>(3, 0).to_scalar();
        assert!(m2 > m1);
        assert!(m3 > m4);
    }
}
