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
    baro_update, gps_update, mag_update, predict_step_with_drag, BaroMeasurement, Covariance,
    GpsMeasurement, ImuMeasurement, MagMeasurement, ProcessNoise, State,
};
use algo_fdir::{HealthLevel, SensorRejectionCounter};
use algo_indi::{
    attitude_to_rate, compute_torque_increment, AttitudeGain, IndiInput, Inertia,
    LowPassFilterVec3, RateCommand, RateGain,
};
use algo_nmpc::{position_to_attitude_thrust_pi, PositionGains, Setpoint};
use core_hal::traits::ImuSample;
use nalgebra::{Matrix4, SVector, Vector3};

pub use algo_alloc::{
    allocate, build_effectiveness, invert_quad_effectiveness, saturate, standard_x_quad,
    MotorGeometry, VirtualCommand,
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
        }
    }
}

impl FlightState {
    /// Roll-up of every sensor stream's health level (worst wins).
    #[must_use]
    pub fn overall_health(&self) -> HealthLevel {
        [self.gps_health.level(), self.mag_health.level(), self.baro_health.level()]
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

    RateLoopOutput {
        motor_thrusts_n: clipped,
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
        // EKF-side drag is opt-in: without the matching ∂v/∂wind_ne
        // Jacobian block (queued for M4), a non-zero value here shifts
        // the PI integrator tuning. Leave off until wind observability
        // is real; app callers can set it explicitly for tests.
        drag_over_mass_hz: 0.0,
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
    // Wind feed-forward: add k·wind_estimate to the setpoint's accel
    // feedforward channel. Zero-latency disturbance compensation that
    // complements the velocity-loop integrator.
    let wind_ff = Vector3::new(
        flight.state.wind_ne.x * cfg.wind_ff_gain,
        flight.state.wind_ne.y * cfg.wind_ff_gain,
        0.0,
    );
    let adjusted_setpoint = Setpoint {
        accel_ned: setpoint.accel_ned + wind_ff,
        ..*setpoint
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
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
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
        let mut flight = FlightState::default();
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
        let m = algo_ekf::BaroMeasurement { altitude_m: 2.0, sigma_m: 0.1 };
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
        assert!(vx_on > vx_off + 0.01,
            "drag FF inactive: vx_on={vx_on} vx_off={vx_off}");
    }

    #[test]
    fn outer_step_wind_feedforward_tilts_against_wind() {
        // With a known horizontal wind, the controller should tilt
        // *into* the wind to cancel drag — even at zero pos/vel error.
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
        // Pretend the EKF has identified +3 m/s east wind.
        flight.state.wind_ne = nalgebra::Vector2::new(0.0, 3.0);
        let imu = stationary_imu(0);
        let setpoint = Setpoint::default();
        let out = outer_step(&mut cfg, &mut flight, imu, 0.001, &setpoint);

        // Same scenario but with wind FF disabled.
        let mut cfg_no_ff = default_config_250g();
        cfg_no_ff.wind_ff_gain = 0.0;
        let mut flight_no_ff = FlightState::default();
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
        assert!(diff > 1.0e-3, "wind FF had no effect on motors: diff={diff}");
    }

    #[test]
    fn outer_step_altitude_setpoint_raises_thrust() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
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
    fn rate_loop_commands_positive_roll_shifts_motors_asymmetrically() {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
        // Seed the filters with 10 stationary samples.
        for i in 0..10 {
            let _ =
                rate_loop_step(&mut cfg, &mut flight, stationary_imu(i), 0.001, RateCommand::default());
        }
        let cmd = RateCommand { body_rate_rad_s: Vector3::new(2.0, 0.0, 0.0) };
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
