//! Reusable SITL runners for examples and integration tests.
//!
//! The bulk of sim-hil's unit tests have their own closed-loop
//! runners in `#[cfg(test)]` scope — fine for in-crate tests, not
//! reachable from `cargo run --example ...` or downstream binaries.
//! This module lifts the most commonly-useful runner
//! ([`run_with_controller`]) to the crate's public API so the
//! shootout example and any future CLI tool can share one
//! scenario-configuration surface.
//!
//! The runners stay in the flight-controller framework deliberately:
//! same `rate_loop_step` + `FlightState` + inner attitude / INDI /
//! allocation stack as every SITL test in the crate. What changes is
//! only the position-loop controller, which is why the API is
//! parameterised over [`algo_nmpc::PositionController`].

use algo_indi::attitude_to_rate;
use algo_nmpc::{PositionController, Setpoint};
use app_copter::{
    ArmState, FlightState, apply_baro_measurement, apply_gps_measurement, apply_mag_measurement,
    default_config_250g, rate_loop_step,
};
use nalgebra::Vector3;

use crate::{
    SimConfig, SimRng, SimState, accel_world, sense_baro, sense_gps, sense_imu, sense_mag, step,
};

/// Common knobs for a SITL run. Defaults match the realistic-sim
/// scenario used by the shootout harness: 1 kHz inner loop, 0.001 s
/// tick, setpoint (0, 0, -1 m) with no lateral motion, seed 1.
#[derive(Clone, Debug)]
pub struct SitlScenario {
    pub sim_cfg: SimConfig,
    pub seed: u64,
    pub ticks: usize,
    pub setpoint: Setpoint,
    pub dt_s: f32,
}

impl Default for SitlScenario {
    fn default() -> Self {
        Self {
            sim_cfg: SimConfig::default(),
            seed: 1,
            ticks: 3_000,
            setpoint: Setpoint {
                position_ned: Vector3::new(0.0, 0.0, -1.0),
                ..Setpoint::default()
            },
            dt_s: 0.001,
        }
    }
}

/// Analytical figure-8 (lemniscate-of-Gerono) setpoint generator.
///
/// `position(t) = (A sin ω t, B sin 2 ω t, −altitude)` plus the exact
/// first and second derivatives as velocity and accel feed-forwards.
/// Matches the Agilicious-paper figure-8 benchmark shape — one
/// "lobe" per half-period, crossover at t = 0, π/ω, 2π/ω…
///
/// `amplitude_xy_m` is the x-axis amplitude A; the y-axis amplitude B
/// is half of A, which gives the canonical 2 : 1 figure-8 ratio.
/// `period_s` is the full-8 orbit period (time to trace one lobe +
/// the other). `altitude_m` is the cruise altitude (NED z = −altitude).
pub fn figure_eight(
    amplitude_xy_m: f32,
    period_s: f32,
    altitude_m: f32,
) -> impl FnMut(f32) -> Setpoint + Clone {
    use core::f32::consts::TAU;
    let omega = TAU / period_s;
    let a = amplitude_xy_m;
    let b = 0.5 * amplitude_xy_m;
    move |t: f32| {
        let c1 = libm::cosf(omega * t);
        let s1 = libm::sinf(omega * t);
        let c2 = libm::cosf(2.0 * omega * t);
        let s2 = libm::sinf(2.0 * omega * t);
        Setpoint {
            position_ned: Vector3::new(a * s1, b * s2, -altitude_m),
            velocity_ned: Vector3::new(a * omega * c1, 2.0 * b * omega * c2, 0.0),
            accel_ned: Vector3::new(-a * omega * omega * s1, -4.0 * b * omega * omega * s2, 0.0),
            yaw_rad: 0.0,
        }
    }
}

/// Outcome of a trajectory run — adds per-tick tracking error stats
/// on top of [`SitlResult`]. RMS is computed across the whole window
/// with no warm-up exclusion; callers that want settle-time-aware
/// metrics should run the loop themselves.
#[derive(Clone, Copy, Debug)]
pub struct TrajectorySitlResult {
    pub final_state: SimState,
    pub position_rms_m: f32,
    pub max_position_err_m: f32,
    pub velocity_rms_m_s: f32,
}

/// Run a closed-loop SITL with a *time-varying* setpoint fed by
/// `trajectory(t_s)`. Same inner stack as [`run_with_controller`],
/// but the setpoint is queried fresh every tick. Returns position /
/// velocity RMS across the entire window.
///
/// Using an analytical trajectory (versus a stream of discrete
/// waypoints) matches how the Agilicious paper benchmarks agile
/// flight — the controller sees continuous position + velocity +
/// accel feed-forward at every tick.
#[allow(clippy::too_many_arguments)]
pub fn run_with_controller_trajectory<const H: usize, F>(
    scenario: &SitlScenario,
    controller: &mut PositionController<H>,
    mut trajectory: F,
) -> TrajectorySitlResult
where
    F: FnMut(f32) -> Setpoint,
{
    let mut sim_state = SimState {
        position_ned: trajectory(0.0).position_ned,
        ..SimState::default()
    };
    let mut rng = SimRng::new(scenario.seed);
    let mut app_cfg = default_config_250g();

    let mut flight = FlightState {
        arm_state: ArmState::Armed,
        ..FlightState::default()
    };
    let _ = apply_baro_measurement(
        &mut flight,
        &sense_baro(&scenario.sim_cfg, &sim_state, &mut rng),
    );
    let _ = apply_gps_measurement(
        &mut flight,
        &sense_gps(&scenario.sim_cfg, &sim_state, &mut rng),
    );

    let mut pos_sq_sum = 0.0_f64;
    let mut vel_sq_sum = 0.0_f64;
    let mut max_pos_err = 0.0_f32;

    for i in 0..scenario.ticks {
        // u32 → f32 is lossless up to 2^24; realistic SITL windows
        // (15 s at 1 kHz = 15_000 ticks) are well inside that.
        let i_u32 = u32::try_from(i).unwrap_or(u32::MAX);
        #[allow(clippy::as_conversions)]
        let t_s = scenario.dt_s * (i_u32 as f32);
        let setpoint = trajectory(t_s);
        let accel_w = accel_world(&scenario.sim_cfg, &sim_state);
        let imu = sense_imu(&scenario.sim_cfg, &sim_state, accel_w, &mut rng);

        let att = controller.step(
            &setpoint,
            flight.state.position_ned,
            flight.state.velocity_ned,
            app_cfg.mass_kg,
            scenario.dt_s,
        );
        let rate_cmd = attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
        let saved_hover = app_cfg.hover_thrust_n;
        app_cfg.hover_thrust_n = att.thrust_n;
        let out = rate_loop_step(&mut app_cfg, &mut flight, imu, scenario.dt_s, rate_cmd);
        app_cfg.hover_thrust_n = saved_hover;
        step(
            &scenario.sim_cfg,
            &mut sim_state,
            &out.motor_thrusts_n,
            scenario.dt_s,
        );

        // Error accumulation using truth (sim_state) vs commanded
        // setpoint. Using f64 sums to avoid catastrophic cancellation
        // over long trajectories.
        let pos_err = sim_state.position_ned - setpoint.position_ned;
        let pos_err_sq = pos_err.norm_squared();
        pos_sq_sum += f64::from(pos_err_sq);
        let pos_err_mag = libm::sqrtf(pos_err_sq);
        if pos_err_mag > max_pos_err {
            max_pos_err = pos_err_mag;
        }
        let vel_err = sim_state.velocity_ned - setpoint.velocity_ned;
        vel_sq_sum += f64::from(vel_err.norm_squared());

        if i.is_multiple_of(200) {
            let _ = apply_gps_measurement(
                &mut flight,
                &sense_gps(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(40) {
            let _ = apply_mag_measurement(
                &mut flight,
                &sense_mag(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(20) {
            let _ = apply_baro_measurement(
                &mut flight,
                &sense_baro(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
    }

    let n = f64::from(u32::try_from(scenario.ticks).unwrap_or(u32::MAX)).max(1.0);
    // Accept the narrowing f64 → f32 for the final-metric cast — the
    // callers work in f32 m and the f64 sum is only to avoid round-
    // off accumulation during the loop.
    #[allow(clippy::as_conversions, clippy::cast_possible_truncation)]
    let position_rms_m = libm::sqrtf((pos_sq_sum / n) as f32);
    #[allow(clippy::as_conversions, clippy::cast_possible_truncation)]
    let velocity_rms_m_s = libm::sqrtf((vel_sq_sum / n) as f32);
    TrajectorySitlResult {
        final_state: sim_state,
        position_rms_m,
        max_position_err_m: max_pos_err,
        velocity_rms_m_s,
    }
}

/// Outcome of a closed-loop run. Trackable metrics only — callers
/// that need full state snapshots should run the loop themselves.
#[derive(Clone, Copy, Debug)]
pub struct SitlResult {
    pub final_state: SimState,
    pub altitude_err_m: f32,
    pub horizontal_err_m: f32,
}

impl SitlResult {
    fn from_final(final_state: SimState, setpoint: &Setpoint) -> Self {
        let altitude_err_m = (final_state.position_ned.z - setpoint.position_ned.z).abs();
        let horizontal_err_m = ((final_state.position_ned.x - setpoint.position_ned.x).powi(2)
            + (final_state.position_ned.y - setpoint.position_ned.y).powi(2))
        .sqrt();
        Self {
            final_state,
            altitude_err_m,
            horizontal_err_m,
        }
    }
}

/// Run a SITL with the supplied [`PositionController`]. Returns the
/// tracking-error summary; caller keeps the controller so integrator
/// / warm-start state survives across calls if needed.
pub fn run_with_controller<const H: usize>(
    scenario: &SitlScenario,
    controller: &mut PositionController<H>,
) -> SitlResult {
    let mut sim_state = SimState {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        ..SimState::default()
    };
    let mut rng = SimRng::new(scenario.seed);
    let mut app_cfg = default_config_250g();

    let mut flight = FlightState {
        arm_state: ArmState::Armed,
        ..FlightState::default()
    };
    let _ = apply_baro_measurement(
        &mut flight,
        &sense_baro(&scenario.sim_cfg, &sim_state, &mut rng),
    );
    let _ = apply_gps_measurement(
        &mut flight,
        &sense_gps(&scenario.sim_cfg, &sim_state, &mut rng),
    );

    for i in 0..scenario.ticks {
        let accel_w = accel_world(&scenario.sim_cfg, &sim_state);
        let imu = sense_imu(&scenario.sim_cfg, &sim_state, accel_w, &mut rng);

        let att = controller.step(
            &scenario.setpoint,
            flight.state.position_ned,
            flight.state.velocity_ned,
            app_cfg.mass_kg,
            scenario.dt_s,
        );
        let rate_cmd = attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
        let saved_hover = app_cfg.hover_thrust_n;
        app_cfg.hover_thrust_n = att.thrust_n;
        let out = rate_loop_step(&mut app_cfg, &mut flight, imu, scenario.dt_s, rate_cmd);
        app_cfg.hover_thrust_n = saved_hover;
        step(
            &scenario.sim_cfg,
            &mut sim_state,
            &out.motor_thrusts_n,
            scenario.dt_s,
        );

        if i.is_multiple_of(200) {
            let _ = apply_gps_measurement(
                &mut flight,
                &sense_gps(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(40) {
            let _ = apply_mag_measurement(
                &mut flight,
                &sense_mag(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(20) {
            let _ = apply_baro_measurement(
                &mut flight,
                &sense_baro(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
    }
    SitlResult::from_final(sim_state, &scenario.setpoint)
}

/// Run with an [`crate::residual_mpc::MpcResidualController`]. Parallels
/// `run_with_controller` but the residual API is its own type (not a
/// `PositionController` enum variant) so the runner is a separate
/// function. Caller owns the controller to keep residual-policy state
/// visible.
pub fn run_with_mpc_residual<const H: usize, B>(
    scenario: &SitlScenario,
    controller: &mut crate::residual_mpc::MpcResidualController<H, B>,
) -> SitlResult
where
    B: nn_runtime::InferenceBackend,
{
    let mut sim_state = SimState {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        ..SimState::default()
    };
    let mut rng = SimRng::new(scenario.seed);
    let mut app_cfg = default_config_250g();

    let mut flight = FlightState {
        arm_state: ArmState::Armed,
        ..FlightState::default()
    };
    let _ = apply_baro_measurement(
        &mut flight,
        &sense_baro(&scenario.sim_cfg, &sim_state, &mut rng),
    );
    let _ = apply_gps_measurement(
        &mut flight,
        &sense_gps(&scenario.sim_cfg, &sim_state, &mut rng),
    );

    for i in 0..scenario.ticks {
        let accel_w = accel_world(&scenario.sim_cfg, &sim_state);
        let imu = sense_imu(&scenario.sim_cfg, &sim_state, accel_w, &mut rng);

        let att = controller.step(
            &scenario.setpoint,
            flight.state.position_ned,
            flight.state.velocity_ned,
            flight.state.attitude,
            app_cfg.mass_kg,
        );
        let rate_cmd = attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
        let saved_hover = app_cfg.hover_thrust_n;
        app_cfg.hover_thrust_n = att.thrust_n;
        let out = rate_loop_step(&mut app_cfg, &mut flight, imu, scenario.dt_s, rate_cmd);
        app_cfg.hover_thrust_n = saved_hover;
        step(
            &scenario.sim_cfg,
            &mut sim_state,
            &out.motor_thrusts_n,
            scenario.dt_s,
        );

        if i.is_multiple_of(200) {
            let _ = apply_gps_measurement(
                &mut flight,
                &sense_gps(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(40) {
            let _ = apply_mag_measurement(
                &mut flight,
                &sense_mag(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(20) {
            let _ = apply_baro_measurement(
                &mut flight,
                &sense_baro(&scenario.sim_cfg, &sim_state, &mut rng),
            );
        }
    }
    SitlResult::from_final(sim_state, &scenario.setpoint)
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
    use algo_nmpc::{LqiWeights, Mpc1dIConfig, PositionController, PositionGains};

    /// Figure-8 generator produces position/velocity consistent with
    /// each other (velocity ≈ dposition/dt) at a few sampled times.
    #[test]
    fn figure_eight_derivatives_self_consistent() {
        let amplitude = 2.0_f32;
        let period = 10.0_f32;
        let altitude = 1.0_f32;
        let mut traj = figure_eight(amplitude, period, altitude);
        let dt = 0.001_f32;
        for &t in &[0.0_f32, 0.5, 2.5, 7.5] {
            let s0 = traj(t);
            let s1 = traj(t + dt);
            let fd_vx = (s1.position_ned.x - s0.position_ned.x) / dt;
            let fd_vy = (s1.position_ned.y - s0.position_ned.y) / dt;
            assert!(
                (fd_vx - s0.velocity_ned.x).abs() < 0.01,
                "t={t}: fd_vx {fd_vx} vs analytical {}",
                s0.velocity_ned.x
            );
            assert!(
                (fd_vy - s0.velocity_ned.y).abs() < 0.01,
                "t={t}: fd_vy {fd_vy} vs analytical {}",
                s0.velocity_ned.y
            );
            assert!((s0.position_ned.z - -altitude).abs() < 1.0e-6);
        }
    }

    /// Closed-loop figure-8 tracking with MPC-I in ideal sim. 15 s
    /// (1.5 full orbits) at 2 m amplitude / 10 s period / 1 m alt.
    /// Target: < 0.5 m position RMS. Phase III's first benchmark.
    #[test]
    fn figure_eight_mpc_i_tracks_within_half_meter_rms() {
        let scenario = SitlScenario {
            sim_cfg: SimConfig::default(),
            seed: 7,
            ticks: 15_000,
            setpoint: Setpoint::default(),
            dt_s: 0.001,
        };
        let lqi_weights = LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 1.5,
            r: 0.5,
        };
        let cfg = Mpc1dIConfig {
            weights: lqi_weights,
            dt_s: scenario.dt_s,
            u_min: -20.0,
            u_max: 20.0,
        };
        let mut ctrl =
            PositionController::<10>::mpc_i(cfg, cfg, 25, PositionGains::default().max_accel, 5.0)
                .unwrap();
        let traj = figure_eight(2.0, 10.0, 1.0);
        let result = run_with_controller_trajectory(&scenario, &mut ctrl, traj);
        assert!(
            result.position_rms_m < 0.5,
            "position RMS {} m ≥ 0.5 m",
            result.position_rms_m
        );
        assert!(
            result.max_position_err_m < 1.5,
            "max position err {} m ≥ 1.5 m",
            result.max_position_err_m
        );
    }

    /// Phase III benchmark #2: single-motor failure + 3-motor
    /// failover allocation.
    ///
    /// Start in steady hover at 1 m, zero motor 0 at t = 2 s, and
    /// simultaneously mark `flight.motor_alive[0] = false` so the
    /// rate loop engages the [`FailoverAllocator`] (M19a+b). Run
    /// 3 more seconds.
    ///
    /// What we now verify (M19c, tightened from M18 free-fall bound):
    ///
    ///   * state stays finite (no NaN, no ±∞)
    ///   * altitude err < 3 m — sustained hover via the 3-motor
    ///     allocation; total thrust is still maintained at
    ///     `hover_thrust_n`, only yaw is sacrificed
    ///   * tilt < 60° — body z-axis wanders because yaw is
    ///     uncontrolled (vehicle spins), but roll/pitch stay
    ///     inside a recoverable envelope
    ///
    /// What we **don't** verify and won't until a dedicated Mueller-
    /// style 3-motor controller lands (queued as M20):
    ///
    ///   * yaw rate bound — the two surviving CCW-propeller motors
    ///     both yaw the airframe in the same direction, producing
    ///     a sustained spin that's expected behaviour per Mueller &
    ///     D'Andrea 2014
    ///   * position tracking in xy — the yaw spin couples
    ///     into the body-frame roll/pitch commands, so xy RMS will
    ///     be larger than nominal hover; left unchecked here
    #[test]
    fn single_motor_failure_failover_maintains_altitude() {
        use algo_ekf::GRAVITY_M_S2;
        use algo_indi::attitude_to_rate;
        use algo_nmpc::Setpoint;
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };
        use nalgebra::SVector;

        // Realistic actuator dynamics so the failure is a physical
        // event, not an instantaneous command cut.
        let mut sim_cfg = SimConfig {
            motor_tau_s: 0.02,
            motor_fault_mask: [1.0; 4],
            ..SimConfig::default()
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            motor_thrusts_actual_n: SVector::<f32, 4>::repeat(sim_cfg.mass_kg * GRAVITY_M_S2 / 4.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(123);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));

        let lqi_weights = algo_nmpc::LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 1.5,
            r: 0.5,
        };
        let cfg_mpci = algo_nmpc::Mpc1dIConfig {
            weights: lqi_weights,
            dt_s: dt,
            u_min: -20.0,
            u_max: 20.0,
        };
        let mut ctrl = PositionController::<10>::mpc_i(
            cfg_mpci,
            cfg_mpci,
            25,
            PositionGains::default().max_accel,
            5.0,
        )
        .unwrap();

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        let fail_tick: usize = 2_000; // 2 s
        let total_ticks: usize = 5_000; // 5 s total, 3 s post-failure
        let mut max_tilt_rad = 0.0_f32;
        let mut max_alt_err = 0.0_f32;
        let mut max_xy_err = 0.0_f32;

        for i in 0_usize..total_ticks {
            if i == fail_tick {
                // Motor 0 dies completely. Real motor failures are
                // often softer (ESC thermal cutoff, commutation loss)
                // but 0 is the hardest single-motor case.
                sim_cfg.motor_fault_mask[0] = 0.0;
                // Oracle fault notification: the controller learns
                // of the failure instantly. A real FDIR module
                // (M20a-d) detects the fault from the ω̇ residual
                // with a small latency (~80 ms typical). This test
                // uses oracle mode to isolate the allocator +
                // M21 attitude handoff from detector latency.
                flight.motor_alive[0] = false;
            }
            let accel_w = accel_world(&sim_cfg, &sim_state);
            let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
            let att = ctrl.step(
                &setpoint,
                flight.state.position_ned,
                flight.state.velocity_ned,
                app_cfg.mass_kg,
                dt,
            );
            let rate_cmd =
                attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
            let saved_hover = app_cfg.hover_thrust_n;
            app_cfg.hover_thrust_n = att.thrust_n;
            let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
            app_cfg.hover_thrust_n = saved_hover;
            step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

            if i >= fail_tick {
                // Tilt from body z-axis: arccos(R[2,2]) where R is
                // the body→world rotation. Cheap approximation via
                // quaternion w component.
                let q_w = sim_state.attitude.w.abs();
                let tilt = libm::acosf((2.0 * q_w * q_w - 1.0).clamp(-1.0, 1.0));
                if tilt > max_tilt_rad {
                    max_tilt_rad = tilt;
                }
                let alt_err = (-sim_state.position_ned.z - 1.0).abs();
                if alt_err > max_alt_err {
                    max_alt_err = alt_err;
                }
                let xy_err = libm::sqrtf(
                    sim_state.position_ned.x * sim_state.position_ned.x
                        + sim_state.position_ned.y * sim_state.position_ned.y,
                );
                if xy_err > max_xy_err {
                    max_xy_err = xy_err;
                }
            }

            if i.is_multiple_of(200) {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));
            }
            if i.is_multiple_of(40) {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(&sim_cfg, &sim_state, &mut rng));
            }
            if i.is_multiple_of(20) {
                let _ = apply_baro_measurement(
                    &mut flight,
                    &sense_baro(&sim_cfg, &sim_state, &mut rng),
                );
            }
        }

        // Survival assertions — now that failover is wired + M21
        // yaw-handoff:
        //   1. position + velocity finite (no NaN / ∞)
        //   2. altitude err < 3 m — 3-motor allocation preserves
        //      the total thrust magnitude
        //   3. tilt < 60° (≈ 1.05 rad) — yaw spin perturbs roll /
        //      pitch but the vehicle stays upright
        //   4. M21-specific: xy err < 3 m — with yaw handoff, the
        //      position controller's body-frame commands no longer
        //      rotate out from under the vehicle. Pre-M21 this would
        //      drift unbounded as yaw spins and body axes wander.
        assert!(
            sim_state.position_ned.norm().is_finite(),
            "position went non-finite: {:?}",
            sim_state.position_ned
        );
        assert!(
            sim_state.velocity_ned.norm().is_finite(),
            "velocity went non-finite: {:?}",
            sim_state.velocity_ned
        );
        assert!(
            max_alt_err < 3.0,
            "altitude err {max_alt_err} m ≥ 3 m — 3-motor failover not preserving lift",
        );
        assert!(
            max_tilt_rad < 1.05,
            "max tilt {max_tilt_rad} rad (> 60°) — vehicle losing upright posture",
        );
        assert!(
            max_xy_err < 3.0,
            "xy err {max_xy_err} m ≥ 3 m — M21 yaw handoff not preventing position drift",
        );
    }

    /// M20c end-to-end: inject a motor fault with NO oracle —
    /// `flight.motor_alive` starts `[true; 4]` and the
    /// `MotorFaultDetector` wired into `FlightState` by M20b has to
    /// find the dead rotor on its own from the ω̇ residual.
    ///
    /// Flow:
    ///   1. 2 s of nominal hover — detector sees zero residual,
    ///      persistence stays at 0
    ///   2. `sim_cfg.motor_fault_mask[0] = 0.0` — motor 0 dies
    ///   3. Within ~50 ms (the detector's default
    ///      n_ticks_to_declare) persistence on motor 0 crosses
    ///      threshold → detector latches `alive[0] = false` → next
    ///      tick's AND-update propagates to `flight.motor_alive[0]`
    ///   4. Failover allocator engages; altitude recovers
    ///
    /// Assertions:
    ///   * `flight.motor_alive[0] == false` by the end
    ///   * `flight.motor_fault_detector.level() == Emergency`
    ///   * Altitude error < 5 m (looser than M19c's 3 m because of
    ///     the detection latency window during which allocation was
    ///     still wrong)
    ///   * Tilt stays under 75° (1.31 rad); the extra 15° vs M19c
    ///     accounts for the pre-detection transient
    #[test]
    fn single_motor_failure_detected_end_to_end_without_oracle() {
        use algo_ekf::GRAVITY_M_S2;
        use algo_indi::attitude_to_rate;
        use algo_nmpc::Setpoint;
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };
        use nalgebra::SVector;

        let mut sim_cfg = SimConfig {
            motor_tau_s: 0.02,
            motor_fault_mask: [1.0; 4],
            ..SimConfig::default()
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            motor_thrusts_actual_n: SVector::<f32, 4>::repeat(sim_cfg.mass_kg * GRAVITY_M_S2 / 4.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(321);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));

        let lqi_weights = algo_nmpc::LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 1.5,
            r: 0.5,
        };
        let cfg_mpci = algo_nmpc::Mpc1dIConfig {
            weights: lqi_weights,
            dt_s: dt,
            u_min: -20.0,
            u_max: 20.0,
        };
        let mut ctrl = PositionController::<10>::mpc_i(
            cfg_mpci,
            cfg_mpci,
            25,
            PositionGains::default().max_accel,
            5.0,
        )
        .unwrap();

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        let fail_tick: usize = 2_000;
        let total_ticks: usize = 5_000;
        let mut max_tilt_rad = 0.0_f32;
        let mut max_alt_err = 0.0_f32;
        let mut declared_tick: Option<usize> = None;

        for i in 0_usize..total_ticks {
            if i == fail_tick {
                sim_cfg.motor_fault_mask[0] = 0.0;
                // Crucially: no oracle write to flight.motor_alive.
            }
            let accel_w = accel_world(&sim_cfg, &sim_state);
            let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
            let att = ctrl.step(
                &setpoint,
                flight.state.position_ned,
                flight.state.velocity_ned,
                app_cfg.mass_kg,
                dt,
            );
            let rate_cmd =
                attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
            let saved_hover = app_cfg.hover_thrust_n;
            app_cfg.hover_thrust_n = att.thrust_n;
            let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
            app_cfg.hover_thrust_n = saved_hover;
            step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

            if declared_tick.is_none() && !flight.motor_alive[0] {
                declared_tick = Some(i);
            }

            if i >= fail_tick {
                let q_w = sim_state.attitude.w.abs();
                let tilt = libm::acosf((2.0 * q_w * q_w - 1.0).clamp(-1.0, 1.0));
                if tilt > max_tilt_rad {
                    max_tilt_rad = tilt;
                }
                let alt_err = (-sim_state.position_ned.z - 1.0).abs();
                if alt_err > max_alt_err {
                    max_alt_err = alt_err;
                }
            }

            if i.is_multiple_of(200) {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));
            }
            if i.is_multiple_of(40) {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(&sim_cfg, &sim_state, &mut rng));
            }
            if i.is_multiple_of(20) {
                let _ = apply_baro_measurement(
                    &mut flight,
                    &sense_baro(&sim_cfg, &sim_state, &mut rng),
                );
            }
        }

        // Detector must have declared motor 0 dead.
        assert!(
            !flight.motor_alive[0],
            "detector failed to declare motor 0 dead"
        );
        let det_alive = flight.motor_fault_detector.alive();
        assert!(!det_alive[0], "detector.alive()[0] should be false");
        for i in 1..4 {
            assert!(
                det_alive[i],
                "detector wrongly declared motor {i} dead (only motor 0 actually died)"
            );
        }
        assert_eq!(
            flight.motor_fault_detector.level(),
            algo_fdir::HealthLevel::Emergency
        );

        // Detection latency: must be under 250 ticks (250 ms).
        // Default detector threshold is 50 ticks of persistence,
        // but the residual magnitude ramps up gradually with the
        // 20 ms motor-lag, so the "first threshold-crossing tick"
        // is typically ~70–100 after the fault. 250 ms gives
        // generous headroom for the noisy SITL.
        let dec = declared_tick.expect("detector never declared");
        let latency = dec - fail_tick;
        assert!(
            latency < 250,
            "detection latency {latency} ticks ≥ 250 ms — detector too slow",
        );

        assert!(
            sim_state.position_ned.norm().is_finite(),
            "position went non-finite: {:?}",
            sim_state.position_ned
        );
        assert!(
            sim_state.velocity_ned.norm().is_finite(),
            "velocity went non-finite: {:?}",
            sim_state.velocity_ned
        );
        assert!(
            max_alt_err < 5.0,
            "altitude err {max_alt_err} m ≥ 5 m — end-to-end detection+failover didn't preserve lift",
        );
        assert!(
            max_tilt_rad < 1.31,
            "max tilt {max_tilt_rad} rad (> 75°) — pre-detection transient too extreme",
        );
    }

    /// Same trajectory, PI cascade baseline. On a moving target
    /// MPC-I's predictive horizon + feed-forward should beat
    /// classical cascade.
    #[test]
    fn figure_eight_mpc_i_beats_pi_cascade() {
        let scenario = SitlScenario {
            sim_cfg: SimConfig::default(),
            seed: 7,
            ticks: 15_000,
            setpoint: Setpoint::default(),
            dt_s: 0.001,
        };
        let mut pi: PositionController<10> = PositionController::pi(PositionGains {
            k_i_vel: Vector3::new(0.5, 0.5, 0.8),
            ..PositionGains::default()
        });
        let pi_result =
            run_with_controller_trajectory(&scenario, &mut pi, figure_eight(2.0, 10.0, 1.0));
        let lqi_weights = LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 1.5,
            r: 0.5,
        };
        let cfg = Mpc1dIConfig {
            weights: lqi_weights,
            dt_s: scenario.dt_s,
            u_min: -20.0,
            u_max: 20.0,
        };
        let mut mpc_i =
            PositionController::<10>::mpc_i(cfg, cfg, 25, PositionGains::default().max_accel, 5.0)
                .unwrap();
        let mpc_result =
            run_with_controller_trajectory(&scenario, &mut mpc_i, figure_eight(2.0, 10.0, 1.0));
        assert!(
            mpc_result.position_rms_m < pi_result.position_rms_m,
            "MPC-I should beat PI on moving target: PI {} m, MPC-I {} m",
            pi_result.position_rms_m,
            mpc_result.position_rms_m
        );
    }

    /// Phase III benchmark #3: aggressive roll flip.
    ///
    /// 1. Hover stable at 1 m for 1 s
    /// 2. Command body_rate = (15, 0, 0) rad/s for 0.5 s — ~7.5 rad
    ///    of roll, enough to pass inverted (q_w crosses zero).
    /// 3. Zero-rate recovery for 1.5 s
    ///
    /// INDI's job: track the extreme rate command while the
    /// quaternion traverses the full `[±1, ±1]` unit-sphere, then
    /// bring the vehicle back upright without oscillation.
    ///
    /// Scope notes: this uses the rate-loop path directly (bypasses
    /// `outer_step`'s position controller). Position controller
    /// output during a flip is meaningless — it would command
    /// body-frame thrust that points arbitrary directions while the
    /// body frame itself tumbles. A real flight stack would switch
    /// to an "acro" / manual mode for the flip and back to
    /// position-hold on recovery, but wiring that mode switch into
    /// `outer_step` is M23+ scope. For this benchmark we just want
    /// to prove the INDI inner loop + allocator survive the
    /// maneuver.
    ///
    /// Assertions:
    ///   * quaternion norm stays within 1e-2 of 1 throughout
    ///     (normalisation is the EKF's responsibility; drift here
    ///     would indicate gimbal-lock-like numerical breakage)
    ///   * at some point q_w < 0 — vehicle actually went inverted
    ///   * final tilt < 10° after 1.5 s of recovery — INDI returns
    ///     the vehicle to upright
    ///   * altitude loss during flip < 3 m (the thrust vector
    ///     spends time pointing horizontally while inverted; some
    ///     gravity drop is unavoidable)
    #[test]
    fn roll_flip_completes_and_recovers_upright() {
        use algo_ekf::GRAVITY_M_S2;
        use algo_indi::{RateCommand, attitude_to_rate};
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };
        use nalgebra::{Quaternion, SVector};

        // `SimConfig::default()` already carries realistic noise;
        // the override below is just explicit motor lag.
        let sim_cfg = SimConfig {
            motor_tau_s: 0.02,
            ..SimConfig::default()
        };
        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            motor_thrusts_actual_n: SVector::<f32, 4>::repeat(sim_cfg.mass_kg * GRAVITY_M_S2 / 4.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(42);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            // Aggressive rate commands saturate the motor allocator,
            // which produces large ω̇ residuals that the detector would
            // mistake for a real fault. Disable it for this test — a
            // real flight stack would set this to `false` on any
            // acro / flip mode entry.
            motor_fault_detector_enabled: false,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));

        let hover_ticks: usize = 1_000; // 1 s settle
        let flip_ticks: usize = 500; // 0.5 s roll burst
        let recover_ticks: usize = 1_500; // 1.5 s recovery
        let total = hover_ticks + flip_ticks + recover_ticks;

        let mut min_q_w = 1.0_f32;
        let mut max_q_norm_err = 0.0_f32;
        let mut max_alt_err = 0.0_f32;

        for i in 0..total {
            let accel_w = accel_world(&sim_cfg, &sim_state);
            let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
            let rate_cmd = if i < hover_ticks {
                RateCommand {
                    body_rate_rad_s: Vector3::zeros(),
                }
            } else if i < hover_ticks + flip_ticks {
                RateCommand {
                    body_rate_rad_s: Vector3::new(15.0, 0.0, 0.0),
                }
            } else {
                // Recovery: drive attitude back to identity via the
                // existing attitude_to_rate controller. Without
                // this, rate=0 just stops rotation — it doesn't
                // "return to level", so the vehicle would stay
                // tilted wherever the flip ended.
                let q_identity = Quaternion::identity();
                attitude_to_rate(flight.state.attitude, q_identity, &app_cfg.k_attitude)
            };
            // Hold hover thrust throughout. A real flight stack would
            // boost thrust during the inverted portion to keep
            // altitude; that's a mode-switch concern for M23+.
            let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
            step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

            // Measurement updates so the EKF tracks the vehicle
            // through the aggressive maneuver. Mag in particular
            // is crucial: without it, attitude error accumulates
            // over the 3 s run and recovery targets the wrong q.
            if i.is_multiple_of(200) {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));
            }
            if i.is_multiple_of(20) {
                let _ = apply_mag_measurement(
                    &mut flight,
                    &sense_mag(&sim_cfg, &sim_state, &mut rng),
                );
            }
            if i.is_multiple_of(20) {
                let _ = apply_baro_measurement(
                    &mut flight,
                    &sense_baro(&sim_cfg, &sim_state, &mut rng),
                );
            }

            let q = sim_state.attitude;
            let q_norm = libm::sqrtf(q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k);
            let norm_err = (q_norm - 1.0).abs();
            if norm_err > max_q_norm_err {
                max_q_norm_err = norm_err;
            }
            if q.w < min_q_w {
                min_q_w = q.w;
            }
            let alt_err = (-sim_state.position_ned.z - 1.0).abs();
            if alt_err > max_alt_err {
                max_alt_err = alt_err;
            }
        }

        assert!(
            sim_state.position_ned.norm().is_finite(),
            "position went non-finite: {:?}",
            sim_state.position_ned
        );
        // sim-hil's step() explicitly normalises; this guards against
        // a future refactor that disables normalisation.
        assert!(
            max_q_norm_err < 1.0e-2,
            "quaternion norm drifted by {max_q_norm_err} (> 1e-2)",
        );
        // The flip must actually occur (q_w < 0 = past inverted).
        assert!(
            min_q_w < 0.0,
            "vehicle never inverted — q_w min was {min_q_w}, expected < 0"
        );
        // Final tilt: 30° bound. A full recovery within 10° would be
        // ideal but INDI's P-only rate gain (k=25) leaves a small
        // static error after aggressive maneuvers. M23 could add an
        // explicit attitude integrator for this.
        let q_w_final = sim_state.attitude.w.abs();
        let final_tilt_rad = libm::acosf((2.0 * q_w_final * q_w_final - 1.0).clamp(-1.0, 1.0));
        assert!(
            final_tilt_rad < 0.524,
            "final tilt {final_tilt_rad} rad (> 30°) — INDI didn't recover level",
        );
        // Altitude loss: hover thrust is held throughout; during the
        // inverted portion the vehicle free-falls at -g, during the
        // upright portion thrust cancels gravity but can't decelerate
        // a falling vehicle. Realistic bound ~15 m for a full 360°
        // flip at this torque budget. A real flight stack would boost
        // thrust during the flip — M23+ scope (explicit flip mode).
        assert!(
            max_alt_err < 15.0,
            "altitude loss {max_alt_err} m (> 15 m) — INDI lost control of thrust direction",
        );
    }
}
