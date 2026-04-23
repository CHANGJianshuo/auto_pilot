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
