//! Host-side simulation harness.
//!
//! This crate provides a minimal-but-real 6-DoF rigid-body simulator for
//! running the flight controller in closed loop without hardware. It stays
//! independent of any specific engine (Gazebo, AerialGym, etc.) — they can
//! be plugged in on top as separate modules later.
//!
//! # What the simulator models
//!
//! * Rigid-body dynamics in NED: quaternion attitude, 3-vec velocity &
//!   position, per-axis angular velocity.
//! * Forces: sum of motor thrusts along `-body_z`, plus gravity.
//! * Torques: per-motor thrust × arm position (+ yaw reaction).
//! * IMU sensors: noise-free accelerometer + gyro derived from the true
//!   state (optional additive noise is a Follow-up).
//! * GPS / magnetometer / barometer: noise-free, at configurable rates.
//!
//! # What it does *not* model (yet)
//!
//! * Motor response lag / first-order ESC dynamics.
//! * Aerodynamic drag.
//! * Sensor noise — see Follow-ups.
//! * Turbulence / wind (`State.wind_ne` in the EKF is fed zero here).

use algo_ekf::{BaroMeasurement, GpsMeasurement, MagMeasurement, GRAVITY_M_S2};
use algo_indi::Inertia;
use core_hal::traits::ImuSample;
use nalgebra::{Matrix3, Quaternion, SVector, UnitQuaternion, Vector3};

pub mod gazebo;

/// Static simulator parameters (mass, inertia, geometry, noise).
#[derive(Clone, Debug)]
pub struct SimConfig {
    pub mass_kg: f32,
    pub inertia: Inertia,
    /// Per-motor body-frame positions (m). Thrust acts in `-body_z`.
    pub motor_positions: [Vector3<f32>; 4],
    /// Per-motor yaw-torque coefficient (Nm/N).
    pub motor_yaw_coef: [f32; 4],
    /// Earth magnetic field in NED (gauss).
    pub mag_earth_ned: Vector3<f32>,
}

impl Default for SimConfig {
    fn default() -> Self {
        let h = 0.15_f32 / core::f32::consts::SQRT_2;
        Self {
            mass_kg: 0.25,
            inertia: Matrix3::from_diagonal(&Vector3::new(0.015, 0.015, 0.025)),
            motor_positions: [
                Vector3::new(h, h, 0.0),
                Vector3::new(-h, -h, 0.0),
                Vector3::new(h, -h, 0.0),
                Vector3::new(-h, h, 0.0),
            ],
            motor_yaw_coef: [0.016, 0.016, -0.016, -0.016],
            mag_earth_ned: Vector3::new(0.21, 0.0, 0.43),
        }
    }
}

/// Simulator's ground-truth state.
#[derive(Clone, Copy, Debug)]
pub struct SimState {
    /// Body-to-world unit quaternion.
    pub attitude: Quaternion<f32>,
    /// Body-frame angular velocity, rad/s.
    pub body_rate_rad_s: Vector3<f32>,
    /// NED-frame velocity, m/s.
    pub velocity_ned: Vector3<f32>,
    /// NED-frame position, m.
    pub position_ned: Vector3<f32>,
    /// Elapsed simulated time, seconds.
    pub time_s: f64,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            attitude: Quaternion::identity(),
            body_rate_rad_s: Vector3::zeros(),
            velocity_ned: Vector3::zeros(),
            position_ned: Vector3::zeros(),
            time_s: 0.0,
        }
    }
}

/// Advance the simulated vehicle by `dt_s` given the four motor thrusts.
///
/// Euler integration (sufficient at 1 ms dt for our correctness tests;
/// RK4 is a follow-up).
pub fn step(cfg: &SimConfig, state: &mut SimState, motor_thrusts_n: &SVector<f32, 4>, dt_s: f32) {
    // --- Forces & torques in body frame ---------------------------------
    let mut total_thrust = 0.0_f32;
    let mut total_torque_body = Vector3::zeros();
    for (i, (pos, yaw_coef)) in
        cfg.motor_positions.iter().zip(cfg.motor_yaw_coef.iter()).enumerate()
    {
        let t_i = motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar();
        total_thrust += t_i;
        // τ = r × F,  F = (0, 0, -T) in body frame (thrust points -z_body)
        //    ⇒ τ_x = -y·T,  τ_y = +x·T
        total_torque_body.x += -pos.y * t_i;
        total_torque_body.y += pos.x * t_i;
        total_torque_body.z += yaw_coef * t_i;
    }
    let force_body = Vector3::new(0.0, 0.0, -total_thrust);

    // --- Translational dynamics in NED ----------------------------------
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    let accel_world = rot * force_body / cfg.mass_kg + Vector3::new(0.0, 0.0, GRAVITY_M_S2);
    let new_velocity = state.velocity_ned + accel_world * dt_s;
    let new_position = state.position_ned
        + state.velocity_ned * dt_s
        + accel_world * (0.5 * dt_s * dt_s);

    // --- Angular dynamics (body frame) ----------------------------------
    // τ = J·ω̇ + ω × (J·ω)   ⇒   ω̇ = J⁻¹·(τ − ω × (J·ω))
    let omega = state.body_rate_rad_s;
    let gyro_term = omega.cross(&(cfg.inertia * omega));
    let j_inv = cfg.inertia.try_inverse().unwrap_or_else(Matrix3::identity);
    let alpha = j_inv * (total_torque_body - gyro_term);
    let new_omega = omega + alpha * dt_s;

    // Integrate attitude: q̇ = ½ · q ⊗ (0, ω)
    let omega_quat = Quaternion::new(0.0, omega.x, omega.y, omega.z);
    let q_dot = state.attitude * omega_quat * 0.5;
    let mut new_q = Quaternion::new(
        state.attitude.w + q_dot.w * dt_s,
        state.attitude.i + q_dot.i * dt_s,
        state.attitude.j + q_dot.j * dt_s,
        state.attitude.k + q_dot.k * dt_s,
    );
    // Normalise to keep ‖q‖ = 1 (Euler integration drifts).
    let qn_sq = new_q.w * new_q.w + new_q.i * new_q.i + new_q.j * new_q.j + new_q.k * new_q.k;
    if qn_sq.is_finite() && qn_sq > 1.0e-12 {
        let qn = libm::sqrtf(qn_sq);
        new_q /= qn;
    }

    state.attitude = new_q;
    state.body_rate_rad_s = new_omega;
    state.velocity_ned = new_velocity;
    state.position_ned = new_position;
    state.time_s += f64::from(dt_s);
}

/// Produce an IMU sample that a real sensor would read given the true
/// state. Zero noise for now.
#[must_use]
pub fn sense_imu(state: &SimState, accel_world: Vector3<f32>) -> ImuSample {
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    let r_transpose = rot.to_rotation_matrix().matrix().transpose();
    // Specific force (accelerometer reading) = R^T · (accel_world − g)
    let specific_force_world = accel_world - Vector3::new(0.0, 0.0, GRAVITY_M_S2);
    let f_body = r_transpose * specific_force_world;
    #[allow(clippy::as_conversions)]
    let timestamp_us = if state.time_s.is_finite() && state.time_s >= 0.0 {
        // SAFETY: finite non-negative f64 at < 10^19 fits in u64 unambiguously.
        // Clamp at u64::MAX for extreme values.
        let micros = state.time_s * 1_000_000.0;
        if micros >= 1.844_674_4e19_f64 {
            u64::MAX
        } else {
            micros as u64
        }
    } else {
        0
    };
    ImuSample {
        timestamp_us,
        gyro_rad_s: state.body_rate_rad_s,
        accel_m_s2: f_body,
        temperature_c: 20.0,
    }
}

/// Compute the accel_world that would be produced next step (for use by
/// `sense_imu`). Call with the same `motor_thrusts_n` you passed to
/// [`step`], **before** calling step.
#[must_use]
pub fn accel_world_from_thrusts(
    cfg: &SimConfig,
    state: &SimState,
    motor_thrusts_n: &SVector<f32, 4>,
) -> Vector3<f32> {
    let total_thrust: f32 =
        (0..4).map(|i| motor_thrusts_n.fixed_view::<1, 1>(i, 0).to_scalar()).sum();
    let force_body = Vector3::new(0.0, 0.0, -total_thrust);
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    rot * force_body / cfg.mass_kg + Vector3::new(0.0, 0.0, GRAVITY_M_S2)
}

/// Ideal GPS reading (no noise).
#[must_use]
pub fn sense_gps(state: &SimState) -> GpsMeasurement {
    GpsMeasurement {
        position_ned: state.position_ned,
        sigma: Vector3::new(0.3, 0.3, 0.4),
    }
}

/// Ideal magnetometer reading (no noise).
#[must_use]
pub fn sense_mag(cfg: &SimConfig, state: &SimState) -> MagMeasurement {
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    let r_transpose = rot.to_rotation_matrix().matrix().transpose();
    MagMeasurement {
        body_field: r_transpose * cfg.mag_earth_ned,
        sigma: Vector3::new(0.01, 0.01, 0.01),
    }
}

/// Ideal barometer reading (no noise).
#[must_use]
pub fn sense_baro(state: &SimState) -> BaroMeasurement {
    BaroMeasurement {
        altitude_m: -state.position_ned.z,
        sigma_m: 0.1,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn hover_thrust_keeps_vehicle_stationary() {
        let cfg = SimConfig::default();
        let mut state = SimState::default();
        let hover_total = cfg.mass_kg * GRAVITY_M_S2;
        let per_motor = hover_total / 4.0;
        let thrusts = SVector::<f32, 4>::repeat(per_motor);
        for _ in 0..1000 {
            step(&cfg, &mut state, &thrusts, 0.001);
        }
        assert!(state.position_ned.norm() < 1.0e-3, "p = {}", state.position_ned);
        assert!(state.velocity_ned.norm() < 1.0e-3, "v = {}", state.velocity_ned);
        assert!((state.attitude.norm() - 1.0).abs() < 1.0e-5);
    }

    #[test]
    fn zero_thrust_is_free_fall() {
        let cfg = SimConfig::default();
        let mut state = SimState::default();
        let thrusts = SVector::<f32, 4>::zeros();
        for _ in 0..100 {
            step(&cfg, &mut state, &thrusts, 0.001);
        }
        // After 100 ms free fall: v_z = g·dt = 0.98 m/s,  z = ½·g·dt² ≈ 0.049 m
        assert!((state.velocity_ned.z - 0.98).abs() < 1.0e-3);
        assert!((state.position_ned.z - 0.049).abs() < 1.0e-3);
    }

    #[test]
    fn sensed_imu_matches_zero_for_level_hover() {
        let cfg = SimConfig::default();
        let state = SimState::default();
        let thrusts = SVector::<f32, 4>::repeat(cfg.mass_kg * GRAVITY_M_S2 / 4.0);
        let accel_w = accel_world_from_thrusts(&cfg, &state, &thrusts);
        let imu = sense_imu(&state, accel_w);
        // Level hover: accel_world ≈ 0, so specific force = −g, f_body = (0,0,−g)
        assert!((imu.accel_m_s2.z - (-GRAVITY_M_S2)).abs() < 1.0e-3);
        assert!(imu.accel_m_s2.x.abs() < 1.0e-3);
        assert!(imu.accel_m_s2.y.abs() < 1.0e-3);
    }

    #[test]
    fn closed_loop_sitl_hovers_with_app_copter() {
        use app_copter::{apply_baro_measurement, apply_gps_measurement, apply_mag_measurement, default_config_250g, outer_step, FlightState};
        use algo_nmpc::Setpoint;

        let sim_cfg = SimConfig::default();
        let mut sim_state = SimState::default();
        // Start at 1 m altitude so the filter has something non-trivial to
        // converge on.
        sim_state.position_ned.z = -1.0;

        let mut app_cfg = default_config_250g();
        let mut flight = FlightState::default();
        // Seed the EKF with a baro sample so altitude starts aligned.
        let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_state));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_state));

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        let dt = 0.001_f32;
        // 3 seconds of closed-loop SITL.
        for i in 0..3000 {
            // First compute expected accel for IMU sensing.
            let thrusts_prev = SVector::<f32, 4>::repeat(sim_cfg.mass_kg * GRAVITY_M_S2 / 4.0);
            let accel_w = accel_world_from_thrusts(&sim_cfg, &sim_state, &thrusts_prev);
            let imu = sense_imu(&sim_state, accel_w);
            // Run the autopilot to get motor thrusts.
            let out = outer_step(&mut app_cfg, &mut flight, imu, dt, &setpoint);
            // Apply thrusts to sim.
            step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);
            // Scheduled measurements.
            if i % 200 == 0 {
                let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_state));
            }
            if i % 40 == 0 {
                let _ = apply_mag_measurement(&mut flight, &sense_mag(&sim_cfg, &sim_state));
            }
            if i % 20 == 0 {
                let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_state));
            }
        }

        // After 3 s of closed-loop flight, the real vehicle should be near
        // the setpoint (1 m altitude). Open tolerance because P-P cascade
        // settles slowly.
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        assert!(altitude_err < 0.5, "altitude err = {altitude_err} m");
        // Horizontal position controlled too.
        assert!(sim_state.position_ned.xy().norm() < 0.5);
        // Attitude didn't diverge.
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-4);
    }
}
