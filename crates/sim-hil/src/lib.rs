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

mod rng;
pub use rng::SimRng;

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
    /// First-order motor lag time constant (s). Typical 15-30 ms.
    /// Set to 0 to keep the old "thrust changes instantly" behaviour.
    pub motor_tau_s: f32,
    /// Linear drag coefficient (N·s/m). `F_drag = -k_lin · v_rel`.
    pub drag_linear: f32,
    /// Quadratic drag coefficient (N·s²/m²). `F_drag += -k_quad · ‖v_rel‖ · v_rel`.
    pub drag_quadratic: f32,
    /// Constant wind velocity in NED (m/s). Drag uses `v_vehicle − wind`.
    pub wind_ned: Vector3<f32>,
    /// Sensor noise. See [`NoiseConfig`].
    pub noise: NoiseConfig,
}

/// 1-σ per-channel noise injected by `sense_*`. Zero = ideal sensor.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoiseConfig {
    pub gyro_sigma: f32,
    pub accel_sigma: f32,
    pub gps_pos_sigma: f32,
    pub mag_sigma: f32,
    pub baro_sigma: f32,
}

impl NoiseConfig {
    /// Realistic defaults for a consumer-grade MEMS IMU + u-blox GNSS +
    /// RM3100 mag + BMP388 baro.
    #[must_use]
    pub const fn realistic() -> Self {
        Self {
            gyro_sigma: 0.003,   // rad/s, ICM-42688 density × √BW
            accel_sigma: 0.05,   // m/s²
            gps_pos_sigma: 0.3,  // m
            mag_sigma: 0.003,    // gauss
            baro_sigma: 0.15,    // m
        }
    }
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
            motor_tau_s: 0.0,
            drag_linear: 0.0,
            drag_quadratic: 0.0,
            wind_ned: Vector3::zeros(),
            noise: NoiseConfig::default(),
        }
    }
}

impl SimConfig {
    /// Same geometry as [`Default::default`] but with realistic motor
    /// lag, drag, and (passed-in) wind; noise is kept off so the caller
    /// can dial it in independently.
    #[must_use]
    pub fn realistic_dynamics(wind_ned: Vector3<f32>) -> Self {
        Self {
            motor_tau_s: 0.02,
            drag_linear: 0.05,
            drag_quadratic: 0.02,
            wind_ned,
            ..Self::default()
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
    /// Actual motor thrusts (after first-order lag). Caller-commanded
    /// thrusts enter [`step`] and are filtered into here.
    pub motor_thrusts_actual_n: SVector<f32, 4>,
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
            motor_thrusts_actual_n: SVector::<f32, 4>::zeros(),
            time_s: 0.0,
        }
    }
}

/// Advance the simulated vehicle by `dt_s` given the four commanded motor
/// thrusts.
///
/// Euler integration (sufficient at 1 ms dt for our correctness tests;
/// RK4 is a follow-up). Handles first-order motor lag, linear +
/// quadratic drag, and a constant wind disturbance when configured.
pub fn step(cfg: &SimConfig, state: &mut SimState, motor_thrusts_cmd_n: &SVector<f32, 4>, dt_s: f32) {
    // --- Motor lag: actual thrust trails the command with τ ------------
    if cfg.motor_tau_s > 0.0 && dt_s > 0.0 {
        let alpha = (dt_s / (cfg.motor_tau_s + dt_s)).clamp(0.0, 1.0);
        state.motor_thrusts_actual_n += (motor_thrusts_cmd_n - state.motor_thrusts_actual_n) * alpha;
    } else {
        state.motor_thrusts_actual_n = *motor_thrusts_cmd_n;
    }
    let motor_thrusts_n = state.motor_thrusts_actual_n;

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
    let thrust_accel_world = rot * force_body / cfg.mass_kg;

    // Aerodynamic drag in NED, opposing velocity-relative-to-wind.
    let v_rel = state.velocity_ned - cfg.wind_ned;
    let v_rel_mag = v_rel.norm();
    let drag_world = if v_rel_mag > 0.0 && v_rel_mag.is_finite() {
        -(cfg.drag_linear + cfg.drag_quadratic * v_rel_mag) * v_rel
    } else {
        Vector3::zeros()
    };
    let drag_accel_world = drag_world / cfg.mass_kg;

    let accel_world = thrust_accel_world + drag_accel_world + Vector3::new(0.0, 0.0, GRAVITY_M_S2);
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

/// Produce an IMU sample that a real sensor would read. Injects
/// zero-mean Gaussian noise on each axis using `cfg.noise`. Use
/// [`NoiseConfig::default`] for noise-free output.
#[must_use]
pub fn sense_imu(
    cfg: &SimConfig,
    state: &SimState,
    accel_world: Vector3<f32>,
    rng: &mut SimRng,
) -> ImuSample {
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    let r_transpose = rot.to_rotation_matrix().matrix().transpose();
    let specific_force_world = accel_world - Vector3::new(0.0, 0.0, GRAVITY_M_S2);
    let f_body = r_transpose * specific_force_world;
    #[allow(clippy::as_conversions)]
    let timestamp_us = if state.time_s.is_finite() && state.time_s >= 0.0 {
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
        gyro_rad_s: state.body_rate_rad_s + rng.gaussian_vec3(cfg.noise.gyro_sigma),
        accel_m_s2: f_body + rng.gaussian_vec3(cfg.noise.accel_sigma),
        temperature_c: 20.0,
    }
}

/// Instantaneous world-frame acceleration produced by the current motor
/// thrusts + drag. Call **before** [`step`] so `sense_imu` sees the same
/// instant's acceleration.
#[must_use]
pub fn accel_world(cfg: &SimConfig, state: &SimState) -> Vector3<f32> {
    let total_thrust: f32 = (0..4)
        .map(|i| state.motor_thrusts_actual_n.fixed_view::<1, 1>(i, 0).to_scalar())
        .sum();
    let force_body = Vector3::new(0.0, 0.0, -total_thrust);
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    let thrust_accel_world = rot * force_body / cfg.mass_kg;

    let v_rel = state.velocity_ned - cfg.wind_ned;
    let v_rel_mag = v_rel.norm();
    let drag_world = if v_rel_mag > 0.0 && v_rel_mag.is_finite() {
        -(cfg.drag_linear + cfg.drag_quadratic * v_rel_mag) * v_rel
    } else {
        Vector3::zeros()
    };
    let drag_accel_world = drag_world / cfg.mass_kg;

    thrust_accel_world + drag_accel_world + Vector3::new(0.0, 0.0, GRAVITY_M_S2)
}

/// GPS position reading with additive noise per `cfg.noise.gps_pos_sigma`.
#[must_use]
pub fn sense_gps(cfg: &SimConfig, state: &SimState, rng: &mut SimRng) -> GpsMeasurement {
    let sigma = cfg.noise.gps_pos_sigma;
    let reported_sigma = sigma.max(0.1);
    GpsMeasurement {
        position_ned: state.position_ned + rng.gaussian_vec3(sigma),
        sigma: Vector3::new(reported_sigma, reported_sigma, reported_sigma * 1.3),
    }
}

/// Magnetometer body-frame reading with additive noise.
#[must_use]
pub fn sense_mag(cfg: &SimConfig, state: &SimState, rng: &mut SimRng) -> MagMeasurement {
    let rot = UnitQuaternion::new_unchecked(state.attitude);
    let r_transpose = rot.to_rotation_matrix().matrix().transpose();
    let reported = cfg.noise.mag_sigma.max(0.001);
    MagMeasurement {
        body_field: r_transpose * cfg.mag_earth_ned + rng.gaussian_vec3(cfg.noise.mag_sigma),
        sigma: Vector3::new(reported, reported, reported),
    }
}

/// Barometer altitude reading with additive noise.
#[must_use]
pub fn sense_baro(cfg: &SimConfig, state: &SimState, rng: &mut SimRng) -> BaroMeasurement {
    BaroMeasurement {
        altitude_m: -state.position_ned.z + rng.gaussian() * cfg.noise.baro_sigma,
        sigma_m: cfg.noise.baro_sigma.max(0.05),
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
        let state = SimState {
            motor_thrusts_actual_n: SVector::<f32, 4>::repeat(cfg.mass_kg * GRAVITY_M_S2 / 4.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(1);
        let accel_w = accel_world(&cfg, &state);
        let imu = sense_imu(&cfg, &state, accel_w, &mut rng);
        // Level hover with zero noise: accel_world ≈ 0, f_body = (0,0,−g)
        assert!((imu.accel_m_s2.z - (-GRAVITY_M_S2)).abs() < 1.0e-3);
        assert!(imu.accel_m_s2.x.abs() < 1.0e-3);
        assert!(imu.accel_m_s2.y.abs() < 1.0e-3);
    }

    fn run_closed_loop_flight(sim_cfg: &SimConfig, seed: u64, steps: usize) -> SimState {
        use app_copter::{
            apply_baro_measurement, apply_gps_measurement, apply_mag_measurement,
            default_config_250g, outer_step, FlightState,
        };
        use algo_nmpc::Setpoint;

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);

        let mut app_cfg = default_config_250g();
        let mut flight = FlightState::default();
        let _ = apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        let dt = 0.001_f32;
        for i in 0..steps {
            let accel_w = accel_world(sim_cfg, &sim_state);
            let imu = sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);
            let out = outer_step(&mut app_cfg, &mut flight, imu, dt, &setpoint);
            step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);
            if i % 200 == 0 {
                let _ = apply_gps_measurement(
                    &mut flight,
                    &sense_gps(sim_cfg, &sim_state, &mut rng),
                );
            }
            if i % 40 == 0 {
                let _ = apply_mag_measurement(
                    &mut flight,
                    &sense_mag(sim_cfg, &sim_state, &mut rng),
                );
            }
            if i % 20 == 0 {
                let _ = apply_baro_measurement(
                    &mut flight,
                    &sense_baro(sim_cfg, &sim_state, &mut rng),
                );
            }
        }
        sim_state
    }

    #[test]
    fn closed_loop_sitl_hovers_with_app_copter() {
        // Ideal sensors, no drag, no lag — fast convergence baseline.
        let sim_cfg = SimConfig::default();
        let sim_state = run_closed_loop_flight(&sim_cfg, 1, 3000);

        // After 3 s of closed-loop flight, the real vehicle should be near
        // the setpoint (1 m altitude). Open tolerance because P-P cascade
        // settles slowly.
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        assert!(altitude_err < 0.5, "altitude err = {altitude_err} m");
        assert!(sim_state.position_ned.xy().norm() < 0.5);
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-4);
    }

    #[test]
    fn closed_loop_sitl_with_realistic_noise_and_dynamics() {
        // Real-world conditions: sensor noise + motor lag + linear drag.
        // Assertion is "doesn't diverge" — the P-P cascade has a known
        // steady-state bias under drag + lag + noise; integral terms
        // (M3.3) tighten this.
        let mut sim_cfg = SimConfig::realistic_dynamics(Vector3::zeros());
        sim_cfg.noise = NoiseConfig::realistic();
        let sim_state = run_closed_loop_flight(&sim_cfg, 42, 5000); // 5 s
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        assert!(altitude_err < 3.0, "altitude err = {altitude_err} m (diverged)");
        assert!(sim_state.position_ned.xy().norm() < 3.0,
            "horiz = {} m (diverged)", sim_state.position_ned.xy().norm());
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-3);
        // Velocity should not be growing unboundedly.
        assert!(sim_state.velocity_ned.norm() < 5.0,
            "velocity = {} m/s (diverged)", sim_state.velocity_ned.norm());
    }

    #[test]
    fn closed_loop_sitl_with_steady_wind() {
        // 3 m/s head-wind along +x; drag pulls vehicle back.
        let sim_cfg = SimConfig::realistic_dynamics(Vector3::new(3.0, 0.0, 0.0));
        let sim_state = run_closed_loop_flight(&sim_cfg, 7, 5000);
        // With 3 m/s wind + linear drag k=0.05 N·s/m, cascade-P steady
        // state has a bias. Just assert vehicle hasn't flown away.
        assert!(sim_state.position_ned.norm() < 5.0, "drift = {}", sim_state.position_ned);
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-3);
    }
}
