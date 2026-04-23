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

use algo_ekf::{BaroMeasurement, GRAVITY_M_S2, GpsMeasurement, MagMeasurement};
use algo_indi::Inertia;
use core_hal::traits::ImuSample;
use nalgebra::{Matrix3, Quaternion, SVector, UnitQuaternion, Vector3};

mod rng;
pub use rng::SimRng;

pub mod mavlink_udp;

pub mod gazebo;

/// Host-side wrapper that pairs the M9.2 MPC controller with the M11a
/// residual policy, producing an "NMPC + NN-compensated" controller.
pub mod residual_mpc;

/// Host-side Zenoh pub/sub wrapper binding `core_bus` messages to a
/// live Zenoh session. Opt-in via the `zenoh-host` feature because the
/// dep graph is heavy.
#[cfg(feature = "zenoh-host")]
pub mod zenoh_bus;

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
            gyro_sigma: 0.003,  // rad/s, ICM-42688 density × √BW
            accel_sigma: 0.05,  // m/s²
            gps_pos_sigma: 0.3, // m
            mag_sigma: 0.003,   // gauss
            baro_sigma: 0.15,   // m
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
pub fn step(
    cfg: &SimConfig,
    state: &mut SimState,
    motor_thrusts_cmd_n: &SVector<f32, 4>,
    dt_s: f32,
) {
    // --- Motor lag: actual thrust trails the command with τ ------------
    if cfg.motor_tau_s > 0.0 && dt_s > 0.0 {
        let alpha = (dt_s / (cfg.motor_tau_s + dt_s)).clamp(0.0, 1.0);
        state.motor_thrusts_actual_n +=
            (motor_thrusts_cmd_n - state.motor_thrusts_actual_n) * alpha;
    } else {
        state.motor_thrusts_actual_n = *motor_thrusts_cmd_n;
    }
    let motor_thrusts_n = state.motor_thrusts_actual_n;

    // --- Forces & torques in body frame ---------------------------------
    let mut total_thrust = 0.0_f32;
    let mut total_torque_body = Vector3::zeros();
    for (i, (pos, yaw_coef)) in cfg
        .motor_positions
        .iter()
        .zip(cfg.motor_yaw_coef.iter())
        .enumerate()
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
    let new_position =
        state.position_ned + state.velocity_ned * dt_s + accel_world * (0.5 * dt_s * dt_s);

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
        .map(|i| {
            state
                .motor_thrusts_actual_n
                .fixed_view::<1, 1>(i, 0)
                .to_scalar()
        })
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
    sense_gps_with_fault(cfg, state, rng, GpsFault::None)
}

/// Optional GPS fault injected for a single observation.
#[derive(Clone, Copy, Debug)]
pub enum GpsFault {
    /// No fault — normal noisy reading.
    None,
    /// Add a constant world-frame offset (m) on top of the real position.
    /// Useful for modelling multipath or spoofing.
    Offset(Vector3<f32>),
    /// Sensor stuck at an absolute position. Simulates hung driver.
    Stuck(Vector3<f32>),
}

/// GPS reading with an optional injected fault mode. Call this from test
/// / simulation drivers that want to exercise FDIR.
#[must_use]
pub fn sense_gps_with_fault(
    cfg: &SimConfig,
    state: &SimState,
    rng: &mut SimRng,
    fault: GpsFault,
) -> GpsMeasurement {
    let sigma = cfg.noise.gps_pos_sigma;
    let reported_sigma = sigma.max(0.1);
    let position_ned = match fault {
        GpsFault::None => state.position_ned + rng.gaussian_vec3(sigma),
        GpsFault::Offset(off) => state.position_ned + off + rng.gaussian_vec3(sigma),
        GpsFault::Stuck(p) => p,
    };
    GpsMeasurement {
        position_ned,
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
        assert!(
            state.position_ned.norm() < 1.0e-3,
            "p = {}",
            state.position_ned
        );
        assert!(
            state.velocity_ned.norm() < 1.0e-3,
            "v = {}",
            state.velocity_ned
        );
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
        use algo_nmpc::Setpoint;
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, outer_step,
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);

        let mut app_cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
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
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ =
                    apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
            }
        }
        sim_state
    }

    /// Runs the SITL against an `app-copter` whose position gains are
    /// LQR-derived rather than hand-tuned PI. Used by the M9.0 smoke test
    /// below. Parameterised over the weight tuning so the same harness
    /// can exercise different operating points.
    fn run_closed_loop_flight_with_lqr(
        sim_cfg: &SimConfig,
        seed: u64,
        steps: usize,
        weights_xy: algo_nmpc::LqrWeights,
        weights_z: algo_nmpc::LqrWeights,
    ) -> SimState {
        use algo_nmpc::{Setpoint, lqr_position_gains};
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, outer_step,
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);

        let mut app_cfg = default_config_250g();
        // Outer loop runs at 1 kHz in this test harness (matches
        // run_closed_loop_flight above), so use dt = 0.001 for the LQR
        // design as well.
        let lqr_pg = lqr_position_gains(
            weights_xy,
            weights_z,
            0.001_f32,
            app_cfg.position_gains.max_accel,
        )
        .expect("LQR DARE converges for sane weights");
        app_cfg.position_gains = lqr_pg;

        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
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
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ =
                    apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
            }
        }
        sim_state
    }

    /// Runs the SITL with the M9.2 `Mpc3dPositionController` driving the
    /// outer loop (bypassing `app-copter::outer_step`'s PI-locked path,
    /// since the controller is the thing under test). Inner loop
    /// (attitude → rate → INDI → allocation) is unchanged.
    fn run_closed_loop_flight_with_mpc(
        sim_cfg: &SimConfig,
        seed: u64,
        steps: usize,
        weights_xy: algo_nmpc::LqrWeights,
        weights_z: algo_nmpc::LqrWeights,
        u_abs_max: f32,
    ) -> SimState {
        use algo_indi::attitude_to_rate;
        use algo_nmpc::{Mpc1dConfig, Mpc3dPositionController, Setpoint};
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();

        let cfg_xy = Mpc1dConfig {
            weights: weights_xy,
            dt_s: dt,
            u_min: -u_abs_max,
            u_max: u_abs_max,
        };
        let cfg_z = Mpc1dConfig {
            weights: weights_z,
            dt_s: dt,
            u_min: -u_abs_max,
            u_max: u_abs_max,
        };
        let mut mpc =
            Mpc3dPositionController::<10>::new(cfg_xy, cfg_z, 25, app_cfg.position_gains.max_accel)
                .expect("MPC QP data constructs for sane weights");

        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        for i in 0..steps {
            let accel_w = accel_world(sim_cfg, &sim_state);
            let imu = sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);

            // Outer loop: MPC supplies attitude + thrust.
            let att = mpc.step(
                &setpoint,
                flight.state.position_ned,
                flight.state.velocity_ned,
                app_cfg.mass_kg,
            );
            let rate_cmd =
                attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);

            // Thrust is the outer loop's demand for this tick.
            let saved_hover = app_cfg.hover_thrust_n;
            app_cfg.hover_thrust_n = att.thrust_n;
            let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
            app_cfg.hover_thrust_n = saved_hover;

            step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);
            if i % 200 == 0 {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ =
                    apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
            }
        }
        sim_state
    }

    /// Drive the SITL with an `Lqi3dPositionController` instead of the
    /// app-copter PI cascade. Similar shape to the MPC harness above;
    /// used by the M9.3 tests below.
    fn run_closed_loop_flight_with_lqi(
        sim_cfg: &SimConfig,
        seed: u64,
        steps: usize,
        weights_xy: algo_nmpc::LqiWeights,
        weights_z: algo_nmpc::LqiWeights,
    ) -> SimState {
        use algo_indi::attitude_to_rate;
        use algo_nmpc::{Lqi3dPositionController, Setpoint};
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();

        let mut lqi = Lqi3dPositionController::new(
            weights_xy,
            weights_z,
            dt,
            app_cfg.position_gains.max_accel,
            5.0,
        )
        .expect("LQI DAREs converge for sane weights");

        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        for i in 0..steps {
            let accel_w = accel_world(sim_cfg, &sim_state);
            let imu = sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);
            let att = lqi.step(
                &setpoint,
                flight.state.position_ned,
                flight.state.velocity_ned,
                app_cfg.mass_kg,
            );
            let rate_cmd =
                attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
            let saved_hover = app_cfg.hover_thrust_n;
            app_cfg.hover_thrust_n = att.thrust_n;
            let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
            app_cfg.hover_thrust_n = saved_hover;
            step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);
            if i % 200 == 0 {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ =
                    apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
            }
        }
        sim_state
    }

    #[test]
    fn lqi_hover_under_drag_and_wind() {
        // Realistic sim: 2 m/s wind + linear + quadratic drag. LQR (no
        // integrator) would show a sustained steady-state bias; LQI's
        // integrator drives it to near-zero over 15 s.
        let sim_cfg = SimConfig::realistic_dynamics(Vector3::new(2.0, -1.0, 0.0));
        let weights = algo_nmpc::LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 1.5,
            r: 0.5,
        };
        let sim_state = run_closed_loop_flight_with_lqi(&sim_cfg, 1, 15_000, weights, weights);
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        let horizontal_err =
            (sim_state.position_ned.x.powi(2) + sim_state.position_ned.y.powi(2)).sqrt();
        assert!(
            altitude_err < 0.3,
            "LQI altitude err {altitude_err} m ≥ 0.30 m under drag+wind"
        );
        assert!(
            horizontal_err < 1.0,
            "LQI horizontal err {horizontal_err} m ≥ 1.0 m under drag+wind"
        );
    }

    /// Drive the SITL with any `PositionController` variant. Shared
    /// harness for the M9.4 shootout — one function, four controllers.
    fn run_closed_loop_with_controller<const H: usize>(
        sim_cfg: &SimConfig,
        seed: u64,
        steps: usize,
        mut controller: algo_nmpc::PositionController<H>,
    ) -> SimState {
        use algo_indi::attitude_to_rate;
        use algo_nmpc::Setpoint;
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();

        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        for i in 0..steps {
            let accel_w = accel_world(sim_cfg, &sim_state);
            let imu = sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);

            let att = controller.step(
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
            step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

            if i % 200 == 0 {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ =
                    apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
            }
        }
        sim_state
    }

    #[test]
    fn controller_shootout_ideal_sim_all_variants_hover() {
        // Ideal sim: every controller should keep the vehicle within
        // 25 cm of the 1 m altitude setpoint in 3 s.
        use algo_nmpc::{LqiWeights, LqrWeights, Mpc1dConfig, PositionController, PositionGains};
        let sim_cfg = SimConfig::default();
        let dt = 0.001_f32;

        let controllers: [PositionController<10>; 4] = [
            PositionController::pi(PositionGains::default()),
            PositionController::lqr(
                LqrWeights::default(),
                LqrWeights::default(),
                dt,
                PositionGains::default().max_accel,
            )
            .unwrap(),
            PositionController::mpc(
                Mpc1dConfig {
                    weights: LqrWeights::default(),
                    dt_s: dt,
                    u_min: -20.0,
                    u_max: 20.0,
                },
                Mpc1dConfig {
                    weights: LqrWeights::default(),
                    dt_s: dt,
                    u_min: -20.0,
                    u_max: 20.0,
                },
                25,
                PositionGains::default().max_accel,
            )
            .unwrap(),
            PositionController::lqi(
                LqiWeights::default(),
                LqiWeights::default(),
                dt,
                PositionGains::default().max_accel,
                5.0,
            )
            .unwrap(),
        ];
        for ctrl in controllers {
            let kind = ctrl.kind();
            let sim = run_closed_loop_with_controller(&sim_cfg, 1, 3000, ctrl);
            let alt_err = (-sim.position_ned.z - 1.0).abs();
            let horiz_err = (sim.position_ned.x.powi(2) + sim.position_ned.y.powi(2)).sqrt();
            assert!(
                alt_err < 0.25 && horiz_err < 0.25,
                "{kind} alt {alt_err} horiz {horiz_err}"
            );
        }
    }

    #[test]
    fn controller_shootout_realistic_sim_bias_tolerant_variants_win() {
        // Realistic sim with drag + wind: only controllers with
        // integral action (PI with k_i>0 and LQI) can eliminate the
        // steady-state bias. We assert the integrating variants land
        // inside 0.6 m altitude error, while LQR (bias-blind) is given
        // the looser 1 m bound — the point of this test is to document
        // the regime separation, not to fail on LQR.
        use algo_nmpc::{LqiWeights, LqrWeights, PositionController, PositionGains};
        let sim_cfg = SimConfig::realistic_dynamics(Vector3::new(2.0, -1.0, 0.0));
        let dt = 0.001_f32;

        // Integrating controllers — should hold tight.
        let lqi_weights = LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 1.5,
            r: 0.5,
        };
        let mpci_cfg = algo_nmpc::Mpc1dIConfig {
            weights: lqi_weights,
            dt_s: dt,
            u_min: -20.0,
            u_max: 20.0,
        };
        let tight_cases: [(PositionController<10>, &str); 3] = [
            (
                PositionController::pi(PositionGains {
                    k_i_vel: Vector3::new(0.5, 0.5, 0.8),
                    ..PositionGains::default()
                }),
                "PI+I",
            ),
            (
                PositionController::lqi(
                    lqi_weights,
                    lqi_weights,
                    dt,
                    PositionGains::default().max_accel,
                    5.0,
                )
                .unwrap(),
                "LQI",
            ),
            (
                PositionController::mpc_i(
                    mpci_cfg,
                    mpci_cfg,
                    25,
                    PositionGains::default().max_accel,
                    5.0,
                )
                .unwrap(),
                "MPC-I",
            ),
        ];
        for (ctrl, name) in tight_cases {
            let sim = run_closed_loop_with_controller(&sim_cfg, 7, 15_000, ctrl);
            let alt_err = (-sim.position_ned.z - 1.0).abs();
            assert!(alt_err < 0.6, "{name} alt_err {alt_err}m ≥ 0.6m");
        }

        // Bias-blind LQR — loose bound, this is the regime limitation.
        let lqr = PositionController::<10>::lqr(
            LqrWeights::default(),
            LqrWeights::default(),
            dt,
            PositionGains::default().max_accel,
        )
        .unwrap();
        let sim = run_closed_loop_with_controller(&sim_cfg, 7, 15_000, lqr);
        let alt_err = (-sim.position_ned.z - 1.0).abs();
        assert!(
            alt_err < 2.0,
            "LQR alt_err {alt_err}m ≥ 2.0m (stability lower bound)"
        );
    }

    /// Drive the SITL with an MPC + residual policy controller. The
    /// caller supplies the backend, so this harness also serves as the
    /// template for future `TractBackend` / `CandleBackend` testing.
    fn run_closed_loop_with_mpc_residual<B>(
        sim_cfg: &SimConfig,
        seed: u64,
        steps: usize,
        weights_xy: algo_nmpc::LqrWeights,
        weights_z: algo_nmpc::LqrWeights,
        policy_backend: B,
    ) -> SimState
    where
        B: nn_runtime::InferenceBackend,
    {
        use algo_indi::attitude_to_rate;
        use algo_nmpc::{Mpc1dConfig, Mpc3dPositionController, Setpoint};
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, rate_loop_step,
        };
        use nn_runtime::{ResidualPolicy, SafetyEnvelope};

        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(seed);
        let dt = 0.001_f32;
        let mut app_cfg = default_config_250g();

        let cfg_xy = Mpc1dConfig {
            weights: weights_xy,
            dt_s: dt,
            u_min: -20.0,
            u_max: 20.0,
        };
        let cfg_z = Mpc1dConfig {
            weights: weights_z,
            dt_s: dt,
            u_min: -20.0,
            u_max: 20.0,
        };
        let mpc =
            Mpc3dPositionController::<10>::new(cfg_xy, cfg_z, 25, app_cfg.position_gains.max_accel)
                .expect("MPC constructs for sane weights");
        let policy =
            ResidualPolicy::new(policy_backend, SafetyEnvelope::small_multirotor_default());
        let mut residual_mpc = crate::residual_mpc::MpcResidualController::new(mpc, policy);

        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        for i in 0..steps {
            let accel_w = accel_world(sim_cfg, &sim_state);
            let imu = sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);
            let att = residual_mpc.step(
                &setpoint,
                flight.state.position_ned,
                flight.state.velocity_ned,
                flight.state.attitude,
                app_cfg.mass_kg,
            );
            let rate_cmd =
                attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
            let saved_hover = app_cfg.hover_thrust_n;
            app_cfg.hover_thrust_n = att.thrust_n;
            let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
            app_cfg.hover_thrust_n = saved_hover;
            step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);
            if i % 200 == 0 {
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ =
                    apply_baro_measurement(&mut flight, &sense_baro(sim_cfg, &sim_state, &mut rng));
            }
        }
        sim_state
    }

    #[test]
    fn mpc_plus_residual_beats_bare_mpc_under_drag() {
        // Realistic sim (motor lag + drag + wind) in which bare MPC
        // shows a drag-induced steady-state bias (LQR/MPC are bias-
        // blind; M9.3 introduced LQI to fix this).
        //
        // Here we use a *hand-crafted* affine policy that mirrors the
        // role the integrator would play: input feature vel_z → output
        // residual accel_z that pushes up. No training involved — the
        // point is to verify the data path end-to-end and demonstrate
        // that the residual-policy slot can close the steady bias gap
        // that plain MPC leaves open.
        use algo_nmpc::LqrWeights;
        use nn_runtime::{AffineBackend, FEATURE_LEN, RESIDUAL_LEN};

        let sim_cfg = SimConfig::realistic_dynamics(Vector3::new(1.5, 0.0, 0.0));
        let weights = LqrWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            r: 0.5,
        };

        // Baseline: bare MPC (residual = 0) in the same sim.
        let baseline = AffineBackend::zero();
        let bare_sim =
            run_closed_loop_with_mpc_residual(&sim_cfg, 11, 15_000, weights, weights, baseline);
        let bare_alt_err = (-bare_sim.position_ned.z - 1.0).abs();
        let bare_horiz_err =
            (bare_sim.position_ned.x.powi(2) + bare_sim.position_ned.y.powi(2)).sqrt();

        // With residual: hand-tune a single-axis correction. Two
        // features fight the wind-drag bias: (a) x-position error
        // pulls the residual toward the setpoint (proportional to
        // pos_err), (b) x-velocity contributes damping. Coefficients
        // picked conservatively — the envelope still owns the hard
        // safety bound.
        let mut w = [[0.0_f32; FEATURE_LEN]; RESIDUAL_LEN];
        w[0][0] = -2.0; // pos_err_x → residual_x (restoring force)
        w[0][3] = -1.0; // vel_x     → residual_x (damping)
        let tuned = AffineBackend::new(w, [0.0; RESIDUAL_LEN], 5.0);
        let residual_sim =
            run_closed_loop_with_mpc_residual(&sim_cfg, 11, 15_000, weights, weights, tuned);
        let residual_alt_err = (-residual_sim.position_ned.z - 1.0).abs();
        let residual_horiz_err =
            (residual_sim.position_ned.x.powi(2) + residual_sim.position_ned.y.powi(2)).sqrt();

        // The residual should reduce the horizontal tracking error,
        // which is dominated by the wind-drag bias. We give the
        // comparison a 20 cm margin so small sim-level noise can't
        // flip the result.
        assert!(
            residual_horiz_err + 0.20 < bare_horiz_err,
            "residual did not help: bare {bare_horiz_err} m vs residual {residual_horiz_err} m"
        );
        // Altitude error shouldn't grow appreciably — the residual is
        // targeted at the x axis, so z tracking should stay similar.
        assert!(
            residual_alt_err <= bare_alt_err + 0.10,
            "residual made altitude worse: bare {bare_alt_err} vs residual {residual_alt_err}"
        );
    }

    #[test]
    fn mpc_closed_loop_sitl_hovers_to_setpoint() {
        // M9.2 smoke test: three-axis MPC controller hovers the vehicle
        // at the 1 m setpoint in ideal sim, same bound as LQR baseline.
        let sim_cfg = SimConfig::default();
        let weights = algo_nmpc::LqrWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            r: 0.5,
        };
        let sim_state = run_closed_loop_flight_with_mpc(&sim_cfg, 1, 3000, weights, weights, 20.0);
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        let horizontal_err =
            (sim_state.position_ned.x.powi(2) + sim_state.position_ned.y.powi(2)).sqrt();
        assert!(
            altitude_err < 0.25,
            "MPC altitude err {altitude_err} m ≥ 0.25 m"
        );
        assert!(
            horizontal_err < 0.25,
            "MPC horizontal err {horizontal_err} m ≥ 0.25 m"
        );
    }

    #[test]
    fn mpc_tight_u_max_still_stable() {
        // With u_max cranked down to 2 m/s² (below what LQR would
        // command from a 1 m initial error), PI / LQR would either
        // saturate-and-clamp (PI) or blindly project (LQR). MPC
        // predicts the constraint and stays stable — assert that the
        // vehicle still lands at the setpoint within a looser tolerance.
        let sim_cfg = SimConfig::default();
        let weights = algo_nmpc::LqrWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            r: 0.5,
        };
        let sim_state = run_closed_loop_flight_with_mpc(&sim_cfg, 1, 8000, weights, weights, 2.0);
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        assert!(
            altitude_err < 0.5,
            "MPC tight-box altitude err {altitude_err} m ≥ 0.5 m"
        );
    }

    #[test]
    fn lqr_closed_loop_sitl_hovers_to_setpoint() {
        // M9.0 smoke test: with LQR-derived gains the vehicle should
        // still hover at the 1 m setpoint. Uses the ideal sim to keep
        // the assertion bound tight — M9.1 (constraints) and M9.2
        // (receding horizon) will layer noise/drag tolerance on top.
        let sim_cfg = SimConfig::default();
        let weights = algo_nmpc::LqrWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            r: 0.5,
        };
        let sim_state = run_closed_loop_flight_with_lqr(&sim_cfg, 1, 3000, weights, weights);
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        let horizontal_err =
            (sim_state.position_ned.x.powi(2) + sim_state.position_ned.y.powi(2)).sqrt();
        assert!(
            altitude_err < 0.25,
            "LQR altitude err {altitude_err} m ≥ 0.25 m"
        );
        assert!(
            horizontal_err < 0.25,
            "LQR horizontal err {horizontal_err} m ≥ 0.25 m"
        );
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
        // M3.2 added the integral term on the velocity loop; steady-state
        // bias under drag/lag is driven to ~cm level over a few seconds.
        let mut sim_cfg = SimConfig::realistic_dynamics(Vector3::zeros());
        sim_cfg.noise = NoiseConfig::realistic();
        let sim_state = run_closed_loop_flight(&sim_cfg, 42, 8000); // 8 s
        let altitude_err = (-sim_state.position_ned.z - 1.0).abs();
        // Integral action: tight tolerance even under full realism.
        assert!(altitude_err < 0.6, "altitude err = {altitude_err} m");
        assert!(
            sim_state.position_ned.xy().norm() < 1.0,
            "horiz = {} m",
            sim_state.position_ned.xy().norm()
        );
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-3);
        assert!(
            sim_state.velocity_ned.norm() < 1.0,
            "velocity = {} m/s (not settled)",
            sim_state.velocity_ned.norm()
        );
    }

    #[test]
    fn ekf_identifies_wind_with_drag_aware_predict_open_loop() {
        // Open-loop wind identification: motors stay at hover, vehicle
        // drifts in the wind, GPS sees the drift, EKF's wind_ne state
        // tracks the true wind direction through the drag-aware Jacobian.
        //
        // Closed-loop testing is misleading here — a well-tuned
        // controller cancels the drift so fast that wind_ne residual
        // never grows.
        use algo_ekf::{ProcessNoise, predict_step_with_drag};
        use app_copter::{
            FlightState, apply_baro_measurement, apply_gps_measurement, apply_mag_measurement,
        };

        let true_wind = Vector3::new(2.0, -1.0, 0.0);
        let sim_cfg = SimConfig::realistic_dynamics(true_wind);
        let mut sim_state = SimState::default();
        let mut rng = SimRng::new(1234);

        let mut flight = FlightState::default();
        // Prime with initial baro + gps so the filter knows position.
        let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));

        // Bump wind process noise so the filter believes wind can change
        // fast enough to track within a few seconds.
        let noise = ProcessNoise {
            wind_per_s: 0.2,
            ..ProcessNoise::default()
        };
        let drag = 0.2_f32;
        // Fix motors at hover.
        let hover_thrusts = SVector::<f32, 4>::repeat(sim_cfg.mass_kg * GRAVITY_M_S2 / 4.0);

        let dt = 0.001_f32;
        for i in 0..15_000 {
            let accel_w = accel_world(&sim_cfg, &sim_state);
            let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
            // Drive the EKF directly.
            let measurement = algo_ekf::ImuMeasurement {
                gyro_rad_s: imu.gyro_rad_s,
                accel_m_s2: imu.accel_m_s2,
            };
            let (next_state, next_cov) = predict_step_with_drag(
                &flight.state,
                &flight.covariance,
                measurement,
                noise,
                dt,
                drag,
            );
            flight.state = next_state;
            flight.covariance = next_cov;
            // Physics uses fixed hover thrust (open loop).
            step(&sim_cfg, &mut sim_state, &hover_thrusts, dt);
            if i % 100 == 0 {
                // More frequent GPS than normal to accelerate convergence.
                let _ =
                    apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(&sim_cfg, &sim_state, &mut rng));
            }
        }

        // The true wind produces sim drift that the EKF explains via
        // wind_ne. Direction should match; magnitude may be off by a
        // factor of the drag-model mismatch (linear-only in EKF, linear
        // + quadratic in sim), so we assert on direction alignment.
        let est = flight.state.wind_ne;
        let true_ne = nalgebra::Vector2::new(true_wind.x, true_wind.y);
        let dot = est.dot(&true_ne) / (est.norm() * true_ne.norm() + 1.0e-6);
        assert!(est.norm() > 0.3, "wind estimate too small: {est:?}");
        assert!(
            dot > 0.5,
            "wind direction mismatch: est={est:?} truth={true_ne:?} cos={dot}"
        );
    }

    #[test]
    fn gps_outliers_degrade_sensor_health() {
        use algo_fdir::HealthLevel;
        use algo_nmpc::Setpoint;
        use app_copter::{
            ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
            apply_mag_measurement, default_config_250g, outer_step,
        };

        let sim_cfg = SimConfig::default();
        let mut sim_state = SimState {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..SimState::default()
        };
        let mut rng = SimRng::new(99);
        let mut app_cfg = default_config_250g();
        let mut flight = FlightState {
            arm_state: ArmState::Armed,
            ..FlightState::default()
        };
        let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
        let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));

        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };

        let dt = 0.001_f32;
        // Inject sustained GPS outliers during the second half of the run.
        let outlier_offset = Vector3::new(500.0, 0.0, 0.0); // way out of any χ² gate
        for i in 0..6000 {
            let accel_w = accel_world(&sim_cfg, &sim_state);
            let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
            let out = outer_step(&mut app_cfg, &mut flight, imu, dt, &setpoint);
            step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);
            if i % 200 == 0 {
                let fault = if i >= 2000 {
                    GpsFault::Offset(outlier_offset)
                } else {
                    GpsFault::None
                };
                let gps = sense_gps_with_fault(&sim_cfg, &sim_state, &mut rng, fault);
                let _ = apply_gps_measurement(&mut flight, &gps);
            }
            if i % 40 == 0 {
                let _ =
                    apply_mag_measurement(&mut flight, &sense_mag(&sim_cfg, &sim_state, &mut rng));
            }
            if i % 20 == 0 {
                let _ = apply_baro_measurement(
                    &mut flight,
                    &sense_baro(&sim_cfg, &sim_state, &mut rng),
                );
            }
        }

        // After 4 s of 200-ms-spaced outliers (20 outlier samples), the
        // GPS health counter should have escalated past Healthy.
        let gps_lvl = flight.gps_health.level();
        let overall = flight.overall_health();
        assert!(
            gps_lvl.severity() >= HealthLevel::Degraded.severity(),
            "gps health stayed {gps_lvl:?}, streak {}",
            flight.gps_health.streak()
        );
        assert!(
            overall.severity() >= gps_lvl.severity(),
            "overall health {overall:?} should track worst source"
        );
        // Vehicle didn't fall out of the sky — mag + baro still anchor
        // attitude and altitude.
        assert!(
            sim_state.position_ned.norm() < 50.0,
            "vehicle lost control: pos = {}",
            sim_state.position_ned
        );
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-3);
    }

    #[test]
    fn closed_loop_sitl_with_steady_wind() {
        // 3 m/s head-wind along +x; drag pulls vehicle back.
        let sim_cfg = SimConfig::realistic_dynamics(Vector3::new(3.0, 0.0, 0.0));
        let sim_state = run_closed_loop_flight(&sim_cfg, 7, 5000);
        // With 3 m/s wind + linear drag k=0.05 N·s/m, cascade-P steady
        // state has a bias. Just assert vehicle hasn't flown away.
        assert!(
            sim_state.position_ned.norm() < 5.0,
            "drift = {}",
            sim_state.position_ned
        );
        assert!((sim_state.attitude.norm() - 1.0).abs() < 1.0e-3);
    }
}
