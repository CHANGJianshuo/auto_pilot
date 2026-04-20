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

use algo_ekf::{predict_step, Covariance, ImuMeasurement, ProcessNoise, State};
use algo_indi::{
    compute_torque_increment, IndiInput, Inertia, LowPassFilterVec3, RateCommand, RateGain,
};
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
    pub inertia: Inertia,
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
}

/// Dynamic flight state held across iterations.
#[derive(Clone, Copy, Debug)]
pub struct FlightState {
    pub state: State,
    pub covariance: Covariance,
    pub last_gyro_filtered: Vector3<f32>,
}

impl Default for FlightState {
    fn default() -> Self {
        Self {
            state: State::default(),
            covariance: algo_ekf::initial_covariance(),
            last_gyro_filtered: Vector3::zeros(),
        }
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
    let (next_state, next_cov) = predict_step(
        &flight.state,
        &flight.covariance,
        measurement,
        cfg.process_noise,
        dt_s,
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
        inertia: nalgebra::Matrix3::from_diagonal(&Vector3::new(0.015, 0.015, 0.025)),
        process_noise: ProcessNoise::default(),
        gyro_lpf: LowPassFilterVec3::new(80.0, 1000.0),
        alpha_lpf: LowPassFilterVec3::new(30.0, 1000.0),
        e_inv,
        motor_min_n: 0.0,
        motor_max_n: 6.0,
        hover_thrust_n: 2.45, // m=0.25 kg × g=9.8 m/s²
    }
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
