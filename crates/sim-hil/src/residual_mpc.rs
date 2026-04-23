//! Residual-policy wrapping of `Mpc3dPositionController`.
//!
//! Bundles the M9.2 MPC controller with the M11a `ResidualPolicy`:
//!
//! ```text
//!        setpoint ─┐
//!        pos, vel  │      ┌───────┐
//!                  ├──────│  MPC  │──── accel_mpc ──┐
//!                  │      └───────┘                  │
//!                  │      ┌───────────┐   residual  ▼
//!                  └──────│ NN policy │──────► accel_mpc + residual
//!                         └───────────┘             │
//!                                                   ▼
//!                                        accel_to_attitude_thrust
//! ```
//!
//! Rejection policy: when the safety envelope declines the NN output,
//! the harness silently falls back to the bare MPC accel (and bumps
//! the policy's internal reject counter — caller logs via telemetry).
//! This guarantees the vehicle never flies the NN's opinion of a
//! dangerous correction, and the classical controller stays in charge.

use algo_nmpc::{
    AttitudeAndThrust, Mpc3dPositionController, PositionGains, Setpoint, accel_to_attitude_thrust,
};
use nalgebra::{Quaternion, Vector3};
use nn_runtime::{FeatureVector, InferenceBackend, PolicyError, ResidualPolicy};

/// MPC-with-residual-policy three-axis position controller.
///
/// Generic over:
///   * `H` — MPC horizon (matches inner `Mpc3dPositionController`)
///   * `B` — inference backend (affine, tract, candle, …)
///
/// Holds the MPC + policy together so the caller keeps a single
/// mutable handle per flight.
#[derive(Debug)]
pub struct MpcResidualController<const H: usize, B: InferenceBackend> {
    mpc: Mpc3dPositionController<H>,
    policy: ResidualPolicy<B>,
    max_accel: f32,
}

impl<const H: usize, B: InferenceBackend> MpcResidualController<H, B> {
    pub fn new(mpc: Mpc3dPositionController<H>, policy: ResidualPolicy<B>) -> Self {
        let max_accel = mpc.max_accel;
        Self {
            mpc,
            policy,
            max_accel,
        }
    }

    /// One tick. `current_attitude` feeds into the feature vector as
    /// roll/pitch/yaw — the extra input versus the bare MPC path is
    /// exactly why the caller uses this wrapper.
    pub fn step(
        &mut self,
        setpoint: &Setpoint,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        current_attitude: Quaternion<f32>,
        mass_kg: f32,
    ) -> AttitudeAndThrust {
        // 1. MPC pre-saturated accel command.
        let accel_mpc = self
            .mpc
            .solve_accel(setpoint, current_position, current_velocity);

        // 2. Feature extraction. Error convention matches MPC: e = x - x_ref.
        let pos_err = current_position - setpoint.position_ned;
        let rpy = quat_to_rpy(current_attitude);
        let features = FeatureVector::from_state(pos_err, current_velocity, rpy);

        // 3. Query policy. Envelope check already wired in.
        let residual = match self.policy.predict(&features, current_velocity) {
            Ok(r) => r.0,
            Err(PolicyError::Inference(_) | PolicyError::Envelope(_)) => {
                // Silent fallback: residual = 0 leaves MPC untouched.
                // Reject counter is bumped inside policy.predict.
                Vector3::zeros()
            }
        };

        // 4. Combine and clamp again (residual could push us over the
        //    envelope). Policy envelope already rejected gross outputs
        //    but this is cheap defence in depth.
        let mut accel_cmd = accel_mpc + residual;
        let mag = accel_cmd.norm();
        if mag.is_finite() && mag > self.max_accel {
            accel_cmd *= self.max_accel / mag;
        }

        // 5. Last-mile force balance.
        let dummy_gains = PositionGains {
            k_pos: Vector3::zeros(),
            k_vel: Vector3::zeros(),
            k_i_vel: Vector3::zeros(),
            max_accel: self.max_accel,
            max_integrator: 0.0,
        };
        accel_to_attitude_thrust(accel_cmd, setpoint.yaw_rad, mass_kg, &dummy_gains)
    }

    #[must_use]
    pub fn reject_count(&self) -> u32 {
        self.policy.reject_count()
    }

    pub fn reset(&mut self) {
        self.mpc.reset_warm_start();
        self.policy.reset_reject_count();
    }
}

/// Hamilton-convention quaternion → (roll, pitch, yaw) in rad, 3-2-1
/// Tait-Bryan. Same math as comms_mavlink::quaternion_to_euler but
/// replicated here so this host-only module doesn't depend on the
/// comms crate.
fn quat_to_rpy(q: Quaternion<f32>) -> Vector3<f32> {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        libm::copysignf(core::f32::consts::FRAC_PI_2, sinp)
    } else {
        libm::asinf(sinp)
    };
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);
    Vector3::new(roll, pitch, yaw)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use algo_nmpc::{LqrWeights, Mpc1dConfig};
    use nn_runtime::{AffineBackend, SafetyEnvelope};

    const H: usize = 10;

    fn make_mpc() -> Mpc3dPositionController<H> {
        let cfg = Mpc1dConfig {
            weights: LqrWeights::default(),
            dt_s: 0.001,
            u_min: -20.0,
            u_max: 20.0,
        };
        Mpc3dPositionController::new(cfg, cfg, 25, 8.0).unwrap()
    }

    #[test]
    fn residual_zero_backend_matches_bare_mpc_output() {
        // With a zero-output backend, the wrapped controller should
        // produce the exact same (q, thrust) as the bare MPC for any
        // input — residual = 0 means the last-mile math sees an
        // identical accel_cmd.
        let mut bare = make_mpc();
        let mut wrapped = MpcResidualController::new(
            make_mpc(),
            ResidualPolicy::new(
                AffineBackend::zero(),
                SafetyEnvelope::small_multirotor_default(),
            ),
        );
        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };
        let pos = Vector3::new(0.3, -0.2, -0.5);
        let vel = Vector3::new(0.1, 0.0, 0.0);
        let attitude = Quaternion::identity();
        let mass = 0.25_f32;

        let out_bare = bare.step(&setpoint, pos, vel, mass);
        let out_wrapped = wrapped.step(&setpoint, pos, vel, attitude, mass);

        assert!(
            (out_bare.thrust_n - out_wrapped.thrust_n).abs() < 1.0e-5,
            "thrust diff {} vs {}",
            out_bare.thrust_n,
            out_wrapped.thrust_n
        );
        // Quaternion components.
        assert!((out_bare.q_desired.w - out_wrapped.q_desired.w).abs() < 1.0e-5);
        assert!((out_bare.q_desired.i - out_wrapped.q_desired.i).abs() < 1.0e-5);
        assert!((out_bare.q_desired.j - out_wrapped.q_desired.j).abs() < 1.0e-5);
        assert!((out_bare.q_desired.k - out_wrapped.q_desired.k).abs() < 1.0e-5);
        assert_eq!(wrapped.reject_count(), 0);
    }

    #[test]
    fn residual_rejection_falls_back_to_bare_mpc_silently() {
        // Backend that tries to command 100 m/s² residual accel on
        // every tick. Envelope (max 8 m/s²) should reject → fallback
        // to bare-MPC output. reject_count should increment.
        let mut w = [[0.0_f32; nn_runtime::FEATURE_LEN]; nn_runtime::RESIDUAL_LEN];
        // Any non-zero feature → saturate the x channel.
        for row in &mut w {
            row[0] = 100.0;
        }
        let aggressive = AffineBackend::new(w, [0.0; nn_runtime::RESIDUAL_LEN], 0.0);

        let mut bare = make_mpc();
        let mut wrapped = MpcResidualController::new(
            make_mpc(),
            ResidualPolicy::new(aggressive, SafetyEnvelope::small_multirotor_default()),
        );
        let setpoint = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };
        let pos = Vector3::new(1.0, 0.0, -0.5); // non-zero err drives the backend
        let vel = Vector3::zeros();
        let attitude = Quaternion::identity();
        let mass = 0.25_f32;

        let out_bare = bare.step(&setpoint, pos, vel, mass);
        let out_wrapped = wrapped.step(&setpoint, pos, vel, attitude, mass);

        // Envelope should have rejected the oversized residual, so the
        // wrapped output equals the bare output.
        assert!((out_bare.thrust_n - out_wrapped.thrust_n).abs() < 1.0e-5);
        assert_eq!(wrapped.reject_count(), 1);
    }
}
