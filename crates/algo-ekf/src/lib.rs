#![no_std]

//! 24-state Extended Kalman Filter skeleton.
//!
//! State vector layout (to be filled in during M1):
//!
//! ```text
//! [ q (4), v_ned (3), p_ned (3), gyro_bias (3), accel_bias (3),
//!   mag_ned (3), mag_bias (3), wind_ne (2) ]
//! ```
//!
//! Design invariants (to be proven via `formal/` harnesses):
//! - Covariance P is symmetric positive-semidefinite at every step.
//! - Quaternion stays unit-normalized within 1e-6 after each predict/update.
//! - No integer overflow, no panic paths.

use nalgebra::{Quaternion, Vector2, Vector3};

/// Dimension of the EKF state vector.
pub const STATE_DIM: usize = 24;

/// Full state, accessed as a struct so consumers don't couple to raw indices.
#[derive(Clone, Copy, Debug)]
pub struct State {
    /// Body-to-NED rotation as unit quaternion.
    pub attitude: Quaternion<f32>,
    /// Velocity in NED frame, m/s.
    pub velocity_ned: Vector3<f32>,
    /// Position in NED frame, m (relative to origin).
    pub position_ned: Vector3<f32>,
    /// Gyro zero-bias, rad/s.
    pub gyro_bias: Vector3<f32>,
    /// Accelerometer zero-bias, m/s^2.
    pub accel_bias: Vector3<f32>,
    /// Earth magnetic field vector in NED, gauss.
    pub mag_ned: Vector3<f32>,
    /// Magnetometer hard-iron offset, gauss.
    pub mag_bias: Vector3<f32>,
    /// Horizontal wind, NE components, m/s.
    pub wind_ne: Vector2<f32>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            attitude: Quaternion::identity(),
            velocity_ned: Vector3::zeros(),
            position_ned: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            mag_ned: Vector3::new(0.21, 0.0, 0.43),
            mag_bias: Vector3::zeros(),
            wind_ne: Vector2::zeros(),
        }
    }
}
