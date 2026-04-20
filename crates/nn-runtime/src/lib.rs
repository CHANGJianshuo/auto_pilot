#![no_std]

//! Neural-network / RL policy inference, gated by a safety envelope.
//!
//! First release: residual-learning on top of NMPC (policy outputs a
//! correction to the NMPC command, not a raw command). Envelope guard
//! rejects values that would drive the vehicle outside known-safe
//! attitude/velocity/acceleration ranges.

use nalgebra::Vector3;

/// Bounds defining the "known-safe" dynamic envelope. Any NN output whose
/// effect takes the vehicle outside this envelope is rejected.
#[derive(Clone, Copy, Debug)]
pub struct SafetyEnvelope {
    pub max_tilt_rad: f32,
    pub max_rate_rad_s: Vector3<f32>,
    pub max_accel_m_s2: f32,
    pub max_velocity_m_s: f32,
}
