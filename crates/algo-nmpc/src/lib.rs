#![no_std]

//! NMPC outer loop (position / velocity) skeleton.
//!
//! Targets 20–50 Hz update with a 10–20 step horizon. QP solver is
//! pluggable (Clarabel.rs host-side, custom MCU-side), selected via
//! cargo feature flags.

use nalgebra::Vector3;

/// Desired position / velocity reference fed from navigation.
#[derive(Clone, Copy, Debug, Default)]
pub struct Setpoint {
    pub position_ned: Vector3<f32>,
    pub velocity_ned: Vector3<f32>,
    pub yaw_rad: f32,
}

/// Outputs of the outer loop consumed by the attitude / rate controller.
#[derive(Clone, Copy, Debug, Default)]
pub struct OuterLoopCommand {
    pub acceleration_ned: Vector3<f32>,
    pub yaw_rate_rad_s: f32,
}
