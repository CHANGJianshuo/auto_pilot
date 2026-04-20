#![no_std]

//! Control allocation — from virtual commands (torque + thrust) to actuator
//! commands, subject to actuator limits.
//!
//! M2 ships pseudo-inverse for the nominal case, then adds a QP solver for
//! saturation handling and motor-failure reallocation.

use nalgebra::Vector3;

#[derive(Clone, Copy, Debug, Default)]
pub struct VirtualCommand {
    pub torque: Vector3<f32>,
    pub thrust_n: f32,
}
