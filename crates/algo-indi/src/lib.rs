#![no_std]

//! Incremental Nonlinear Dynamic Inversion (INDI) inner loop.
//!
//! INDI computes actuator increments from the desired angular-acceleration
//! increment and the measured angular acceleration, using the control
//! effectiveness matrix `G`. Compared to PID, INDI is far less dependent on
//! an accurate plant model and handles actuator failures gracefully.

use nalgebra::{Matrix3, Vector3};

/// Per-axis angular-rate command produced by the attitude loop.
#[derive(Clone, Copy, Debug, Default)]
pub struct RateCommand {
    pub body_rate_rad_s: Vector3<f32>,
}

/// Inputs required for one INDI update.
#[derive(Clone, Copy, Debug)]
pub struct IndiStep<'a> {
    pub cmd: RateCommand,
    pub measured_rate_rad_s: Vector3<f32>,
    pub measured_accel_rad_s2: Vector3<f32>,
    /// Control effectiveness, mapping actuator increments to body-frame torque.
    pub effectiveness: &'a Matrix3<f32>,
    /// Gain on the rate error; tunable per airframe.
    pub k_rate: Vector3<f32>,
}

/// Angular acceleration increment requested from the allocator.
#[derive(Clone, Copy, Debug)]
pub struct TorqueIncrement {
    pub body_torque_inc: Vector3<f32>,
}

/// Compute the body-frame torque increment required to close the rate error.
///
/// The control-allocation layer is responsible for turning this into physical
/// actuator commands.
#[must_use]
pub fn compute_torque_increment(step: &IndiStep) -> TorqueIncrement {
    let rate_err = step.cmd.body_rate_rad_s - step.measured_rate_rad_s;
    let desired_accel = step.k_rate.component_mul(&rate_err);
    let accel_inc = desired_accel - step.measured_accel_rad_s2;
    TorqueIncrement { body_torque_inc: step.effectiveness * accel_inc }
}
