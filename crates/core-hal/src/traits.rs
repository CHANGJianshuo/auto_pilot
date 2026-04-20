//! Trait interfaces between hardware drivers and algorithm crates.
//!
//! Algorithm crates depend on these traits, never on silicon-specific types.
//! Driver implementations live behind feature flags (e.g. `stm32h753`).

use nalgebra::Vector3;

/// Timestamp in microseconds since boot. Monotonic; never wraps in flight.
pub type TimestampUs = u64;

/// Reading from a rate + accelerometer IMU.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub timestamp_us: TimestampUs,
    /// Body-frame angular rate, rad/s.
    pub gyro_rad_s: Vector3<f32>,
    /// Body-frame specific force, m/s^2 (includes gravity).
    pub accel_m_s2: Vector3<f32>,
    /// Sensor temperature, Celsius. Used for bias compensation.
    pub temperature_c: f32,
}

/// Source of IMU samples (typically DMA-backed ring buffer).
pub trait ImuSource {
    type Error: core::fmt::Debug;
    fn try_read(&mut self) -> Result<Option<ImuSample>, Self::Error>;
}

/// A single actuator (ESC, servo) output channel.
#[derive(Clone, Copy, Debug)]
pub struct ActuatorCommand {
    /// Normalized command in \[-1, 1\] for servos or \[0, 1\] for ESC throttle.
    pub value: f32,
}

/// Sink that drives N actuators in a single atomic update.
pub trait ActuatorSink<const N: usize> {
    type Error: core::fmt::Debug;
    fn write(&mut self, commands: &[ActuatorCommand; N]) -> Result<(), Self::Error>;
}
