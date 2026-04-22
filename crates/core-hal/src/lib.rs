#![no_std]

//! Hardware abstraction layer.
//!
//! Owns the peripherals. Exposes project-specific traits that the algorithm
//! crates consume without knowing the underlying silicon.

pub mod imu;
pub mod imu_buffer;
pub mod traits;
