//! IMU drivers.
//!
//! Each sub-module is a pure `embedded-hal` driver that stays agnostic of
//! the underlying MCU. A Pixhawk 6X-class FMU wires three of them up
//! (ICM-42688-P primary, ICM-45686 secondary, BMI088 tertiary) and votes
//! across them in the FDIR layer — that voting lives in `algo-fdir`, not
//! here.

pub mod icm42688;
