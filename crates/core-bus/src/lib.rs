#![no_std]

//! Internal message bus and ROS 2 / Zenoh bridge.
//!
//! Topics here match the ROS 2 IDL catalog in `docs/topics.md`.

/// Topic name tokens used across the firmware.
pub mod topics {
    pub const IMU_RAW: &str = "auto_pilot/imu/raw";
    pub const ATTITUDE: &str = "auto_pilot/estimator/attitude";
    pub const VELOCITY_NED: &str = "auto_pilot/estimator/velocity_ned";
    pub const POSITION_NED: &str = "auto_pilot/estimator/position_ned";
    pub const SETPOINT_POSITION: &str = "auto_pilot/control/setpoint_position";
    pub const ACTUATOR_CMD: &str = "auto_pilot/control/actuator_cmd";
    pub const HEALTH: &str = "auto_pilot/system/health";
}
