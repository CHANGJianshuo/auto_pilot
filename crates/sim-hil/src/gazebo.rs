//! Stub for Gazebo Harmonic integration (M3.2+).
//!
//! Planned interface: a `GazeboBridge` that exposes the same step / sense
//! API as [`super::step`] and [`super::sense_imu`] but pulls state from
//! a running Gazebo server via gz-transport protobuf messages. That
//! swap-in will let the same `closed_loop_sitl_hovers_with_app_copter`
//! test run against the real simulator.
