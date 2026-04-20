//! Host-side simulation / HITL harness.
//!
//! This crate runs on the developer machine (not on the MCU) and wires up
//! the algorithm crates against a simulator (Gazebo, AerialGym) or a real
//! FMU over serial / UDP. It is the only crate in the workspace that uses
//! `tokio`.

#![allow(clippy::missing_errors_doc)]

pub fn placeholder() {
    tracing::info!("sim-hil placeholder — wire up in M1");
}
