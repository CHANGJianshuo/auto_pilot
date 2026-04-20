#![no_std]

//! Fault Detection, Isolation, and Recovery.

/// Top-level vehicle health classes. Transitions are unidirectional within
/// a flight (Healthy -> Degraded -> Emergency -> Failed), except explicit
/// recoveries on the ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HealthLevel {
    Healthy,
    Degraded,
    Emergency,
    Failed,
}
