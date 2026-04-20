#![no_std]

//! Task scheduling primitives.
//!
//! The concrete scheduler (embassy-executor or Hubris) plugs in behind these
//! abstractions so algorithm code is unaware of which runtime owns it.

/// Priority levels used across the system. Lower number = higher priority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    RateLoop = 0,
    AttitudeLoop = 1,
    Estimator = 2,
    PositionLoop = 3,
    Navigation = 4,
    Telemetry = 5,
    Logging = 6,
    Background = 7,
}

/// Fixed-period task descriptor used for schedule analysis and WCET budgets.
#[derive(Clone, Copy, Debug)]
pub struct TaskSpec {
    pub name: &'static str,
    pub priority: Priority,
    /// Nominal period in microseconds.
    pub period_us: u32,
    /// Budgeted worst-case execution time in microseconds.
    pub wcet_budget_us: u32,
}
