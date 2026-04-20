//! Multirotor application entry point.
//!
//! M0: host-side placeholder that prints the composed task graph so CI has a
//! smoke test. M1 wires this to `embassy-stm32` on `thumbv7em-none-eabihf`.

use core_rtos::{Priority, TaskSpec};

fn main() {
    let tasks: &[TaskSpec] = &[
        TaskSpec {
            name: "imu_sample",
            priority: Priority::RateLoop,
            period_us: 1_000,
            wcet_budget_us: 150,
        },
        TaskSpec {
            name: "rate_loop_indi",
            priority: Priority::RateLoop,
            period_us: 1_000,
            wcet_budget_us: 500,
        },
        TaskSpec {
            name: "attitude_loop",
            priority: Priority::AttitudeLoop,
            period_us: 2_000,
            wcet_budget_us: 400,
        },
        TaskSpec {
            name: "ekf_update",
            priority: Priority::Estimator,
            period_us: 4_000,
            wcet_budget_us: 1_500,
        },
        TaskSpec {
            name: "nmpc_outer",
            priority: Priority::PositionLoop,
            period_us: 20_000,
            wcet_budget_us: 8_000,
        },
        TaskSpec {
            name: "navigation",
            priority: Priority::Navigation,
            period_us: 20_000,
            wcet_budget_us: 1_000,
        },
        TaskSpec {
            name: "mavlink_tx",
            priority: Priority::Telemetry,
            period_us: 100_000,
            wcet_budget_us: 2_000,
        },
    ];

    println!("auto_pilot M0 — task graph:");
    for t in tasks {
        println!(
            "  {:<16} prio={:?} period={} us wcet<={} us",
            t.name, t.priority, t.period_us, t.wcet_budget_us
        );
    }
}
