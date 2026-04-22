//! Multirotor application entry point.
//!
//! Host-side demo: runs a short rate-loop simulation against a deterministic
//! IMU trace so `cargo run -p app-copter --release` prints something useful
//! without hardware attached. M1.13 wires this to `embassy-stm32` on
//! `thumbv7em-none-eabihf`.

use app_copter::{FlightState, default_config_250g, rate_loop_step};
use core_hal::traits::ImuSample;
use core_rtos::{Priority, TaskSpec};
use nalgebra::Vector3;

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

    println!("auto_pilot — task graph:");
    for t in tasks {
        println!(
            "  {:<16} prio={:?} period={} us wcet<={} us",
            t.name, t.priority, t.period_us, t.wcet_budget_us
        );
    }

    println!();
    println!("Rate-loop demo (100 ms of stationary IMU):");

    let mut cfg = default_config_250g();
    let mut flight = FlightState::default();
    let dt = 0.001_f32;

    for i in 0..100 {
        let imu = ImuSample {
            timestamp_us: u64::try_from(i).unwrap_or(0),
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, -algo_ekf::GRAVITY_M_S2),
            temperature_c: 20.0,
        };
        let out = rate_loop_step(
            &mut cfg,
            &mut flight,
            imu,
            dt,
            algo_indi::RateCommand::default(),
        );
        if i == 0 || i == 49 || i == 99 {
            let t0 = out.motor_thrusts_n.fixed_view::<1, 1>(0, 0).to_scalar();
            let t1 = out.motor_thrusts_n.fixed_view::<1, 1>(1, 0).to_scalar();
            let t2 = out.motor_thrusts_n.fixed_view::<1, 1>(2, 0).to_scalar();
            let t3 = out.motor_thrusts_n.fixed_view::<1, 1>(3, 0).to_scalar();
            let q = flight.state.attitude;
            println!(
                "  step {i:3}: motors=[{t0:.3} {t1:.3} {t2:.3} {t3:.3}] N, q=({:.3},{:.3},{:.3},{:.3})",
                q.w, q.i, q.j, q.k
            );
        }
    }
}
