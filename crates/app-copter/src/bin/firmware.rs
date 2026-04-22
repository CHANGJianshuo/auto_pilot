//! Firmware entry point for Pixhawk 6X-class boards (STM32H753).
//!
//! This is a **link + runtime check**, not a complete firmware:
//!   * embassy-stm32 HAL initialises the chip (PLL, TIM2 as time driver).
//!   * embassy-executor runs the async main.
//!   * A 1 kHz Ticker drives `rate_loop_step` against a zero-g IMU stub.
//!   * A 1 Hz Ticker wakes up as a liveness beacon (future `defmt::info!`
//!     target).
//!
//! Real peripheral drivers (SPI → IMU, UART → MAVLink, DShot PWM → ESC)
//! arrive in M7.3.
//!
//! Build: `cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf`

#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded {
    use algo_indi::RateCommand;
    use app_copter::{FlightState, RateLoopConfig, default_config_250g, rate_loop_step};
    use core_hal::traits::ImuSample;
    use embassy_executor::Spawner;
    use embassy_time::{Duration, Ticker};
    use nalgebra::Vector3;
    use panic_halt as _;

    /// 1 kHz rate loop. Runs `rate_loop_step` against a zero-g-body-z
    /// stub IMU so the whole control stack is exercised every tick.
    #[embassy_executor::task]
    async fn rate_loop_task() {
        let mut ticker = Ticker::every(Duration::from_hz(1000));
        let mut cfg: RateLoopConfig = default_config_250g();
        let mut flight = FlightState::default();
        let dt = 0.001_f32;
        let imu = ImuSample {
            timestamp_us: 0,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, -algo_ekf::GRAVITY_M_S2),
            temperature_c: 20.0,
        };
        let rate_cmd = RateCommand::default();
        loop {
            let _ = rate_loop_step(&mut cfg, &mut flight, imu, dt, rate_cmd);
            ticker.next().await;
        }
    }

    /// 1 Hz heartbeat. Placeholder for a future `defmt::info!` / GPIO
    /// toggle — currently does nothing visible but proves a second task
    /// can co-schedule alongside the rate loop without starving it.
    #[embassy_executor::task]
    async fn heartbeat_task() {
        let mut ticker = Ticker::every(Duration::from_hz(1));
        loop {
            ticker.next().await;
        }
    }

    #[embassy_executor::main]
    async fn main(spawner: Spawner) {
        let _p = embassy_stm32::init(embassy_stm32::Config::default());
        spawner.must_spawn(rate_loop_task());
        spawner.must_spawn(heartbeat_task());
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    // Host check-only stub. The real firmware entry lives behind the
    // cortex-m-rt cfg above. Running this binary on the host is not
    // meaningful.
    eprintln!("firmware binary is embedded-only; build with --target thumbv7em-none-eabihf");
}
