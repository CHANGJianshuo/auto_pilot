//! Firmware entry point for Pixhawk 6X-class boards (STM32H753).
//!
//! This is a **link check**, not a working firmware. It sets up the
//! `cortex-m-rt` reset handler, runs a single `rate_loop_step` against a
//! zero-filled IMU sample, then spins forever. Real peripheral drivers
//! (SPI → IMU, UART → MAVLink, DShot → ESC) arrive in M7.2.
//!
//! Build: `cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf`
//! Link: relies on `memory.x` in this crate and `link.x` from `cortex-m-rt`.

#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// Host-side builds (`cargo check -p app-copter --bin firmware`) still need
// a `main` so the bin target compiles. Gate the real entry behind the
// bare-metal cfg; provide a stub otherwise.
#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded {
    use algo_indi::RateCommand;
    use app_copter::{FlightState, default_config_250g, rate_loop_step};
    use core_hal::traits::ImuSample;
    use cortex_m_rt::entry;
    use nalgebra::Vector3;
    use panic_halt as _;

    #[entry]
    fn main() -> ! {
        let mut cfg = default_config_250g();
        let mut flight = FlightState::default();
        let dt = 0.001_f32;
        let imu = ImuSample {
            timestamp_us: 0,
            gyro_rad_s: Vector3::zeros(),
            // g is sensed as -z in the body frame at rest.
            accel_m_s2: Vector3::new(0.0, 0.0, -algo_ekf::GRAVITY_M_S2),
            temperature_c: 20.0,
        };
        let rate_cmd = RateCommand::default();
        loop {
            let _ = rate_loop_step(&mut cfg, &mut flight, imu, dt, rate_cmd);
            // M7.2 will replace this spin with a timer-driven sleep.
            cortex_m::asm::nop();
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    // Host check-only stub. The real firmware entry lives behind the
    // `cortex-m-rt` cfg above. Running this binary on the host is not
    // meaningful.
    eprintln!("firmware binary is embedded-only; build with --target thumbv7em-none-eabihf");
}
