//! Driver for the InvenSense / TDK **ICM-42688-P** 6-axis IMU.
//!
//! Pure `embedded-hal@1.0` `SpiDevice`-generic driver — any MCU whose HAL
//! exposes a [`SpiDevice`] can instantiate it. The Pixhawk 6X-class FMU
//! puts this chip on SPI4 / SPI5 (bank-dependent); the actual peripheral
//! wiring lives in the firmware binary, not here.
//!
//! # Register layout (bank 0 only — bank switching not used)
//!
//! The chip has multiple register banks; everything the flight controller
//! needs (power, ODR, FSR, data) is in bank 0, so this driver never
//! touches `REG_BANK_SEL`. If a future feature needs bank 1+ (e.g.
//! self-test), add a `with_bank(&mut self, bank: u8)` helper.
//!
//! # Default configuration
//!
//! After [`Icm42688::configure`] the chip is in:
//!   * accelerometer Low-Noise mode, ±16 g FSR, 1 kHz ODR
//!   * gyro Low-Noise mode, ±2000 dps FSR, 1 kHz ODR
//!
//! These match CLAUDE.md's 1 kHz inner rate loop. Change [`Config`] if a
//! different airframe needs different ranges — be sure to update the
//! scaling constants below to match.

#![allow(clippy::as_conversions)] // i16 → f32 via `From` below; other as-casts are bit-layout reads.

use embedded_hal::spi::SpiDevice;
use nalgebra::Vector3;

use crate::traits::{ImuSample, TimestampUs};

/// Bank 0 register addresses used by this driver. The full chip map has
/// many more — only the ones we actually touch are listed.
pub mod reg {
    /// Start of the contiguous 14-byte data block (temp + accel + gyro).
    pub const TEMP_DATA1: u8 = 0x1D;
    /// Power management: gyro and accel mode bits.
    pub const PWR_MGMT0: u8 = 0x4E;
    /// Gyro full-scale + output data rate.
    pub const GYRO_CONFIG0: u8 = 0x4F;
    /// Accel full-scale + output data rate.
    pub const ACCEL_CONFIG0: u8 = 0x50;
    /// Fixed identifier, see [`WHO_AM_I_VAL`].
    pub const WHO_AM_I: u8 = 0x75;
}

/// The value `WHO_AM_I` (0x75) returns on a real ICM-42688-P.
pub const WHO_AM_I_VAL: u8 = 0x47;

/// ±16 g range → 2048 LSB / g. Scale raw counts to m/s² with g = 9.80665.
const ACCEL_LSB_TO_MPS2: f32 = 9.806_65 / 2048.0;
/// ±2000 dps range → 16.4 LSB / (deg/s). Scale raw counts to rad/s.
const GYRO_LSB_TO_RAD_S: f32 = (core::f32::consts::PI / 180.0) / 16.4;
/// Datasheet: T_c = raw / 132.48 + 25.
const TEMP_LSB_PER_DEGC: f32 = 132.48;
const TEMP_OFFSET_C: f32 = 25.0;

/// Errors surfaced by the driver.
#[derive(Debug)]
pub enum Error<S> {
    /// Underlying SPI transaction failed.
    Spi(S),
    /// WHO_AM_I didn't match [`WHO_AM_I_VAL`]; wrong chip, bad wiring, or
    /// bus corruption. The returned byte is what the chip actually sent
    /// back so the caller can log it.
    WhoAmIMismatch { got: u8 },
}

/// Driver handle.
///
/// Owns the `SpiDevice` so ownership of chip-select and bus access is
/// unambiguous. The `embedded-hal` SpiDevice abstraction already wraps
/// CS assertion around every call, so this driver never pokes CS
/// directly.
pub struct Icm42688<SPI> {
    spi: SPI,
}

impl<SPI: core::fmt::Debug> core::fmt::Debug for Icm42688<SPI> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Icm42688").field("spi", &self.spi).finish()
    }
}

impl<SPI: SpiDevice> Icm42688<SPI> {
    /// Probe the bus, check `WHO_AM_I`, and take ownership of the SPI
    /// device. Does **not** apply [`configure`]; call that separately so
    /// the caller can override [`Config`] first.
    pub fn new(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        let mut drv = Self { spi };
        let id = drv.read_reg(reg::WHO_AM_I)?;
        if id != WHO_AM_I_VAL {
            return Err(Error::WhoAmIMismatch { got: id });
        }
        Ok(drv)
    }

    /// Bring the chip to the canonical low-noise 1 kHz / ±16 g / ±2000 dps
    /// configuration. After this the data registers update every 1 ms.
    pub fn configure(&mut self) -> Result<(), Error<SPI::Error>> {
        // Gyro + Accel both in Low-Noise mode (mode bits = 0b11).
        // Layout: [ bits 7..4 reserved | 3..2 GYRO_MODE | 1..0 ACCEL_MODE ]
        self.write_reg(reg::PWR_MGMT0, 0b0000_1111)?;
        // GYRO_CONFIG0: [ 7..5 FS_SEL (000 = ±2000 dps) | 4 reserved | 3..0 ODR (0110 = 1 kHz) ].
        self.write_reg(reg::GYRO_CONFIG0, 0b0000_0110)?;
        // ACCEL_CONFIG0: same layout, 000 = ±16 g, 0110 = 1 kHz.
        self.write_reg(reg::ACCEL_CONFIG0, 0b0000_0110)?;
        Ok(())
    }

    /// Burst-read the 14-byte data block (temp + accel + gyro), scale
    /// each axis to SI units, tag with `timestamp_us`.
    ///
    /// The chip returns big-endian 16-bit integers; we sign-extend to
    /// `i16` before scaling so negative accelerations survive.
    pub fn read_sample(
        &mut self,
        timestamp_us: TimestampUs,
    ) -> Result<ImuSample, Error<SPI::Error>> {
        let mut buf = [0u8; 15];
        buf[0] = 0x80 | reg::TEMP_DATA1;
        self.spi.transfer_in_place(&mut buf).map_err(Error::Spi)?;
        // buf[0] is the echoed command byte; payload is buf[1..=14].
        let temp_raw = be_i16(buf.get(1..=2));
        let ax = be_i16(buf.get(3..=4));
        let ay = be_i16(buf.get(5..=6));
        let az = be_i16(buf.get(7..=8));
        let gx = be_i16(buf.get(9..=10));
        let gy = be_i16(buf.get(11..=12));
        let gz = be_i16(buf.get(13..=14));
        Ok(ImuSample {
            timestamp_us,
            accel_m_s2: Vector3::new(
                f32::from(ax) * ACCEL_LSB_TO_MPS2,
                f32::from(ay) * ACCEL_LSB_TO_MPS2,
                f32::from(az) * ACCEL_LSB_TO_MPS2,
            ),
            gyro_rad_s: Vector3::new(
                f32::from(gx) * GYRO_LSB_TO_RAD_S,
                f32::from(gy) * GYRO_LSB_TO_RAD_S,
                f32::from(gz) * GYRO_LSB_TO_RAD_S,
            ),
            temperature_c: f32::from(temp_raw) / TEMP_LSB_PER_DEGC + TEMP_OFFSET_C,
        })
    }

    fn read_reg(&mut self, addr: u8) -> Result<u8, Error<SPI::Error>> {
        let mut buf = [0x80 | addr, 0];
        self.spi.transfer_in_place(&mut buf).map_err(Error::Spi)?;
        Ok(buf.get(1).copied().unwrap_or(0))
    }

    fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        // MSB clear = write.
        let buf = [addr & 0x7F, value];
        self.spi.write(&buf).map_err(Error::Spi)
    }
}

fn be_i16(bytes: Option<&[u8]>) -> i16 {
    match bytes {
        Some(s) if s.len() == 2 => i16::from_be_bytes([
            s.first().copied().unwrap_or(0),
            s.get(1).copied().unwrap_or(0),
        ]),
        _ => 0,
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::expect_used,
    clippy::indexing_slicing
)]
mod tests {
    use super::*;
    use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

    /// Minimal in-memory SpiDevice mock: a register file the chip would
    /// expose over SPI, with the driver's read/write framing modelled.
    #[derive(Debug)]
    struct MockSpi {
        regs: [u8; 256],
    }
    impl MockSpi {
        fn new() -> Self {
            let mut regs = [0u8; 256];
            regs[usize::from(reg::WHO_AM_I)] = WHO_AM_I_VAL;
            Self { regs }
        }
        fn set_block(&mut self, start: u8, bytes: &[u8]) {
            for (i, b) in bytes.iter().enumerate() {
                self.regs[usize::from(start) + i] = *b;
            }
        }
    }
    #[derive(Debug)]
    struct MockError;
    impl embedded_hal::spi::Error for MockError {
        fn kind(&self) -> embedded_hal::spi::ErrorKind {
            embedded_hal::spi::ErrorKind::Other
        }
    }
    impl ErrorType for MockSpi {
        type Error = MockError;
    }
    impl SpiDevice for MockSpi {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match op {
                    Operation::TransferInPlace(buf) => {
                        let first = buf.first().copied().unwrap_or(0);
                        let reading = first & 0x80 != 0;
                        let addr = usize::from(first & 0x7F);
                        if reading {
                            // Fill buf[1..] from the register file.
                            let n = buf.len();
                            for i in 1..n {
                                buf[i] = self.regs.get(addr + i - 1).copied().unwrap_or(0);
                            }
                        } else {
                            // Writes via transfer_in_place are unusual; treat as no-op echo.
                        }
                    }
                    Operation::Write(buf) => {
                        let first = buf.first().copied().unwrap_or(0);
                        let addr = usize::from(first & 0x7F);
                        for (i, b) in buf.iter().skip(1).enumerate() {
                            self.regs[addr + i] = *b;
                        }
                    }
                    _ => {
                        // Other ops not exercised by this driver.
                    }
                }
            }
            Ok(())
        }
    }

    #[test]
    fn new_rejects_wrong_who_am_i() {
        let mut spi = MockSpi::new();
        spi.regs[usize::from(reg::WHO_AM_I)] = 0xAA;
        let err = Icm42688::new(spi).expect_err("must reject");
        match err {
            Error::WhoAmIMismatch { got } => assert_eq!(got, 0xAA),
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn new_accepts_correct_who_am_i() {
        let spi = MockSpi::new();
        assert!(Icm42688::new(spi).is_ok());
    }

    #[test]
    fn configure_writes_expected_bytes() {
        let spi = MockSpi::new();
        let mut drv = Icm42688::new(spi).unwrap();
        drv.configure().unwrap();
        // Expose the register file for assertions via a tiny helper:
        // read back via the same driver path.
        assert_eq!(drv.read_reg(reg::PWR_MGMT0).unwrap(), 0b0000_1111);
        assert_eq!(drv.read_reg(reg::GYRO_CONFIG0).unwrap(), 0b0000_0110);
        assert_eq!(drv.read_reg(reg::ACCEL_CONFIG0).unwrap(), 0b0000_0110);
    }

    #[test]
    fn read_sample_scales_accel_one_g() {
        let mut spi = MockSpi::new();
        // ±16 g full-scale, 2048 LSB/g → 1 g = 2048 counts → big-endian 0x0800.
        // Block layout: TEMP(2), AX(2), AY(2), AZ(2), GX(2), GY(2), GZ(2).
        let mut block = [0u8; 14];
        // temp 0
        // ax = 0
        // ay = 0
        // az = 2048
        block[6] = 0x08;
        block[7] = 0x00;
        spi.set_block(reg::TEMP_DATA1, &block);
        let mut drv = Icm42688::new(spi).unwrap();
        let s = drv.read_sample(12_345).unwrap();
        assert_eq!(s.timestamp_us, 12_345);
        assert!((s.accel_m_s2.z - 9.806_65).abs() < 1.0e-3);
        assert!(s.accel_m_s2.x.abs() < 1.0e-3 && s.accel_m_s2.y.abs() < 1.0e-3);
        assert!(s.gyro_rad_s.norm() < 1.0e-3);
    }

    #[test]
    fn read_sample_scales_gyro_hundred_dps() {
        let mut spi = MockSpi::new();
        let mut block = [0u8; 14];
        // gyro X = 100 dps * 16.4 LSB/dps = 1640 counts = 0x0668.
        block[8] = 0x06;
        block[9] = 0x68;
        spi.set_block(reg::TEMP_DATA1, &block);
        let mut drv = Icm42688::new(spi).unwrap();
        let s = drv.read_sample(0).unwrap();
        let expected = 100.0_f32 * core::f32::consts::PI / 180.0;
        assert!((s.gyro_rad_s.x - expected).abs() < 1.0e-3);
    }

    #[test]
    fn read_sample_scales_negative_accel() {
        let mut spi = MockSpi::new();
        let mut block = [0u8; 14];
        // ax = -1 g = -2048 counts → i16 two's complement big-endian.
        let raw: i16 = -2048;
        let be = raw.to_be_bytes();
        block[2] = be[0];
        block[3] = be[1];
        spi.set_block(reg::TEMP_DATA1, &block);
        let mut drv = Icm42688::new(spi).unwrap();
        let s = drv.read_sample(0).unwrap();
        assert!((s.accel_m_s2.x + 9.806_65).abs() < 1.0e-3);
    }

    #[test]
    fn read_sample_decodes_temperature() {
        let mut spi = MockSpi::new();
        let mut block = [0u8; 14];
        // temp raw = 132.48 * 20 ≈ 2650 → rounded to i16.
        let raw: i16 = 2650;
        let be = raw.to_be_bytes();
        block[0] = be[0];
        block[1] = be[1];
        spi.set_block(reg::TEMP_DATA1, &block);
        let mut drv = Icm42688::new(spi).unwrap();
        let s = drv.read_sample(0).unwrap();
        // expected ≈ 25 + 2650/132.48 ≈ 45.0 °C
        assert!((s.temperature_c - 45.0).abs() < 0.1);
    }
}
