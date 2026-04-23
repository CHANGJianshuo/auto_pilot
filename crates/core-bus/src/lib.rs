#![no_std]

//! Internal message bus and ROS 2 / Zenoh bridge.
//!
//! Two layers:
//!
//! 1. **Topic names** ([`topics`]) — canonical string keys that match the
//!    ROS 2 topic catalog in `docs/topics.md`. Used as Zenoh key
//!    expressions so the same subscribers work onboard or on a
//!    ROS 2 companion computer.
//!
//! 2. **Typed messages** (this crate's top-level types) — the payload
//!    for each topic. Every message derives `serde::{Serialize,
//!    Deserialize}` so [`postcard`] can encode it into a `heapless::Vec`
//!    for transport; no heap, no alloc. Host-side Zenoh integration in
//!    `sim-hil` uses the same types — one serde schema spans firmware
//!    and ground station.
//!
//! The message definitions are intentionally **plain data** (no
//! `nalgebra` types, no lifetimes, no enums-with-data). That keeps the
//! schema stable across changes in `core-hal` / `algo-*` and lets the
//! same payload traverse UART / CAN / Zenoh / ROS 2 without a codec
//! translation layer per transport.

use heapless::Vec;
use serde::{Deserialize, Serialize};

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

/// Maximum encoded payload size across every message in this module.
/// Fits the largest (ActuatorCmdMsg with 16 channels + timestamp + flags
/// = ~72 B) with comfortable headroom. Transport buffers size to this.
pub const MAX_PAYLOAD_LEN: usize = 256;

/// Concrete byte buffer type for on-wire payloads.
pub type Payload = Vec<u8, MAX_PAYLOAD_LEN>;

/// Encoding / decoding errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecError {
    /// Output buffer is too small for the encoded payload.
    Overflow,
    /// Deserialisation failed (truncated or corrupt payload).
    Decode,
}

impl From<postcard::Error> for CodecError {
    fn from(err: postcard::Error) -> Self {
        use postcard::Error;
        match err {
            Error::SerializeBufferFull => Self::Overflow,
            _ => Self::Decode,
        }
    }
}

/// Encode any serde-serialisable message into a fixed-capacity
/// `heapless::Vec<u8>` using postcard. Safe in `no_std`; returns a
/// typed error on overflow / corrupt payload.
pub fn encode<M: Serialize>(msg: &M) -> Result<Payload, CodecError> {
    let mut out = Payload::new();
    // Grow to capacity so `to_slice` has room to write.
    out.resize(MAX_PAYLOAD_LEN, 0)
        .map_err(|_| CodecError::Overflow)?;
    let used = postcard::to_slice(msg, out.as_mut_slice())?.len();
    out.truncate(used);
    Ok(out)
}

/// Decode a serde-deserialisable message from a byte slice.
pub fn decode<'a, M: Deserialize<'a>>(bytes: &'a [u8]) -> Result<M, CodecError> {
    postcard::from_bytes(bytes).map_err(Into::into)
}

/// Monotonic microsecond timestamp. Every message carries this so
/// subscribers can correlate across topics.
pub type TimestampUs = u64;

// --- Topic messages ---------------------------------------------------------

/// `auto_pilot/imu/raw` — one raw IMU sample.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuMsg {
    pub timestamp_us: TimestampUs,
    pub gyro_rad_s: [f32; 3],
    pub accel_m_s2: [f32; 3],
    pub temperature_c: f32,
}

/// `auto_pilot/estimator/attitude` — current attitude estimate.
///
/// `quaternion` is Hamilton-convention (w, x, y, z) and normalised.
/// `cov_diag` is the EKF's diagonal variance on (roll, pitch, yaw)
/// — full 3×3 covariance doesn't ride the bus every tick; subscribers
/// that need it fetch from a `cov` topic (future work).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AttitudeMsg {
    pub timestamp_us: TimestampUs,
    pub quaternion: [f32; 4],
    pub cov_diag: [f32; 3],
}

/// `auto_pilot/estimator/velocity_ned` — body velocity in NED.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VelocityNedMsg {
    pub timestamp_us: TimestampUs,
    pub velocity_m_s: [f32; 3],
    pub cov_diag: [f32; 3],
}

/// `auto_pilot/estimator/position_ned` — body position in NED (m).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PositionNedMsg {
    pub timestamp_us: TimestampUs,
    pub position_m: [f32; 3],
    pub cov_diag: [f32; 3],
}

/// `auto_pilot/control/setpoint_position` — guidance setpoint in NED.
///
/// Mirrors `algo_nmpc::Setpoint`. `yaw_rad` uses aerospace (3-2-1)
/// convention so it's directly consumable by attitude control.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SetpointPositionMsg {
    pub timestamp_us: TimestampUs,
    pub position_ned_m: [f32; 3],
    pub velocity_ned_m_s: [f32; 3],
    pub accel_ned_m_s2: [f32; 3],
    pub yaw_rad: f32,
}

/// Actuator channel count — fits our 4-motor X-quad plus headroom for
/// a future hex / octo. Fixed at schema time so the on-wire payload
/// size is bounded.
pub const ACTUATOR_MAX_CHANNELS: usize = 8;

/// `auto_pilot/control/actuator_cmd` — per-motor command.
///
/// `channels[i]` is Newtons of commanded thrust for motor `i`. `n`
/// counts how many channels are populated (≤ `ACTUATOR_MAX_CHANNELS`)
/// — subscribers read `&channels[..n]`. Keeping a fixed-size array
/// (vs a Vec) preserves `Copy` + predictable wire size.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActuatorCmdMsg {
    pub timestamp_us: TimestampUs,
    pub channels_n: [f32; ACTUATOR_MAX_CHANNELS],
    pub n: u8,
}

/// Vehicle health classes. Mirrors `algo_fdir::HealthLevel` but lives
/// here so subscribers don't have to depend on the FDIR crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum HealthLevel {
    Healthy = 0,
    Degraded = 1,
    Emergency = 2,
    Failed = 3,
}

/// `auto_pilot/system/health` — overall vehicle health + per-sensor
/// breakdown. Flags are bit-packed so new sensors don't bloat the
/// schema: bit 0 GPS fault, bit 1 mag, bit 2 baro, etc.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HealthMsg {
    pub timestamp_us: TimestampUs,
    pub overall: HealthLevel,
    /// Per-sensor health, same ordering as `SensorFaultBit` below.
    pub sensors: [HealthLevel; 4],
    /// Bitmap of active fault flags, defined in [`SensorFaultBit`].
    pub fault_flags: u32,
}

/// Bit positions for [`HealthMsg::fault_flags`].
#[derive(Debug)]
pub struct SensorFaultBit;

impl SensorFaultBit {
    pub const GPS: u32 = 1 << 0;
    pub const MAG: u32 = 1 << 1;
    pub const BARO: u32 = 1 << 2;
    pub const IMU: u32 = 1 << 3;
    pub const MOTOR: u32 = 1 << 4;
    pub const BATTERY: u32 = 1 << 5;
    pub const EKF: u32 = 1 << 6;
}

// ----------------------------------------------------------------------------
// Formal verification — Kani harnesses
// ----------------------------------------------------------------------------
//
// Scope: integer / bit-level invariants over the message schema. CBMC
// handles these in sub-second time because there are no floats to
// reason about symbolically. Proofs that touch payload bytes (serde
// round-trip) stay in the property-tests module — they're not Kani-
// tractable at this problem size.
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// Generate an arbitrary bounded `HealthLevel`. Kani enumerates all
    /// 4 variants symbolically.
    fn any_health_level() -> HealthLevel {
        let s: u8 = kani::any();
        kani::assume(s < 4);
        match s {
            0 => HealthLevel::Healthy,
            1 => HealthLevel::Degraded,
            2 => HealthLevel::Emergency,
            _ => HealthLevel::Failed,
        }
    }

    /// `HealthLevel` severity-by-u8-cast ordering matches the declared
    /// severity order. Catches a future variant reorder that would let
    /// `Failed` silently land between `Healthy` and `Degraded`.
    #[kani::proof]
    fn health_level_u8_cast_preserves_severity_order() {
        assert!((HealthLevel::Healthy as u8) < (HealthLevel::Degraded as u8));
        assert!((HealthLevel::Degraded as u8) < (HealthLevel::Emergency as u8));
        assert!((HealthLevel::Emergency as u8) < (HealthLevel::Failed as u8));
    }

    /// Round-trip: `HealthLevel` → u8 → HealthLevel recovers the same
    /// variant for every one.
    #[kani::proof]
    fn health_level_u8_round_trip() {
        let h = any_health_level();
        let encoded = h as u8;
        let recovered = match encoded {
            0 => HealthLevel::Healthy,
            1 => HealthLevel::Degraded,
            2 => HealthLevel::Emergency,
            3 => HealthLevel::Failed,
            _ => panic!("out of range"),
        };
        assert_eq!(h, recovered);
    }

    /// Each `SensorFaultBit` constant has exactly one bit set and the
    /// constants are pairwise disjoint. Catches a typo like
    /// `MOTOR: u32 = 1 << 3` that would silently alias with `IMU`.
    #[kani::proof]
    fn sensor_fault_bits_are_distinct_powers_of_two() {
        let bits = [
            SensorFaultBit::GPS,
            SensorFaultBit::MAG,
            SensorFaultBit::BARO,
            SensorFaultBit::IMU,
            SensorFaultBit::MOTOR,
            SensorFaultBit::BATTERY,
            SensorFaultBit::EKF,
        ];
        // Each has exactly one bit set.
        for b in bits {
            assert!(b != 0);
            assert_eq!(b & (b - 1), 0, "not a power of two");
        }
        // Pairwise disjoint.
        for i in 0..bits.len() {
            for j in (i + 1)..bits.len() {
                let bi = bits[i];
                let bj = bits[j];
                assert_eq!(bi & bj, 0, "bits collide");
            }
        }
    }

    /// Combining bits via `|` preserves each constituent — `BITS_A |
    /// BITS_B` contains both masks. The inverse (AND-check for
    /// specific flags) is how [`HealthMsg::fault_flags`] is consumed,
    /// so this proof locks the contract in.
    #[kani::proof]
    fn sensor_fault_bits_union_preserves_each_member() {
        let combined = SensorFaultBit::GPS | SensorFaultBit::MAG | SensorFaultBit::EKF;
        assert_ne!(combined & SensorFaultBit::GPS, 0);
        assert_ne!(combined & SensorFaultBit::MAG, 0);
        assert_ne!(combined & SensorFaultBit::EKF, 0);
        // Bits not in the union stay clear.
        assert_eq!(combined & SensorFaultBit::BARO, 0);
        assert_eq!(combined & SensorFaultBit::IMU, 0);
    }

    /// `ACTUATOR_MAX_CHANNELS` fits in u8 — `ActuatorCmdMsg::n` is u8
    /// and callers cast `ACTUATOR_MAX_CHANNELS as u8` via
    /// `u8::try_from`. Verify the hard-coded value never exceeds u8's
    /// range, catching a future bump past 256.
    #[kani::proof]
    fn actuator_max_channels_fits_u8() {
        assert!(ACTUATOR_MAX_CHANNELS <= (u8::MAX as usize));
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn imu_msg_round_trip_preserves_every_field() {
        let orig = ImuMsg {
            timestamp_us: 123_456,
            gyro_rad_s: [0.1, -0.2, 0.05],
            accel_m_s2: [0.0, 0.0, -9.806_65],
            temperature_c: 22.5,
        };
        let buf = encode(&orig).unwrap();
        let back: ImuMsg = decode(&buf).unwrap();
        assert_eq!(orig, back);
    }

    #[test]
    fn attitude_msg_round_trip() {
        let orig = AttitudeMsg {
            timestamp_us: 1_000,
            quaternion: [1.0, 0.0, 0.0, 0.0],
            cov_diag: [0.01, 0.01, 0.02],
        };
        let buf = encode(&orig).unwrap();
        let back: AttitudeMsg = decode(&buf).unwrap();
        assert_eq!(orig, back);
    }

    #[test]
    fn setpoint_msg_round_trip_preserves_yaw() {
        let orig = SetpointPositionMsg {
            timestamp_us: 42,
            position_ned_m: [0.0, 0.0, -1.0],
            velocity_ned_m_s: [0.0, 0.0, 0.0],
            accel_ned_m_s2: [0.0, 0.0, 0.0],
            yaw_rad: core::f32::consts::FRAC_PI_4,
        };
        let buf = encode(&orig).unwrap();
        let back: SetpointPositionMsg = decode(&buf).unwrap();
        assert_eq!(orig, back);
    }

    #[test]
    fn actuator_cmd_round_trip_preserves_channel_count() {
        let mut channels = [0.0_f32; ACTUATOR_MAX_CHANNELS];
        channels[0] = 0.6;
        channels[1] = 0.7;
        channels[2] = 0.5;
        channels[3] = 0.8;
        let orig = ActuatorCmdMsg {
            timestamp_us: 7,
            channels_n: channels,
            n: 4,
        };
        let buf = encode(&orig).unwrap();
        let back: ActuatorCmdMsg = decode(&buf).unwrap();
        assert_eq!(orig, back);
        let populated = back
            .channels_n
            .get(..usize::from(back.n))
            .expect("n within array bounds");
        assert_eq!(populated, &[0.6, 0.7, 0.5, 0.8]);
    }

    #[test]
    fn health_msg_round_trip() {
        let orig = HealthMsg {
            timestamp_us: 99,
            overall: HealthLevel::Degraded,
            sensors: [
                HealthLevel::Healthy,
                HealthLevel::Degraded,
                HealthLevel::Healthy,
                HealthLevel::Healthy,
            ],
            fault_flags: SensorFaultBit::MAG | SensorFaultBit::BATTERY,
        };
        let buf = encode(&orig).unwrap();
        let back: HealthMsg = decode(&buf).unwrap();
        assert_eq!(orig, back);
        assert_eq!(back.overall, HealthLevel::Degraded);
        assert!(back.fault_flags & SensorFaultBit::MAG != 0);
        assert!(back.fault_flags & SensorFaultBit::BATTERY != 0);
        assert_eq!(back.fault_flags & SensorFaultBit::GPS, 0);
    }

    #[test]
    fn decode_of_corrupt_payload_returns_error() {
        // Empty buffer can never decode any message with required fields.
        let r: Result<ImuMsg, CodecError> = decode(&[]);
        assert_eq!(r, Err(CodecError::Decode));
    }

    #[test]
    fn encoded_payload_fits_max_len() {
        // The largest message (actuator cmd) stays well inside the
        // MAX_PAYLOAD_LEN budget. If someone adds a bigger variant,
        // this test flags it so the transport buffers can be resized
        // in one place.
        let biggest = ActuatorCmdMsg {
            timestamp_us: u64::MAX,
            channels_n: [f32::MAX; ACTUATOR_MAX_CHANNELS],
            n: u8::try_from(ACTUATOR_MAX_CHANNELS).expect("channel count fits u8"),
        };
        let buf = encode(&biggest).unwrap();
        assert!(
            buf.len() <= MAX_PAYLOAD_LEN,
            "largest message {} > MAX_PAYLOAD_LEN {}",
            buf.len(),
            MAX_PAYLOAD_LEN
        );
        // Sanity: ≥ 40 B, else the field layout is wrong.
        assert!(buf.len() >= 40, "suspiciously small {}", buf.len());
    }

    #[test]
    fn topic_names_are_ros2_canonical() {
        // Firmware topic strings must use `/` separators and
        // `auto_pilot/` prefix so ROS 2 subscribers can resolve them.
        for t in [
            topics::IMU_RAW,
            topics::ATTITUDE,
            topics::VELOCITY_NED,
            topics::POSITION_NED,
            topics::SETPOINT_POSITION,
            topics::ACTUATOR_CMD,
            topics::HEALTH,
        ] {
            assert!(t.starts_with("auto_pilot/"), "{t} missing prefix");
            assert!(t.contains('/'), "{t} must be hierarchical");
            assert!(!t.contains(' '), "{t} must not contain spaces");
        }
    }
}
