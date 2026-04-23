#![no_std]

//! MAVLink 2 adapter.
//!
//! This crate provides **stateless encoders** from the EKF-estimated vehicle
//! state into MAVLink messages (HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT)
//! and a matching `parse_frame` that extracts incoming messages from a
//! byte buffer. Transport (serial, UDP) lives in the application crate.
//!
//! We deliberately take raw numeric types rather than `algo_ekf::State` so
//! `comms-mavlink` does not depend on the filter crate.

use mavlink::common::{
    ATTITUDE_DATA, COMMAND_ACK_DATA, GLOBAL_POSITION_INT_DATA, HEARTBEAT_DATA, MavCmd, MavMessage,
    MavResult, STATUSTEXT_DATA,
};
pub use mavlink::common::MavSeverity;
use mavlink::{MAVLinkV2MessageRaw, MavHeader};
use nalgebra::{Quaternion, Vector3};

/// A fixed-size buffer big enough for the largest MAVLink 2 payload we
/// currently emit (GLOBAL_POSITION_INT is 28 B + 12 B framing = 40 B).
/// Keep power-of-two with headroom so users of the API can compose
/// several messages.
pub const MAX_FRAME_LEN: usize = 280;
pub type FrameBuffer = heapless::Vec<u8, MAX_FRAME_LEN>;

/// Pack a full MAVLink 2 frame for `msg` into `FrameBuffer`. System /
/// component IDs match QGroundControl's expectations.
#[must_use]
pub fn encode(header: &MavHeader, msg: &MavMessage) -> FrameBuffer {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(*header, msg);
    let mut out = FrameBuffer::new();
    let _ = out.extend_from_slice(raw.raw_bytes());
    out
}

/// Build a `HEARTBEAT` frame. The autopilot reports itself as a
/// `MAV_TYPE_QUADROTOR` with our in-development autopilot kind.
#[must_use]
pub fn encode_heartbeat(system_id: u8, component_id: u8, sequence: u8) -> FrameBuffer {
    use mavlink::common::{MavAutopilot, MavModeFlag, MavState, MavType};
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    });
    let header = MavHeader {
        system_id,
        component_id,
        sequence,
    };
    encode(&header, &msg)
}

/// Extract roll/pitch/yaw (radians) from a unit quaternion using the
/// standard aerospace (3-2-1) convention.
#[must_use]
pub fn quaternion_to_euler(q: Quaternion<f32>) -> (f32, f32, f32) {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    // Roll (x-axis)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);
    // Pitch (y-axis, handle gimbal lock)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        libm::copysignf(core::f32::consts::FRAC_PI_2, sinp)
    } else {
        libm::asinf(sinp)
    };
    // Yaw (z-axis)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);
    (roll, pitch, yaw)
}

/// Build a `COMMAND_ACK` frame acknowledging a prior `COMMAND_LONG`.
///
/// Per MAVLink spec, the responder must ACK every `COMMAND_LONG` with
/// the echoed `command` id and a [`MavResult`] outcome. QGroundControl's
/// UI buttons (Arm, Land) spin until ACK arrives; without this, users
/// see stuck spinners even when the command was accepted.
#[must_use]
pub fn encode_command_ack(
    system_id: u8,
    component_id: u8,
    sequence: u8,
    command: MavCmd,
    result: MavResult,
) -> FrameBuffer {
    let msg = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result });
    let header = MavHeader {
        system_id,
        component_id,
        sequence,
    };
    encode(&header, &msg)
}

/// Build a `STATUSTEXT` frame.
///
/// MAVLink STATUSTEXT is a 50-byte UTF-8 payload with an attached
/// severity (EMERGENCY → DEBUG). QGroundControl surfaces these as
/// toast notifications in the HUD — the canonical channel for
/// out-of-band pilot alerts such as "MOTOR 0 FAILED".
///
/// `text` is copied byte-for-byte into the 50-byte buffer, null-
/// terminated. Input longer than 50 bytes is silently truncated;
/// input shorter is null-padded. No multi-chunk fragmentation is
/// performed — a single 50-byte message covers every alert the
/// current firmware emits.
#[must_use]
pub fn encode_statustext(
    system_id: u8,
    component_id: u8,
    sequence: u8,
    severity: MavSeverity,
    text: &str,
) -> FrameBuffer {
    let mut buf = [0_u8; 50];
    // Copy up to 49 bytes; leave index 49 as the implicit null terminator.
    let src = text.as_bytes();
    let n = if src.len() > 49 { 49 } else { src.len() };
    for (dst, &byte) in buf.iter_mut().take(n).zip(src.iter()) {
        *dst = byte;
    }
    let msg = MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity,
        text: buf,
    });
    let header = MavHeader {
        system_id,
        component_id,
        sequence,
    };
    encode(&header, &msg)
}

/// Build an `ATTITUDE` frame from a unit quaternion + angular rate.
///
/// `time_boot_ms` is the timestamp in milliseconds since boot (the
/// caller typically sets this from a monotonic clock).
#[must_use]
pub fn encode_attitude(
    system_id: u8,
    component_id: u8,
    sequence: u8,
    time_boot_ms: u32,
    attitude: Quaternion<f32>,
    body_rate_rad_s: Vector3<f32>,
) -> FrameBuffer {
    let (roll, pitch, yaw) = quaternion_to_euler(attitude);
    let msg = MavMessage::ATTITUDE(ATTITUDE_DATA {
        time_boot_ms,
        roll,
        pitch,
        yaw,
        rollspeed: body_rate_rad_s.x,
        pitchspeed: body_rate_rad_s.y,
        yawspeed: body_rate_rad_s.z,
    });
    let header = MavHeader {
        system_id,
        component_id,
        sequence,
    };
    encode(&header, &msg)
}

/// Scale factor to convert degrees to the 1e-7-degrees integer that
/// `GLOBAL_POSITION_INT` uses.
const DEG_TO_E7: f64 = 1.0e7;

/// Build a `GLOBAL_POSITION_INT` frame from a local NED position + the
/// geodetic origin. Lat/lon are reported as 1e-7-degrees integers,
/// altitude as millimetres.
///
/// The NED → lat/lon conversion uses a flat-earth approximation (good
/// to ~1 m at ≤ 10 km ranges). For long-range flights swap in a WGS-84
/// Vincenty solver later.
#[allow(clippy::too_many_arguments)]
#[must_use]
pub fn encode_global_position_int(
    system_id: u8,
    component_id: u8,
    sequence: u8,
    time_boot_ms: u32,
    origin_lat_deg: f64,
    origin_lon_deg: f64,
    origin_alt_m: f32,
    position_ned_m: Vector3<f32>,
    velocity_ned_m_s: Vector3<f32>,
    heading_rad: f32,
) -> FrameBuffer {
    // Flat-earth small-angle delta to latitude / longitude.
    const EARTH_RADIUS_M: f64 = 6_378_137.0;
    let delta_lat_rad = f64::from(position_ned_m.x) / EARTH_RADIUS_M;
    let delta_lon_rad =
        f64::from(position_ned_m.y) / (EARTH_RADIUS_M * libm::cos(origin_lat_deg.to_radians()));
    let lat_deg = origin_lat_deg + delta_lat_rad.to_degrees();
    let lon_deg = origin_lon_deg + delta_lon_rad.to_degrees();

    // NED convention: +z is down → altitude_m = origin_alt − z.
    let altitude_m = origin_alt_m - position_ned_m.z;

    let lat_i32 = clamp_f64_to_i32(lat_deg * DEG_TO_E7);
    let lon_i32 = clamp_f64_to_i32(lon_deg * DEG_TO_E7);
    let alt_mm = clamp_f32_to_i32(altitude_m * 1000.0);
    // MAV uses cm/s for velocity.
    let vx_cms = clamp_f32_to_i16(velocity_ned_m_s.x * 100.0);
    let vy_cms = clamp_f32_to_i16(velocity_ned_m_s.y * 100.0);
    let vz_cms = clamp_f32_to_i16(velocity_ned_m_s.z * 100.0);

    // `hdg` is centidegrees, wrapped to 0..=35999. 65535 = unknown.
    let heading_deg = heading_rad.to_degrees();
    let wrapped = libm::fmodf(heading_deg + 360.0, 360.0);
    let hdg_cdeg = {
        let v = libm::roundf(wrapped * 100.0);
        if v.is_finite() && (0.0..=35_999.0).contains(&v) {
            #[allow(clippy::as_conversions)]
            let u = v as u16;
            u
        } else {
            65_535
        }
    };

    let msg = MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
        time_boot_ms,
        lat: lat_i32,
        lon: lon_i32,
        alt: alt_mm,
        relative_alt: clamp_f32_to_i32((altitude_m - origin_alt_m) * 1000.0),
        vx: vx_cms,
        vy: vy_cms,
        vz: vz_cms,
        hdg: hdg_cdeg,
    });
    let header = MavHeader {
        system_id,
        component_id,
        sequence,
    };
    encode(&header, &msg)
}

/// A parse-side error wrapping the upstream crate's reason.
#[derive(Debug)]
pub enum ParseError {
    /// Not enough bytes in the buffer to read a complete frame.
    Incomplete,
    /// CRC mismatch or framing bytes wrong.
    Corrupt,
    /// Message ID not in the `common` dialect or payload malformed.
    Unknown,
}

/// Parse one MAVLink 2 frame from the front of `bytes`.
///
/// Returns `Some((header, msg))` on success; `None` if the buffer is
/// too short or the frame is malformed. Does not return how many bytes
/// were consumed — callers that stream should use `parse_frame_at` once
/// it lands (next step).
///
/// Intended entry point for UDP / UART receivers: after `recv_from`,
/// pass the datagram payload straight in.
///
/// `no_std`-safe: `&[u8]` already implements the `embedded_io::Read`
/// trait the mavlink crate needs, so there is no std / alloc fallout.
pub fn parse_frame(bytes: &[u8]) -> Result<(MavHeader, MavMessage), ParseError> {
    use mavlink::peek_reader::PeekReader;
    // `&[u8]` implements mavlink's embedded_io::Read, no cursor needed.
    let mut reader = PeekReader::<&[u8], 280>::new(bytes);
    match mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
        Ok((header, msg)) => Ok((header, msg)),
        Err(mavlink::error::MessageReadError::Io) => Err(ParseError::Incomplete),
        Err(mavlink::error::MessageReadError::Parse(_)) => Err(ParseError::Corrupt),
    }
}

// --- Internal conversions ---------------------------------------------------

fn clamp_f64_to_i32(v: f64) -> i32 {
    if !v.is_finite() {
        return 0;
    }
    let clamped = v.clamp(f64::from(i32::MIN), f64::from(i32::MAX));
    #[allow(clippy::as_conversions)]
    let out = clamped as i32;
    out
}

fn clamp_f32_to_i32(v: f32) -> i32 {
    if !v.is_finite() {
        return 0;
    }
    #[allow(clippy::as_conversions)]
    let clamped = v.clamp(i32::MIN as f32, i32::MAX as f32);
    #[allow(clippy::as_conversions)]
    let out = clamped as i32;
    out
}

fn clamp_f32_to_i16(v: f32) -> i16 {
    if !v.is_finite() {
        return 0;
    }
    #[allow(clippy::as_conversions)]
    let clamped = v.clamp(f32::from(i16::MIN), f32::from(i16::MAX));
    #[allow(clippy::as_conversions)]
    let out = clamped as i16;
    out
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

    fn byte_at(frame: &FrameBuffer, idx: usize) -> u8 {
        *frame.get(idx).unwrap_or(&0)
    }

    #[test]
    fn heartbeat_round_trips() {
        let frame = encode_heartbeat(7, 1, 3);
        // MAVLink 2 frame layout:
        //   idx 0  STX (0xFD)
        //   idx 1  payload length
        //   idx 2  incompat flags
        //   idx 3  compat flags
        //   idx 4  sequence
        //   idx 5  system ID
        //   idx 6  component ID
        assert!(frame.len() >= 12, "frame too small: {}", frame.len());
        assert_eq!(byte_at(&frame, 0), 0xFD, "expected MAVLink v2 magic");
        assert_eq!(byte_at(&frame, 4), 3, "sequence");
        assert_eq!(byte_at(&frame, 5), 7, "sys_id");
        assert_eq!(byte_at(&frame, 6), 1, "comp_id");
    }

    #[test]
    fn attitude_encodes_expected_euler() {
        // 90° rotation about z: quaternion (cos 45°, 0, 0, sin 45°).
        let half = core::f32::consts::FRAC_PI_4;
        let q = Quaternion::new(libm::cosf(half), 0.0, 0.0, libm::sinf(half));
        let (_r, _p, yaw) = quaternion_to_euler(q);
        assert!((yaw - core::f32::consts::FRAC_PI_2).abs() < 1.0e-4);

        let frame = encode_attitude(1, 1, 0, 12345, q, Vector3::new(0.1, -0.2, 0.3));
        assert_eq!(byte_at(&frame, 0), 0xFD);
        assert!(frame.len() > 20);
    }

    #[test]
    fn global_position_int_wraps_heading() {
        let pos = Vector3::new(100.0, 0.0, -5.0); // 100 m north, 5 m up
        let vel = Vector3::new(1.0, 0.0, 0.0);

        let frame = encode_global_position_int(1, 1, 0, 1000, 40.0, -105.0, 1655.0, pos, vel, 0.0);
        assert_eq!(byte_at(&frame, 0), 0xFD);
        // MAVLink 2 truncates trailing zeros, so payload length is variable.
        // Just require the frame fits the 28-byte upper bound plus overhead.
        assert!(frame.len() <= 12 + 28);
    }

    #[test]
    fn global_position_int_handles_negative_heading() {
        let frame = encode_global_position_int(
            1,
            1,
            0,
            0,
            0.0,
            0.0,
            0.0,
            Vector3::zeros(),
            Vector3::zeros(),
            -1.0, // -1 rad heading
        );
        assert_eq!(byte_at(&frame, 0), 0xFD);
    }

    #[test]
    fn clamp_helpers_reject_non_finite() {
        assert_eq!(clamp_f64_to_i32(f64::NAN), 0);
        assert_eq!(clamp_f32_to_i32(f32::INFINITY), 0);
        assert_eq!(clamp_f32_to_i16(f32::NEG_INFINITY), 0);
    }

    #[test]
    fn heartbeat_round_trip_through_parser() {
        let encoded = encode_heartbeat(9, 2, 17);
        let (header, msg) = parse_frame(encoded.as_slice()).expect("parse succeeds");
        assert_eq!(header.system_id, 9);
        assert_eq!(header.component_id, 2);
        assert_eq!(header.sequence, 17);
        match msg {
            MavMessage::HEARTBEAT(_) => (),
            other => panic!("expected HEARTBEAT, got {other:?}"),
        }
    }

    #[test]
    fn attitude_round_trip_preserves_euler() {
        let half = core::f32::consts::FRAC_PI_4;
        let q = Quaternion::new(libm::cosf(half), 0.0, 0.0, libm::sinf(half));
        let omega = Vector3::new(0.1, -0.2, 0.05);
        let encoded = encode_attitude(1, 1, 0, 42, q, omega);
        let (_h, msg) = parse_frame(encoded.as_slice()).expect("parse succeeds");
        match msg {
            MavMessage::ATTITUDE(data) => {
                assert!((data.yaw - core::f32::consts::FRAC_PI_2).abs() < 1.0e-4);
                assert!((data.rollspeed - 0.1).abs() < 1.0e-6);
                assert!((data.pitchspeed - (-0.2)).abs() < 1.0e-6);
                assert!((data.yawspeed - 0.05).abs() < 1.0e-6);
                assert_eq!(data.time_boot_ms, 42);
            }
            other => panic!("expected ATTITUDE, got {other:?}"),
        }
    }

    #[test]
    fn parse_rejects_short_buffer() {
        let short = [0xFD, 0x00];
        assert!(matches!(parse_frame(&short), Err(ParseError::Incomplete)));
    }

    #[test]
    fn parse_rejects_corrupted_frame() {
        // Emit a heartbeat, flip a CRC byte — should be rejected.
        let mut frame = encode_heartbeat(1, 1, 0);
        let idx = frame.len() - 1;
        if let Some(byte) = frame.get_mut(idx) {
            *byte ^= 0xFF;
        }
        let res = parse_frame(frame.as_slice());
        assert!(res.is_err(), "expected error on CRC flip, got {res:?}");
    }

    #[test]
    fn statustext_round_trips_short_message() {
        let frame = encode_statustext(1, 1, 0, MavSeverity::MAV_SEVERITY_CRITICAL, "MOTOR 0 FAILED");
        assert_eq!(byte_at(&frame, 0), 0xFD);
        // Parse it back and confirm the message reaches the caller.
        let (_hdr, msg) = parse_frame(frame.as_slice()).expect("parse failed");
        if let MavMessage::STATUSTEXT(s) = msg {
            assert_eq!(s.severity, MavSeverity::MAV_SEVERITY_CRITICAL);
            // 14 chars + 36 null bytes; text field is fixed [u8; 50].
            let prefix: &[u8] = b"MOTOR 0 FAILED";
            assert_eq!(&s.text[..prefix.len()], prefix);
            for &b in &s.text[prefix.len()..] {
                assert_eq!(b, 0, "expected null-padded tail, got {b:#x}");
            }
        } else {
            panic!("parsed wrong message variant");
        }
    }

    #[test]
    fn statustext_truncates_overlong_text() {
        // 60-byte input — only 49 bytes should fit (last slot is the
        // null terminator).
        let long = "X".repeat(60);
        let frame = encode_statustext(1, 1, 0, MavSeverity::MAV_SEVERITY_INFO, &long);
        let (_hdr, msg) = parse_frame(frame.as_slice()).expect("parse failed");
        if let MavMessage::STATUSTEXT(s) = msg {
            for &b in &s.text[..49] {
                assert_eq!(b, b'X');
            }
            assert_eq!(s.text[49], 0, "byte 49 should be the null terminator");
        } else {
            panic!("parsed wrong message variant");
        }
    }
}
