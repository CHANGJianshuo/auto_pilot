//! MAVLink over UDP transport for host-side SITL.
//!
//! Opens a UDP socket, sends telemetry to one or more ground-control
//! stations. Pure `tokio` — not for embedded targets (those use UART /
//! radio drivers via `core-hal`).
//!
//! Typical usage from a demo binary:
//!
//! ```ignore
//! let sink = MavlinkUdpSink::bind("127.0.0.1:14551", "127.0.0.1:14550").await?;
//! sink.send_heartbeat(seq).await?;
//! sink.send_attitude(seq, t_ms, q, omega).await?;
//! ```

use algo_nmpc::Setpoint;
use comms_mavlink::{
    FrameBuffer, ParseError, encode_attitude, encode_command_ack, encode_global_position_int,
    encode_heartbeat, parse_frame,
};
use mavlink::MavHeader;
use mavlink::common::{MavCmd, MavMessage, MavResult};
use nalgebra::{Quaternion, Vector3};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use tokio::net::UdpSocket;

/// System / component IDs advertised on the bus. Defaulting to 1/1 is
/// what QGC expects for a single vehicle.
pub const DEFAULT_SYSTEM_ID: u8 = 1;
pub const DEFAULT_COMPONENT_ID: u8 = 1;

/// Extract an `algo_nmpc::Setpoint` from a MAVLink message if it is a
/// `SET_POSITION_TARGET_LOCAL_NED` — otherwise `None`.
///
/// Only position + velocity + yaw fields are honoured; the `type_mask`
/// bitmap saying "ignore these fields" is currently assumed to be
/// zeroed (QGC does this). Acceleration FF is taken from the message.
#[must_use]
pub fn setpoint_from_mav_message(msg: &MavMessage) -> Option<Setpoint> {
    if let MavMessage::SET_POSITION_TARGET_LOCAL_NED(data) = msg {
        Some(Setpoint {
            position_ned: Vector3::new(data.x, data.y, data.z),
            velocity_ned: Vector3::new(data.vx, data.vy, data.vz),
            accel_ned: Vector3::new(data.afx, data.afy, data.afz),
            yaw_rad: data.yaw,
        })
    } else {
        None
    }
}

/// Returns `Some(altitude_m)` if `msg` is a COMMAND_LONG requesting
/// automatic takeoff (`MAV_CMD_NAV_TAKEOFF`, id 22). `altitude_m` is
/// `param7` — the target altitude in metres above the home position
/// (positive = up). `None` for every other message.
///
/// The other COMMAND_LONG params (pitch, yaw, lat/lon) are ignored:
/// demo always climbs from the home xy. Production firmware with a
/// navigator would honour them.
#[must_use]
pub fn takeoff_request_from_mav_message(msg: &MavMessage) -> Option<f32> {
    if let MavMessage::COMMAND_LONG(data) = msg {
        if data.command == mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF {
            return Some(data.param7);
        }
    }
    None
}

/// Returns `true` if `msg` is a COMMAND_LONG requesting automatic
/// landing (`MAV_CMD_NAV_LAND`, id 21). False for everything else.
///
/// We ignore the lat/lon/alt parameters — callers land in place using
/// the EKF's current horizontal estimate. Production firmware should
/// honour the target location when it's non-NaN.
#[must_use]
pub fn land_request_from_mav_message(msg: &MavMessage) -> bool {
    if let MavMessage::COMMAND_LONG(data) = msg {
        return data.command == mavlink::common::MavCmd::MAV_CMD_NAV_LAND;
    }
    false
}

/// Extract an ARM/DISARM request from a MAVLink message.
///
/// MAV_CMD_COMPONENT_ARM_DISARM (id 400) is carried in COMMAND_LONG with
/// `param1 = 1.0` (arm) or `param1 = 0.0` (disarm). Returns `Some(true)`
/// to arm, `Some(false)` to disarm, `None` for anything else.
///
/// Any COMMAND_LONG with a different command id, or any other message
/// type, returns `None` — the caller should treat that as "ignore".
///
/// `target_system` / `target_component` are not checked here: demo /
/// SITL accepts all. Production firmware must verify they match the
/// local IDs before acting.
#[must_use]
pub fn arm_change_from_mav_message(msg: &MavMessage) -> Option<bool> {
    if let MavMessage::COMMAND_LONG(data) = msg {
        if data.command == mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM {
            return Some(data.param1 >= 0.5);
        }
    }
    None
}

/// One-way MAVLink sender that broadcasts telemetry to a fixed remote
/// address (the GCS). Sequence numbers auto-increment.
#[derive(Debug)]
pub struct MavlinkUdpSink {
    socket: UdpSocket,
    remote: SocketAddr,
    system_id: u8,
    component_id: u8,
    sequence: AtomicU8,
}

impl MavlinkUdpSink {
    /// Bind a local UDP socket and target `remote` for every send.
    /// `local` is a usable "bind" address; `remote` is the GCS endpoint
    /// (default QGC listen port is 14550).
    pub async fn bind(local: &str, remote: &str) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(local).await?;
        let remote: SocketAddr = remote.parse().map_err(|e: std::net::AddrParseError| {
            std::io::Error::new(std::io::ErrorKind::InvalidInput, e)
        })?;
        Ok(Self {
            socket,
            remote,
            system_id: DEFAULT_SYSTEM_ID,
            component_id: DEFAULT_COMPONENT_ID,
            sequence: AtomicU8::new(0),
        })
    }

    /// Consume and return the next sequence byte.
    fn next_seq(&self) -> u8 {
        self.sequence.fetch_add(1, Ordering::Relaxed)
    }

    async fn send_frame(&self, frame: &FrameBuffer) -> std::io::Result<()> {
        self.socket.send_to(frame.as_slice(), self.remote).await?;
        Ok(())
    }

    /// Emit a HEARTBEAT.
    pub async fn send_heartbeat(&self) -> std::io::Result<()> {
        let frame = encode_heartbeat(self.system_id, self.component_id, self.next_seq());
        self.send_frame(&frame).await
    }

    /// Emit a COMMAND_ACK in response to a prior COMMAND_LONG.
    pub async fn send_command_ack(
        &self,
        command: MavCmd,
        result: MavResult,
    ) -> std::io::Result<()> {
        let frame = encode_command_ack(
            self.system_id,
            self.component_id,
            self.next_seq(),
            command,
            result,
        );
        self.send_frame(&frame).await
    }

    /// Emit ATTITUDE from the current estimate.
    pub async fn send_attitude(
        &self,
        time_boot_ms: u32,
        attitude: Quaternion<f32>,
        body_rate_rad_s: Vector3<f32>,
    ) -> std::io::Result<()> {
        let frame = encode_attitude(
            self.system_id,
            self.component_id,
            self.next_seq(),
            time_boot_ms,
            attitude,
            body_rate_rad_s,
        );
        self.send_frame(&frame).await
    }

    /// Non-blocking receive. Returns `Ok(None)` when no datagram is
    /// waiting. Errors come from the socket layer; corrupt or
    /// incomplete frames produce `Ok(None)` (we silently drop them so
    /// the caller's poll loop keeps running).
    pub fn try_recv(&self) -> std::io::Result<Option<(MavHeader, MavMessage, SocketAddr)>> {
        let mut buf = [0u8; 512];
        match self.socket.try_recv_from(&mut buf) {
            Ok((n, src)) => {
                let slice = buf.get(..n).unwrap_or(&[]);
                match parse_frame(slice) {
                    Ok((header, msg)) => Ok(Some((header, msg, src))),
                    Err(ParseError::Incomplete | ParseError::Corrupt | ParseError::Unknown) => {
                        Ok(None)
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Emit GLOBAL_POSITION_INT from a local NED position + geodetic origin.
    #[allow(clippy::too_many_arguments)]
    pub async fn send_global_position_int(
        &self,
        time_boot_ms: u32,
        origin_lat_deg: f64,
        origin_lon_deg: f64,
        origin_alt_m: f32,
        position_ned_m: Vector3<f32>,
        velocity_ned_m_s: Vector3<f32>,
        heading_rad: f32,
    ) -> std::io::Result<()> {
        let frame = encode_global_position_int(
            self.system_id,
            self.component_id,
            self.next_seq(),
            time_boot_ms,
            origin_lat_deg,
            origin_lon_deg,
            origin_alt_m,
            position_ned_m,
            velocity_ned_m_s,
            heading_rad,
        );
        self.send_frame(&frame).await
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn sink_bind_and_send_round_trip() {
        // Receiver socket first so we know which ephemeral port the OS
        // picked, then bind the sender to target it.
        let recv = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let recv_addr = recv.local_addr().unwrap();
        let sink = MavlinkUdpSink::bind("127.0.0.1:0", &recv_addr.to_string())
            .await
            .unwrap();
        sink.send_heartbeat().await.unwrap();
        let mut buf = [0u8; 512];
        let (n, _) =
            tokio::time::timeout(std::time::Duration::from_secs(1), recv.recv_from(&mut buf))
                .await
                .unwrap()
                .unwrap();
        assert!(n >= 12, "frame too small: {n}");
        assert_eq!(buf.first().copied(), Some(0xFD), "expected v2 magic");
    }

    #[tokio::test]
    async fn try_recv_returns_none_when_empty() {
        let sink = MavlinkUdpSink::bind("127.0.0.1:0", "127.0.0.1:14550")
            .await
            .unwrap();
        let res = sink.try_recv().unwrap();
        assert!(res.is_none());
    }

    #[tokio::test]
    async fn try_recv_parses_heartbeat() {
        // `sender` sends into the sink's bound port.
        let sink = MavlinkUdpSink::bind("127.0.0.1:0", "127.0.0.1:14550")
            .await
            .unwrap();
        let sink_addr = sink.socket.local_addr().unwrap();

        let sender = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let frame = comms_mavlink::encode_heartbeat(3, 1, 7);
        sender.send_to(frame.as_slice(), sink_addr).await.unwrap();

        // Give tokio a moment to deliver.
        for _ in 0..20 {
            if let Some((header, msg, _)) = sink.try_recv().unwrap() {
                assert_eq!(header.system_id, 3);
                assert_eq!(header.sequence, 7);
                assert!(matches!(msg, MavMessage::HEARTBEAT(_)));
                return;
            }
            tokio::time::sleep(std::time::Duration::from_millis(5)).await;
        }
        panic!("did not receive the heartbeat within 100 ms");
    }

    #[test]
    fn setpoint_from_non_target_message_is_none() {
        use mavlink::common::HEARTBEAT_DATA;
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: mavlink::common::MavModeFlag::empty(),
            system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        });
        assert!(setpoint_from_mav_message(&msg).is_none());
    }

    #[test]
    fn setpoint_from_target_message_maps_fields() {
        use mavlink::common::{
            MavFrame, PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA,
        };
        let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,
            target_system: 1,
            target_component: 1,
            coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
            type_mask: PositionTargetTypemask::empty(),
            x: 1.0,
            y: 2.0,
            z: -3.0,
            vx: 0.1,
            vy: 0.2,
            vz: 0.3,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 1.2,
            yaw_rate: 0.0,
        });
        let sp = setpoint_from_mav_message(&msg).expect("setpoint extracted");
        assert_eq!(sp.position_ned, Vector3::new(1.0, 2.0, -3.0));
        assert_eq!(sp.velocity_ned, Vector3::new(0.1, 0.2, 0.3));
        assert!((sp.yaw_rad - 1.2).abs() < 1.0e-6);
    }

    #[test]
    fn arm_change_from_non_command_message_is_none() {
        use mavlink::common::HEARTBEAT_DATA;
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: mavlink::common::MavModeFlag::empty(),
            system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        });
        assert!(arm_change_from_mav_message(&msg).is_none());
    }

    #[test]
    fn arm_change_from_wrong_command_is_none() {
        use mavlink::common::COMMAND_LONG_DATA;
        let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        assert!(arm_change_from_mav_message(&msg).is_none());
    }

    #[test]
    fn arm_change_arm_and_disarm_round_trip() {
        use mavlink::common::COMMAND_LONG_DATA;
        let make = |p1: f32| {
            MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                param1: p1,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
                command: mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                target_system: 1,
                target_component: 1,
                confirmation: 0,
            })
        };
        assert_eq!(arm_change_from_mav_message(&make(1.0)), Some(true));
        assert_eq!(arm_change_from_mav_message(&make(0.0)), Some(false));
    }

    #[tokio::test]
    async fn command_ack_round_trips_to_receiver() {
        let recv = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let recv_addr = recv.local_addr().unwrap();
        let sink = MavlinkUdpSink::bind("127.0.0.1:0", &recv_addr.to_string())
            .await
            .unwrap();
        sink.send_command_ack(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            MavResult::MAV_RESULT_ACCEPTED,
        )
        .await
        .unwrap();
        let mut buf = [0u8; 512];
        let (n, _) =
            tokio::time::timeout(std::time::Duration::from_secs(1), recv.recv_from(&mut buf))
                .await
                .unwrap()
                .unwrap();
        let slice = buf.get(..n).unwrap_or(&[]);
        let (_hdr, msg) = parse_frame(slice).expect("parses");
        match msg {
            MavMessage::COMMAND_ACK(data) => {
                assert_eq!(data.command, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
                assert!(matches!(data.result, MavResult::MAV_RESULT_ACCEPTED));
            }
            _ => panic!("expected COMMAND_ACK, got {msg:?}"),
        }
    }

    #[test]
    fn takeoff_request_parses_target_altitude() {
        use mavlink::common::COMMAND_LONG_DATA;
        let takeoff = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 5.0, // 5 m target altitude
            command: mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        assert_eq!(takeoff_request_from_mav_message(&takeoff), Some(5.0));

        let land = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 5.0,
            command: mavlink::common::MavCmd::MAV_CMD_NAV_LAND,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        assert!(takeoff_request_from_mav_message(&land).is_none());
    }

    #[test]
    fn land_request_matches_only_nav_land_command_long() {
        use mavlink::common::COMMAND_LONG_DATA;
        let land = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mavlink::common::MavCmd::MAV_CMD_NAV_LAND,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        assert!(land_request_from_mav_message(&land));
        let takeoff = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        assert!(!land_request_from_mav_message(&takeoff));
    }

    #[tokio::test]
    async fn sequence_increments() {
        let recv = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let recv_addr = recv.local_addr().unwrap();
        let sink = MavlinkUdpSink::bind("127.0.0.1:0", &recv_addr.to_string())
            .await
            .unwrap();
        for _ in 0..3 {
            sink.send_heartbeat().await.unwrap();
        }
        let mut buf = [0u8; 512];
        let mut seqs = Vec::new();
        for _ in 0..3 {
            let (n, _) =
                tokio::time::timeout(std::time::Duration::from_secs(1), recv.recv_from(&mut buf))
                    .await
                    .unwrap()
                    .unwrap();
            assert!(n >= 12);
            seqs.push(buf.get(4).copied().unwrap_or(0));
        }
        assert_eq!(seqs, vec![0, 1, 2]);
    }
}
