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

use comms_mavlink::{
    encode_attitude, encode_global_position_int, encode_heartbeat, FrameBuffer,
};
use nalgebra::{Quaternion, Vector3};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use tokio::net::UdpSocket;

/// System / component IDs advertised on the bus. Defaulting to 1/1 is
/// what QGC expects for a single vehicle.
pub const DEFAULT_SYSTEM_ID: u8 = 1;
pub const DEFAULT_COMPONENT_ID: u8 = 1;

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
        let remote: SocketAddr = remote
            .parse()
            .map_err(|e: std::net::AddrParseError| {
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
        let (n, _) = tokio::time::timeout(
            std::time::Duration::from_secs(1),
            recv.recv_from(&mut buf),
        )
        .await
        .unwrap()
        .unwrap();
        assert!(n >= 12, "frame too small: {n}");
        assert_eq!(buf.first().copied(), Some(0xFD), "expected v2 magic");
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
            let (n, _) = tokio::time::timeout(
                std::time::Duration::from_secs(1),
                recv.recv_from(&mut buf),
            )
            .await
            .unwrap()
            .unwrap();
            assert!(n >= 12);
            seqs.push(buf.get(4).copied().unwrap_or(0));
        }
        assert_eq!(seqs, vec![0, 1, 2]);
    }
}
