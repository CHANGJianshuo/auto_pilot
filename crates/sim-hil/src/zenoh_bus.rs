//! Host-side Zenoh session wrapper.
//!
//! Binds the [`core_bus`] typed messages to a live [Zenoh] router so
//! the same payload bytes move between firmware and ground tools
//! without a per-transport codec. Topics come straight from
//! `core_bus::topics` — no renaming layer.
//!
//! The whole module is gated behind the `zenoh-host` feature in
//! `sim-hil/Cargo.toml`. Zenoh pulls a large transitive dep graph; CI
//! builds that don't exercise the bus skip it.
//!
//! # Wire contract
//!
//! Every payload is the output of `core_bus::encode(msg)` — a
//! postcard-serialised `heapless::Vec<u8, 256>`. Because the same
//! encode/decode pair is used at both ends, there is **one** schema
//! fork: if a message changes in core-bus, both sides recompile
//! together.
//!
//! # Lifecycle
//!
//! ```ignore
//! let session = ZenohSession::open_peer().await?;
//! let pub_ = session.publisher::<SetpointPositionMsg>(topics::SETPOINT_POSITION).await?;
//! pub_.put(&SetpointPositionMsg { .. }).await?;
//!
//! let mut sub = session.subscriber::<SetpointPositionMsg>(topics::SETPOINT_POSITION).await?;
//! let msg = sub.recv().await?;
//! ```

use core_bus::{CodecError, decode, encode};
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use std::sync::Arc;
use zenoh::Session;
use zenoh::bytes::ZBytes;
use zenoh::pubsub::{Publisher as ZPublisher, Subscriber as ZSubscriber};

/// Errors surfaced by the bus wrapper.
#[derive(Debug, thiserror::Error)]
pub enum ZenohBusError {
    /// Underlying Zenoh session / router failure.
    #[error("zenoh: {0}")]
    Zenoh(String),
    /// Payload didn't round-trip through `core_bus` codec.
    #[error("codec: {0:?}")]
    Codec(CodecError),
    /// Subscriber stream ended.
    #[error("subscriber closed")]
    SubscriberClosed,
}

impl From<zenoh::Error> for ZenohBusError {
    fn from(e: zenoh::Error) -> Self {
        Self::Zenoh(format!("{e:?}"))
    }
}

impl From<CodecError> for ZenohBusError {
    fn from(e: CodecError) -> Self {
        Self::Codec(e)
    }
}

/// Thin wrapper around a Zenoh session. Cheap to clone (Arc inside).
#[derive(Clone, Debug)]
pub struct ZenohSession {
    inner: Arc<Session>,
}

impl ZenohSession {
    /// Open a peer-mode Zenoh session using the library's default
    /// config. Peers auto-discover each other on loopback / LAN
    /// without needing an explicit router process — fine for the
    /// SITL tests and single-machine ground stations. Production
    /// deployments can pass their own [`zenoh::Config`] via
    /// [`open_with_config`].
    pub async fn open_peer() -> Result<Self, ZenohBusError> {
        let config = zenoh::Config::default();
        Self::open_with_config(config).await
    }

    /// Open a session with caller-supplied config.
    pub async fn open_with_config(config: zenoh::Config) -> Result<Self, ZenohBusError> {
        let session = zenoh::open(config).await?;
        Ok(Self {
            inner: Arc::new(session),
        })
    }

    /// Bind a typed publisher to `topic`. The type parameter `M` is
    /// purely a compile-time guard so `put` / `recv` in the same
    /// module can't disagree about payload shape.
    pub async fn publisher<M: Serialize>(
        &self,
        topic: &str,
    ) -> Result<Publisher<M>, ZenohBusError> {
        let raw = self
            .inner
            .declare_publisher(topic.to_owned())
            .await
            .map_err(|e| ZenohBusError::Zenoh(format!("{e:?}")))?;
        Ok(Publisher {
            raw,
            _phantom: PhantomData,
        })
    }

    /// Bind a typed subscriber to `topic`. Messages are delivered via
    /// [`Subscriber::recv`].
    pub async fn subscriber<M: for<'de> Deserialize<'de>>(
        &self,
        topic: &str,
    ) -> Result<Subscriber<M>, ZenohBusError> {
        let raw = self
            .inner
            .declare_subscriber(topic.to_owned())
            .await
            .map_err(|e| ZenohBusError::Zenoh(format!("{e:?}")))?;
        Ok(Subscriber {
            raw,
            _phantom: PhantomData,
        })
    }

    /// Close the session. Dropping the handle also closes it; this
    /// method exists so the caller can surface close errors explicitly.
    pub async fn close(self) -> Result<(), ZenohBusError> {
        // Try to unwrap the Arc; if other clones remain, give up and
        // let Drop handle it. This matches Zenoh's own lifecycle
        // expectations — close is a best-effort explicit shutdown.
        if let Ok(s) = Arc::try_unwrap(self.inner) {
            s.close()
                .await
                .map_err(|e| ZenohBusError::Zenoh(format!("{e:?}")))?;
        }
        Ok(())
    }
}

/// Typed publisher bound to a `core-bus` message type.
pub struct Publisher<M> {
    raw: ZPublisher<'static>,
    _phantom: PhantomData<M>,
}

impl<M: Serialize> Publisher<M> {
    /// Encode and publish `msg`. Blocks until Zenoh accepts the frame.
    pub async fn put(&self, msg: &M) -> Result<(), ZenohBusError> {
        let payload = encode(msg)?;
        self.raw
            .put(ZBytes::from(payload.as_slice().to_vec()))
            .await
            .map_err(|e| ZenohBusError::Zenoh(format!("{e:?}")))?;
        Ok(())
    }
}

impl<M> std::fmt::Debug for Publisher<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Publisher")
            .field("topic", &self.raw.key_expr().as_str())
            .finish()
    }
}

/// Typed subscriber bound to a `core-bus` message type.
pub struct Subscriber<M> {
    raw: ZSubscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
    _phantom: PhantomData<M>,
}

impl<M: for<'de> Deserialize<'de>> Subscriber<M> {
    /// Await the next message on this topic, decoding it from postcard.
    pub async fn recv(&mut self) -> Result<M, ZenohBusError> {
        let sample = self
            .raw
            .recv_async()
            .await
            .map_err(|_| ZenohBusError::SubscriberClosed)?;
        let bytes = sample.payload().to_bytes();
        let msg: M = decode(bytes.as_ref())?;
        Ok(msg)
    }
}

impl<M> std::fmt::Debug for Subscriber<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Subscriber")
            .field("topic", &self.raw.key_expr().as_str())
            .finish()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use core_bus::{SetpointPositionMsg, topics};
    use std::time::Duration;

    /// Spin up two Zenoh sessions on the same tokio runtime, publish a
    /// setpoint from one, subscribe from the other, check round-trip.
    ///
    /// Zenoh peer-mode auto-discovers sessions on loopback, so no
    /// router process is required. Test creates sessions with a short
    /// config, publishes once, then tears down.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn setpoint_round_trips_through_two_peers() {
        // Publisher session first so a subscriber can observe it.
        let pub_session = ZenohSession::open_peer().await.expect("pub session");
        let sub_session = ZenohSession::open_peer().await.expect("sub session");
        // Allow peer discovery to settle.
        tokio::time::sleep(Duration::from_millis(150)).await;

        let publisher = pub_session
            .publisher::<SetpointPositionMsg>(topics::SETPOINT_POSITION)
            .await
            .unwrap();
        let mut subscriber = sub_session
            .subscriber::<SetpointPositionMsg>(topics::SETPOINT_POSITION)
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        let sent = SetpointPositionMsg {
            timestamp_us: 42_000,
            position_ned_m: [1.0, -0.5, -2.0],
            velocity_ned_m_s: [0.0, 0.0, 0.0],
            accel_ned_m_s2: [0.0, 0.0, 0.0],
            yaw_rad: core::f32::consts::FRAC_PI_2,
        };
        publisher.put(&sent).await.unwrap();

        let received = tokio::time::timeout(Duration::from_secs(2), subscriber.recv())
            .await
            .expect("no message within 2 s")
            .unwrap();
        assert_eq!(sent, received);
    }
}
