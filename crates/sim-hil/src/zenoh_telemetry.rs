//! Full-telemetry Zenoh broadcaster, tying `core_bus` messages to the
//! closed-loop SITL.
//!
//! A single `TelemetryPublisher` owns one Zenoh publisher per
//! `core_bus::topics::*` string and exposes typed `publish_*` methods
//! that the SITL main loop calls at each message's documented cadence
//! (`docs/topics.md`). The wire format is always `core_bus::encode(msg)`
//! so a future ROS 2 / Python subscriber consuming the same bytes
//! needs only the `core-bus` schema — no MAVLink codec, no ad-hoc
//! JSON bridge.
//!
//! Gated behind the `zenoh-host` feature in `sim-hil/Cargo.toml` for
//! the same reason as [`crate::zenoh_bus`] — zenoh pulls ~80
//! transitive crates.

use core_bus::{
    ActuatorCmdMsg, AttitudeMsg, HealthLevel, HealthMsg, ImuMsg, PositionNedMsg,
    SetpointPositionMsg, VelocityNedMsg, topics,
};

use crate::zenoh_bus::{Publisher, ZenohBusError, ZenohSession};

/// Owns one Zenoh publisher per firmware telemetry topic. Construct
/// once per flight, clone cheaply (Arc inside `ZenohSession`), drop
/// when the flight ends.
pub struct TelemetryPublisher {
    imu: Publisher<ImuMsg>,
    attitude: Publisher<AttitudeMsg>,
    velocity: Publisher<VelocityNedMsg>,
    position: Publisher<PositionNedMsg>,
    setpoint: Publisher<SetpointPositionMsg>,
    actuator: Publisher<ActuatorCmdMsg>,
    health: Publisher<HealthMsg>,
    // Keep a session handle so the Zenoh router stays alive for the
    // whole lifetime of the publishers (otherwise Drop on the local
    // handle would tear the session down and invalidate them).
    _session: ZenohSession,
}

impl core::fmt::Debug for TelemetryPublisher {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TelemetryPublisher")
            .field("topics", &"<7 publishers>")
            .finish()
    }
}

impl TelemetryPublisher {
    /// Declare a publisher for each topic. Declaring up-front costs one
    /// round-trip to the Zenoh router per topic; after that, `put` is
    /// async-local and fast.
    pub async fn new(session: &ZenohSession) -> Result<Self, ZenohBusError> {
        Ok(Self {
            imu: session.publisher::<ImuMsg>(topics::IMU_RAW).await?,
            attitude: session.publisher::<AttitudeMsg>(topics::ATTITUDE).await?,
            velocity: session
                .publisher::<VelocityNedMsg>(topics::VELOCITY_NED)
                .await?,
            position: session
                .publisher::<PositionNedMsg>(topics::POSITION_NED)
                .await?,
            setpoint: session
                .publisher::<SetpointPositionMsg>(topics::SETPOINT_POSITION)
                .await?,
            actuator: session
                .publisher::<ActuatorCmdMsg>(topics::ACTUATOR_CMD)
                .await?,
            health: session.publisher::<HealthMsg>(topics::HEALTH).await?,
            _session: session.clone(),
        })
    }

    pub async fn publish_imu(&self, msg: &ImuMsg) -> Result<(), ZenohBusError> {
        self.imu.put(msg).await
    }
    pub async fn publish_attitude(&self, msg: &AttitudeMsg) -> Result<(), ZenohBusError> {
        self.attitude.put(msg).await
    }
    pub async fn publish_velocity(&self, msg: &VelocityNedMsg) -> Result<(), ZenohBusError> {
        self.velocity.put(msg).await
    }
    pub async fn publish_position(&self, msg: &PositionNedMsg) -> Result<(), ZenohBusError> {
        self.position.put(msg).await
    }
    pub async fn publish_setpoint(&self, msg: &SetpointPositionMsg) -> Result<(), ZenohBusError> {
        self.setpoint.put(msg).await
    }
    pub async fn publish_actuator(&self, msg: &ActuatorCmdMsg) -> Result<(), ZenohBusError> {
        self.actuator.put(msg).await
    }
    pub async fn publish_health(&self, msg: &HealthMsg) -> Result<(), ZenohBusError> {
        self.health.put(msg).await
    }
}

/// Default health message for an "all-good" vehicle. Tests that need
/// to inject faults build their own.
#[must_use]
pub fn healthy_msg(timestamp_us: u64) -> HealthMsg {
    HealthMsg {
        timestamp_us,
        overall: HealthLevel::Healthy,
        sensors: [HealthLevel::Healthy; 4],
        fault_flags: 0,
    }
}

/// Helper: set a single fault flag in an existing HealthMsg, bumping
/// the overall severity to Degraded if it was Healthy.
pub fn with_fault(mut msg: HealthMsg, flag: u32) -> HealthMsg {
    msg.fault_flags |= flag;
    if msg.overall == HealthLevel::Healthy {
        msg.overall = HealthLevel::Degraded;
    }
    msg
}

/// Convenience: pre-bake all seven telemetry fields from a
/// [`crate::SimState`] snapshot plus a few extras the caller
/// supplies (setpoint + actuator are caller-owned). Currently unused
/// inside the module but exposed for SITL examples that need to turn
/// the sim snapshot into bus messages on every tick.
#[must_use]
pub fn sim_state_to_telemetry(
    sim: &crate::SimState,
    timestamp_us: u64,
) -> (ImuMsg, AttitudeMsg, VelocityNedMsg, PositionNedMsg) {
    #[allow(clippy::similar_names)]
    {}
    // Synthetic IMU — SITL doesn't separate IMU-sampling from state,
    // so we approximate: gyro = body rate, accel = 0 (the EKF would
    // subtract gravity anyway in a real pipeline).
    let imu = ImuMsg {
        timestamp_us,
        gyro_rad_s: [
            sim.body_rate_rad_s.x,
            sim.body_rate_rad_s.y,
            sim.body_rate_rad_s.z,
        ],
        accel_m_s2: [0.0, 0.0, 0.0],
        temperature_c: 20.0,
    };
    let attitude = AttitudeMsg {
        timestamp_us,
        quaternion: [
            sim.attitude.w,
            sim.attitude.i,
            sim.attitude.j,
            sim.attitude.k,
        ],
        cov_diag: [0.0, 0.0, 0.0],
    };
    let velocity = VelocityNedMsg {
        timestamp_us,
        velocity_m_s: [sim.velocity_ned.x, sim.velocity_ned.y, sim.velocity_ned.z],
        cov_diag: [0.0, 0.0, 0.0],
    };
    let position = PositionNedMsg {
        timestamp_us,
        position_m: [sim.position_ned.x, sim.position_ned.y, sim.position_ned.z],
        cov_diag: [0.0, 0.0, 0.0],
    };
    (imu, attitude, velocity, position)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::zenoh_bus::ZenohSession;
    use core_bus::SensorFaultBit;
    use std::time::Duration;

    /// End-to-end: open a publisher session and a subscriber session
    /// on loopback, push one message to each of the seven telemetry
    /// topics, assert each subscriber sees its typed message with the
    /// expected payload round-tripped through Zenoh + postcard.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn full_catalog_round_trips_through_zenoh() {
        let pub_session = ZenohSession::open_peer().await.unwrap();
        let sub_session = ZenohSession::open_peer().await.unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        // Subscribers declared up front so they don't miss the first
        // put on each topic.
        let mut sub_imu = sub_session
            .subscriber::<ImuMsg>(topics::IMU_RAW)
            .await
            .unwrap();
        let mut sub_att = sub_session
            .subscriber::<AttitudeMsg>(topics::ATTITUDE)
            .await
            .unwrap();
        let mut sub_vel = sub_session
            .subscriber::<VelocityNedMsg>(topics::VELOCITY_NED)
            .await
            .unwrap();
        let mut sub_pos = sub_session
            .subscriber::<PositionNedMsg>(topics::POSITION_NED)
            .await
            .unwrap();
        let mut sub_sp = sub_session
            .subscriber::<SetpointPositionMsg>(topics::SETPOINT_POSITION)
            .await
            .unwrap();
        let mut sub_act = sub_session
            .subscriber::<ActuatorCmdMsg>(topics::ACTUATOR_CMD)
            .await
            .unwrap();
        let mut sub_hlth = sub_session
            .subscriber::<HealthMsg>(topics::HEALTH)
            .await
            .unwrap();

        let pubs = TelemetryPublisher::new(&pub_session).await.unwrap();
        tokio::time::sleep(Duration::from_millis(150)).await;

        // Put one of each.
        let imu = ImuMsg {
            timestamp_us: 1,
            gyro_rad_s: [0.1, 0.2, 0.3],
            accel_m_s2: [0.0, 0.0, -9.8],
            temperature_c: 22.0,
        };
        let att = AttitudeMsg {
            timestamp_us: 2,
            quaternion: [1.0, 0.0, 0.0, 0.0],
            cov_diag: [0.01, 0.01, 0.01],
        };
        let vel = VelocityNedMsg {
            timestamp_us: 3,
            velocity_m_s: [0.0, 0.0, 0.0],
            cov_diag: [0.01, 0.01, 0.01],
        };
        let pos = PositionNedMsg {
            timestamp_us: 4,
            position_m: [0.0, 0.0, -1.0],
            cov_diag: [0.01, 0.01, 0.01],
        };
        let sp = SetpointPositionMsg {
            timestamp_us: 5,
            position_ned_m: [0.0, 0.0, -1.0],
            velocity_ned_m_s: [0.0; 3],
            accel_ned_m_s2: [0.0; 3],
            yaw_rad: 0.0,
        };
        let mut channels = [0.0_f32; core_bus::ACTUATOR_MAX_CHANNELS];
        channels[0] = 0.6;
        channels[1] = 0.7;
        channels[2] = 0.5;
        channels[3] = 0.8;
        let act = ActuatorCmdMsg {
            timestamp_us: 6,
            channels_n: channels,
            n: 4,
        };
        let hlth = with_fault(healthy_msg(7), SensorFaultBit::GPS);

        pubs.publish_imu(&imu).await.unwrap();
        pubs.publish_attitude(&att).await.unwrap();
        pubs.publish_velocity(&vel).await.unwrap();
        pubs.publish_position(&pos).await.unwrap();
        pubs.publish_setpoint(&sp).await.unwrap();
        pubs.publish_actuator(&act).await.unwrap();
        pubs.publish_health(&hlth).await.unwrap();

        let timeout = Duration::from_secs(3);
        assert_eq!(
            tokio::time::timeout(timeout, sub_imu.recv())
                .await
                .unwrap()
                .unwrap(),
            imu
        );
        assert_eq!(
            tokio::time::timeout(timeout, sub_att.recv())
                .await
                .unwrap()
                .unwrap(),
            att
        );
        assert_eq!(
            tokio::time::timeout(timeout, sub_vel.recv())
                .await
                .unwrap()
                .unwrap(),
            vel
        );
        assert_eq!(
            tokio::time::timeout(timeout, sub_pos.recv())
                .await
                .unwrap()
                .unwrap(),
            pos
        );
        assert_eq!(
            tokio::time::timeout(timeout, sub_sp.recv())
                .await
                .unwrap()
                .unwrap(),
            sp
        );
        assert_eq!(
            tokio::time::timeout(timeout, sub_act.recv())
                .await
                .unwrap()
                .unwrap(),
            act
        );
        let got_hlth = tokio::time::timeout(timeout, sub_hlth.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(got_hlth, hlth);
        assert_ne!(got_hlth.fault_flags & SensorFaultBit::GPS, 0);
    }

    #[test]
    fn healthy_msg_has_no_faults() {
        let h = healthy_msg(42);
        assert_eq!(h.overall, HealthLevel::Healthy);
        assert_eq!(h.fault_flags, 0);
        assert!(h.sensors.iter().all(|s| *s == HealthLevel::Healthy));
    }

    #[test]
    fn with_fault_bumps_overall_to_degraded() {
        let h = with_fault(healthy_msg(0), SensorFaultBit::BARO);
        assert_eq!(h.overall, HealthLevel::Degraded);
        assert_ne!(h.fault_flags & SensorFaultBit::BARO, 0);
    }
}
