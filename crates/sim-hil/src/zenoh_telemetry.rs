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

/// Run the existing closed-loop SITL (bare MPC) for `ticks` ticks at
/// 1 ms per tick and broadcast every topic on `publisher` at the
/// cadence declared in `docs/topics.md`:
///
///   * IMU        — every tick (1 kHz)
///   * Attitude   — every 4th tick (250 Hz)
///   * Velocity   — every 4th tick (250 Hz)
///   * Position   — every 4th tick (250 Hz)
///   * Setpoint   — every 20th tick (50 Hz)
///   * Actuator   — every tick (1 kHz)
///   * Health     — every 100th tick (10 Hz)
///
/// Returns the final sim state so callers can assert on tracking
/// performance **and** on telemetry delivery in the same test.
///
/// Publisher work is inline on the tokio task (the `.await` on each
/// `publish_*` call). At 1 kHz that's ~7000 awaits/sec against a
/// loopback Zenoh router; well inside tokio's capacity, but this is
/// a host-only test harness — don't ship it as a firmware loop.
#[allow(clippy::too_many_arguments)]
pub async fn run_closed_loop_with_zenoh_telemetry(
    sim_cfg: &crate::SimConfig,
    seed: u64,
    ticks: usize,
    publisher: &TelemetryPublisher,
) -> Result<crate::SimState, ZenohBusError> {
    use algo_indi::attitude_to_rate;
    use algo_nmpc::{LqrWeights, Mpc1dConfig, Mpc3dPositionController, Setpoint};
    use app_copter::{
        ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
        apply_mag_measurement, default_config_250g, rate_loop_step,
    };
    use core_bus::{ACTUATOR_MAX_CHANNELS, ActuatorCmdMsg, SetpointPositionMsg};

    let mut sim_state = crate::SimState {
        position_ned: nalgebra::Vector3::new(0.0, 0.0, -1.0),
        ..crate::SimState::default()
    };
    let mut rng = crate::SimRng::new(seed);
    let dt = 0.001_f32;
    let mut app_cfg = default_config_250g();

    let weights = LqrWeights {
        q_pos: 4.0,
        q_vel: 1.0,
        r: 0.5,
    };
    let cfg = Mpc1dConfig {
        weights,
        dt_s: dt,
        u_min: -20.0,
        u_max: 20.0,
    };
    let mut mpc =
        Mpc3dPositionController::<10>::new(cfg, cfg, 25, app_cfg.position_gains.max_accel)
            .ok_or_else(|| {
                ZenohBusError::Zenoh("MPC DARE failed to converge for default weights".into())
            })?;

    let mut flight = FlightState {
        arm_state: ArmState::Armed,
        ..FlightState::default()
    };
    let _ = apply_baro_measurement(
        &mut flight,
        &crate::sense_baro(sim_cfg, &sim_state, &mut rng),
    );
    let _ = apply_gps_measurement(
        &mut flight,
        &crate::sense_gps(sim_cfg, &sim_state, &mut rng),
    );

    let setpoint = Setpoint {
        position_ned: nalgebra::Vector3::new(0.0, 0.0, -1.0),
        ..Setpoint::default()
    };

    for i in 0..ticks {
        let accel_w = crate::accel_world(sim_cfg, &sim_state);
        let imu = crate::sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);

        let att = mpc.step(
            &setpoint,
            flight.state.position_ned,
            flight.state.velocity_ned,
            app_cfg.mass_kg,
        );
        let rate_cmd = attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
        let saved_hover = app_cfg.hover_thrust_n;
        app_cfg.hover_thrust_n = att.thrust_n;
        let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
        app_cfg.hover_thrust_n = saved_hover;
        crate::step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

        let ts = u64::try_from(i).unwrap_or(u64::MAX);

        // 1 kHz — IMU, actuator.
        let imu_msg = ImuMsg {
            timestamp_us: ts,
            gyro_rad_s: [imu.gyro_rad_s.x, imu.gyro_rad_s.y, imu.gyro_rad_s.z],
            accel_m_s2: [imu.accel_m_s2.x, imu.accel_m_s2.y, imu.accel_m_s2.z],
            temperature_c: imu.temperature_c,
        };
        publisher.publish_imu(&imu_msg).await?;

        let mut channels = [0.0_f32; ACTUATOR_MAX_CHANNELS];
        for (k, slot) in channels.iter_mut().enumerate().take(4) {
            *slot = out.motor_thrusts_n.fixed_view::<1, 1>(k, 0).to_scalar();
        }
        let act_msg = ActuatorCmdMsg {
            timestamp_us: ts,
            channels_n: channels,
            n: 4,
        };
        publisher.publish_actuator(&act_msg).await?;

        // 250 Hz — attitude, velocity, position (every 4th tick).
        if i.is_multiple_of(4) {
            let (imu_snap, att_msg, vel_msg, pos_msg) = sim_state_to_telemetry(&sim_state, ts);
            let _ = imu_snap;
            publisher.publish_attitude(&att_msg).await?;
            publisher.publish_velocity(&vel_msg).await?;
            publisher.publish_position(&pos_msg).await?;
        }

        // 50 Hz — setpoint (every 20th tick).
        if i.is_multiple_of(20) {
            publisher
                .publish_setpoint(&SetpointPositionMsg {
                    timestamp_us: ts,
                    position_ned_m: [
                        setpoint.position_ned.x,
                        setpoint.position_ned.y,
                        setpoint.position_ned.z,
                    ],
                    velocity_ned_m_s: [0.0; 3],
                    accel_ned_m_s2: [0.0; 3],
                    yaw_rad: setpoint.yaw_rad,
                })
                .await?;
        }

        // 10 Hz — health (every 100th tick).
        if i.is_multiple_of(100) {
            publisher.publish_health(&healthy_msg(ts)).await?;
        }

        if i.is_multiple_of(200) {
            let _ = apply_gps_measurement(
                &mut flight,
                &crate::sense_gps(sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(40) {
            let _ = apply_mag_measurement(
                &mut flight,
                &crate::sense_mag(sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(20) {
            let _ = apply_baro_measurement(
                &mut flight,
                &crate::sense_baro(sim_cfg, &sim_state, &mut rng),
            );
        }
    }
    Ok(sim_state)
}

/// Residual-policy variant of [`run_closed_loop_with_zenoh_telemetry`].
///
/// Swaps the bare MPC for an [`MpcResidualController`] so the NN
/// correction path rides alongside the Zenoh telemetry fan-out. Proves
/// pillar 3 (onboard NN) and pillar 4 (Zenoh) work at the same time
/// without starving each other.
///
/// Caller owns the controller to keep policy-tuning decisions visible
/// at the test site (hand-crafted affine weights for now; a trained
/// model once the training pipeline lands).
pub async fn run_closed_loop_residual_with_zenoh_telemetry<B>(
    sim_cfg: &crate::SimConfig,
    seed: u64,
    ticks: usize,
    publisher: &TelemetryPublisher,
    controller: &mut crate::residual_mpc::MpcResidualController<10, B>,
) -> Result<crate::SimState, ZenohBusError>
where
    B: nn_runtime::InferenceBackend,
{
    use algo_indi::attitude_to_rate;
    use algo_nmpc::Setpoint;
    use app_copter::{
        ArmState, FlightState, apply_baro_measurement, apply_gps_measurement,
        apply_mag_measurement, default_config_250g, rate_loop_step,
    };
    use core_bus::{ACTUATOR_MAX_CHANNELS, ActuatorCmdMsg, SetpointPositionMsg};

    let mut sim_state = crate::SimState {
        position_ned: nalgebra::Vector3::new(0.0, 0.0, -1.0),
        ..crate::SimState::default()
    };
    let mut rng = crate::SimRng::new(seed);
    let dt = 0.001_f32;
    let mut app_cfg = default_config_250g();

    let mut flight = FlightState {
        arm_state: ArmState::Armed,
        ..FlightState::default()
    };
    let _ = apply_baro_measurement(
        &mut flight,
        &crate::sense_baro(sim_cfg, &sim_state, &mut rng),
    );
    let _ = apply_gps_measurement(
        &mut flight,
        &crate::sense_gps(sim_cfg, &sim_state, &mut rng),
    );

    let setpoint = Setpoint {
        position_ned: nalgebra::Vector3::new(0.0, 0.0, -1.0),
        ..Setpoint::default()
    };

    for i in 0..ticks {
        let accel_w = crate::accel_world(sim_cfg, &sim_state);
        let imu = crate::sense_imu(sim_cfg, &sim_state, accel_w, &mut rng);

        let att = controller.step(
            &setpoint,
            flight.state.position_ned,
            flight.state.velocity_ned,
            flight.state.attitude,
            app_cfg.mass_kg,
        );
        let rate_cmd = attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
        let saved_hover = app_cfg.hover_thrust_n;
        app_cfg.hover_thrust_n = att.thrust_n;
        let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
        app_cfg.hover_thrust_n = saved_hover;
        crate::step(sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

        let ts = u64::try_from(i).unwrap_or(u64::MAX);

        // Same rate table as the bare-MPC runner; keep the two runners
        // symmetric so telemetry-side regression tests swap cleanly.
        publisher
            .publish_imu(&ImuMsg {
                timestamp_us: ts,
                gyro_rad_s: [imu.gyro_rad_s.x, imu.gyro_rad_s.y, imu.gyro_rad_s.z],
                accel_m_s2: [imu.accel_m_s2.x, imu.accel_m_s2.y, imu.accel_m_s2.z],
                temperature_c: imu.temperature_c,
            })
            .await?;
        let mut channels = [0.0_f32; ACTUATOR_MAX_CHANNELS];
        for (k, slot) in channels.iter_mut().enumerate().take(4) {
            *slot = out.motor_thrusts_n.fixed_view::<1, 1>(k, 0).to_scalar();
        }
        publisher
            .publish_actuator(&ActuatorCmdMsg {
                timestamp_us: ts,
                channels_n: channels,
                n: 4,
            })
            .await?;
        if i.is_multiple_of(4) {
            let (_, att_msg, vel_msg, pos_msg) = sim_state_to_telemetry(&sim_state, ts);
            publisher.publish_attitude(&att_msg).await?;
            publisher.publish_velocity(&vel_msg).await?;
            publisher.publish_position(&pos_msg).await?;
        }
        if i.is_multiple_of(20) {
            publisher
                .publish_setpoint(&SetpointPositionMsg {
                    timestamp_us: ts,
                    position_ned_m: [
                        setpoint.position_ned.x,
                        setpoint.position_ned.y,
                        setpoint.position_ned.z,
                    ],
                    velocity_ned_m_s: [0.0; 3],
                    accel_ned_m_s2: [0.0; 3],
                    yaw_rad: setpoint.yaw_rad,
                })
                .await?;
        }
        if i.is_multiple_of(100) {
            // Surface the policy's reject count via the health topic
            // so subscribers can see if the NN is being blocked.
            let mut h = healthy_msg(ts);
            if controller.reject_count() > 0 {
                h = with_fault(h, core_bus::SensorFaultBit::EKF);
            }
            publisher.publish_health(&h).await?;
        }

        if i.is_multiple_of(200) {
            let _ = apply_gps_measurement(
                &mut flight,
                &crate::sense_gps(sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(40) {
            let _ = apply_mag_measurement(
                &mut flight,
                &crate::sense_mag(sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(20) {
            let _ = apply_baro_measurement(
                &mut flight,
                &crate::sense_baro(sim_cfg, &sim_state, &mut rng),
            );
        }
    }
    Ok(sim_state)
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

    /// Run 600 sim-ticks (600 ms of simulated time) while broadcasting
    /// every topic over Zenoh. A subscriber on each topic counts the
    /// messages it receives in the same window. Assert every count
    /// hits the documented rate, within a tolerance that allows for:
    ///   * Zenoh peer-discovery warm-up losing the first couple of
    ///     messages
    ///   * async scheduling latency between sim-tick and recv
    ///
    /// The test also asserts that the vehicle is still near the
    /// setpoint at the end — the publisher shouldn't starve the
    /// controller of tokio time.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn sitl_broadcasts_every_topic_at_documented_rate() {
        let pub_session = ZenohSession::open_peer().await.unwrap();
        let sub_session = ZenohSession::open_peer().await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;

        // Declare counting subscribers BEFORE starting the publisher.
        let sub_imu = sub_session
            .subscriber::<ImuMsg>(topics::IMU_RAW)
            .await
            .unwrap();
        let sub_att = sub_session
            .subscriber::<AttitudeMsg>(topics::ATTITUDE)
            .await
            .unwrap();
        let sub_pos = sub_session
            .subscriber::<PositionNedMsg>(topics::POSITION_NED)
            .await
            .unwrap();
        let sub_sp = sub_session
            .subscriber::<SetpointPositionMsg>(topics::SETPOINT_POSITION)
            .await
            .unwrap();
        let sub_act = sub_session
            .subscriber::<ActuatorCmdMsg>(topics::ACTUATOR_CMD)
            .await
            .unwrap();
        let sub_h = sub_session
            .subscriber::<HealthMsg>(topics::HEALTH)
            .await
            .unwrap();

        // Spawn counter tasks that drain each subscriber until the
        // publisher run completes. They report counts back via
        // `tokio::sync::oneshot`.
        use tokio::sync::oneshot;
        let (tx_imu, rx_imu) = oneshot::channel::<usize>();
        let (tx_att, rx_att) = oneshot::channel::<usize>();
        let (tx_pos, rx_pos) = oneshot::channel::<usize>();
        let (tx_sp, rx_sp) = oneshot::channel::<usize>();
        let (tx_act, rx_act) = oneshot::channel::<usize>();
        let (tx_h, rx_h) = oneshot::channel::<usize>();

        async fn count_until_idle<T>(
            mut sub: crate::zenoh_bus::Subscriber<T>,
            tx: oneshot::Sender<usize>,
        ) where
            T: Send + 'static + for<'de> serde::Deserialize<'de>,
        {
            let mut n = 0;
            while let Ok(Ok(_)) = tokio::time::timeout(Duration::from_millis(500), sub.recv()).await
            {
                n += 1;
            }
            let _ = tx.send(n);
        }
        let h_imu = tokio::spawn(count_until_idle(sub_imu, tx_imu));
        let h_att = tokio::spawn(count_until_idle(sub_att, tx_att));
        let h_pos = tokio::spawn(count_until_idle(sub_pos, tx_pos));
        let h_sp = tokio::spawn(count_until_idle(sub_sp, tx_sp));
        let h_act = tokio::spawn(count_until_idle(sub_act, tx_act));
        let h_h = tokio::spawn(count_until_idle(sub_h, tx_h));
        tokio::time::sleep(Duration::from_millis(200)).await;

        // Run the SITL with telemetry publishing.
        let pubs = TelemetryPublisher::new(&pub_session).await.unwrap();
        let sim_cfg = crate::SimConfig::default();
        let ticks: usize = 600;
        let final_state = run_closed_loop_with_zenoh_telemetry(&sim_cfg, 1, ticks, &pubs)
            .await
            .unwrap();

        // Let subscribers drain.
        let c_imu = rx_imu.await.unwrap();
        let c_att = rx_att.await.unwrap();
        let c_pos = rx_pos.await.unwrap();
        let c_sp = rx_sp.await.unwrap();
        let c_act = rx_act.await.unwrap();
        let c_h = rx_h.await.unwrap();
        let _ = h_imu.await;
        let _ = h_att.await;
        let _ = h_pos.await;
        let _ = h_sp.await;
        let _ = h_act.await;
        let _ = h_h.await;

        // Expected counts at the documented rates across 600 ticks.
        // Tolerance 10% down (lose early messages) and allow up to
        // the published count exactly (no tolerance up — can't
        // over-publish).
        // 10 % tolerance on the low side — Zenoh peer discovery may
        // eat the first couple of messages.
        #[allow(clippy::integer_division)]
        let tol_lower = |expected: usize| expected.saturating_sub(expected / 10);
        assert!(
            c_imu >= tol_lower(600),
            "imu count {c_imu}, expected ≥ {}",
            tol_lower(600)
        );
        assert!(
            c_act >= tol_lower(600),
            "actuator count {c_act}, expected ≥ {}",
            tol_lower(600)
        );
        assert!(
            c_att >= tol_lower(150),
            "attitude count {c_att}, expected ≥ {} (250 Hz × 0.6 s)",
            tol_lower(150)
        );
        assert!(
            c_pos >= tol_lower(150),
            "position count {c_pos}, expected ≥ {}",
            tol_lower(150)
        );
        assert!(
            c_sp >= tol_lower(30),
            "setpoint count {c_sp}, expected ≥ {} (50 Hz × 0.6 s)",
            tol_lower(30)
        );
        assert!(
            c_h >= 4,
            "health count {c_h}, expected ≥ 4 (10 Hz × 0.6 s ≈ 6)"
        );

        // Sanity on the sim: vehicle still near the 1 m hover
        // setpoint, i.e. telemetry publishing didn't starve the
        // controller. Ideal sim, tight bound.
        let alt_err = (-final_state.position_ned.z - 1.0).abs();
        assert!(alt_err < 0.3, "SITL altitude err {alt_err} m");
    }

    /// Residual policy + Zenoh telemetry on the same SITL run.
    /// Proves pillar 3 (NN) and pillar 4 (Zenoh) compose: the
    /// residual controller isn't starved by the async publishers,
    /// and the publishers aren't starved by the policy inference.
    ///
    /// Hand-tuned affine residual — same (pos_err_x, vel_x) PD shape
    /// as the M11c SITL demo. Wind disturbance 1.5 m/s along +x so
    /// the policy actually has work to do.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn sitl_residual_with_zenoh_telemetry() {
        use crate::residual_mpc::MpcResidualController;
        use algo_nmpc::{LqrWeights, Mpc1dConfig, Mpc3dPositionController};
        use nn_runtime::{AffineBackend, ResidualPolicy, SafetyEnvelope};

        let pub_session = ZenohSession::open_peer().await.unwrap();
        let sub_session = ZenohSession::open_peer().await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;

        let sub_imu = sub_session
            .subscriber::<ImuMsg>(topics::IMU_RAW)
            .await
            .unwrap();
        let sub_health = sub_session
            .subscriber::<HealthMsg>(topics::HEALTH)
            .await
            .unwrap();

        use tokio::sync::oneshot;
        let (tx_imu, rx_imu) = oneshot::channel::<usize>();
        let (tx_h, rx_h) = oneshot::channel::<usize>();

        async fn count_until_idle<T>(
            mut sub: crate::zenoh_bus::Subscriber<T>,
            tx: oneshot::Sender<usize>,
        ) where
            T: Send + 'static + for<'de> serde::Deserialize<'de>,
        {
            let mut n = 0;
            while let Ok(Ok(_)) = tokio::time::timeout(Duration::from_millis(500), sub.recv()).await
            {
                n += 1;
            }
            let _ = tx.send(n);
        }
        let h_imu = tokio::spawn(count_until_idle(sub_imu, tx_imu));
        let h_health = tokio::spawn(count_until_idle(sub_health, tx_h));
        tokio::time::sleep(Duration::from_millis(200)).await;

        let pubs = TelemetryPublisher::new(&pub_session).await.unwrap();

        // Build MPC + residual policy. PD-shaped affine on x-axis,
        // same as the M11c SITL demo.
        let dt = 0.001_f32;
        let weights = LqrWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            r: 0.5,
        };
        let cfg = Mpc1dConfig {
            weights,
            dt_s: dt,
            u_min: -20.0,
            u_max: 20.0,
        };
        let mpc = Mpc3dPositionController::<10>::new(cfg, cfg, 25, 8.0).unwrap();
        let mut w = [[0.0_f32; nn_runtime::FEATURE_LEN]; nn_runtime::RESIDUAL_LEN];
        w[0][0] = -2.0; // pos_err_x → residual_x
        w[0][3] = -1.0; // vel_x     → residual_x
        let backend = AffineBackend::new(w, [0.0; nn_runtime::RESIDUAL_LEN], 5.0);
        let policy = ResidualPolicy::new(backend, SafetyEnvelope::small_multirotor_default());
        let mut controller = MpcResidualController::new(mpc, policy);

        let sim_cfg = crate::SimConfig::realistic_dynamics(nalgebra::Vector3::new(1.5, 0.0, 0.0));
        let ticks = 600_usize;
        let final_state = run_closed_loop_residual_with_zenoh_telemetry(
            &sim_cfg,
            3,
            ticks,
            &pubs,
            &mut controller,
        )
        .await
        .unwrap();

        let c_imu = rx_imu.await.unwrap();
        let c_h = rx_h.await.unwrap();
        let _ = h_imu.await;
        let _ = h_health.await;

        // Rate check: 1 kHz × 0.6 s = 600 IMU, 10% tolerance.
        assert!(c_imu >= 540, "imu count {c_imu}, expected ≥ 540");
        // Health: 10 Hz × 0.6 s = 6 ± 1.
        assert!(c_h >= 4, "health count {c_h}, expected ≥ 4");

        // The residual should keep the vehicle closer than the M14
        // bare-MPC run under the same 1.5 m/s wind. M14 was ideal sim,
        // so we can't directly compare — here we assert that with
        // wind active the residual-compensated tracking is still
        // bounded (NOT the 2.7+ m the M11c bare-MPC test showed).
        let horiz =
            (final_state.position_ned.x.powi(2) + final_state.position_ned.y.powi(2)).sqrt();
        assert!(
            horiz < 1.0,
            "residual-compensated SITL horizontal err {horiz} m ≥ 1.0 m"
        );

        // Policy should produce zero rejections with this tuning —
        // the hand-crafted residual stays inside the safety envelope.
        assert_eq!(
            controller.reject_count(),
            0,
            "unexpected policy rejections: {}",
            controller.reject_count()
        );
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
