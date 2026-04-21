//! SITL flight with live MAVLink telemetry to QGroundControl.
//!
//! Run:
//!   cargo run -p sim-hil --example sitl_mavlink [-- 127.0.0.1:14550]
//!
//! Then open QGroundControl. It listens on UDP 14550 by default and
//! should auto-discover the vehicle within a few seconds.
//!
//! What the demo does:
//!   * Initialises the autopilot and sim at 1 m altitude.
//!   * Runs the closed-loop control stack at 1 kHz (predict + PI cascade
//!     + INDI + allocation + drag-free EKF).
//!   * Broadcasts HEARTBEAT at 1 Hz, ATTITUDE at 50 Hz,
//!     GLOBAL_POSITION_INT at 5 Hz.
//!   * Holds the vehicle at an (0, 0, -1 m) hover setpoint forever —
//!     kill with Ctrl-C.

use algo_ekf::GRAVITY_M_S2;
use algo_nmpc::Setpoint;
use app_copter::{
    apply_baro_measurement, apply_gps_measurement, apply_mag_measurement,
    default_config_250g, outer_step, FlightState,
};
use nalgebra::Vector3;
use sim_hil::{
    accel_world, mavlink_udp::MavlinkUdpSink, sense_baro, sense_gps, sense_imu, sense_mag, step,
    SimConfig, SimRng, SimState,
};
use std::time::{Duration, Instant};
use tokio::sync::mpsc;

const DEFAULT_GCS: &str = "127.0.0.1:14550";

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    let target = std::env::args().nth(1).unwrap_or_else(|| DEFAULT_GCS.into());
    tracing_subscriber::fmt::init();
    tracing::info!(%target, "sitl_mavlink: sending MAVLink telemetry");

    let sink = MavlinkUdpSink::bind("0.0.0.0:0", &target).await?;

    // Channel carries (time_ms, attitude, body_rate, pos_ned, vel_ned,
    // heading). Sim thread produces; telemetry task consumes & fans out
    // at per-message rates.
    let (tx, mut rx) = mpsc::unbounded_channel::<Snapshot>();

    let sim_task = tokio::task::spawn_blocking(move || run_sim(tx));

    let start = Instant::now();
    let mut last_hb = Instant::now() - Duration::from_secs(1);
    let mut last_att = Instant::now();
    let mut last_pos = Instant::now();
    let mut latest: Option<Snapshot> = None;

    loop {
        // Drain any incoming snapshots to keep `latest` fresh.
        while let Ok(snap) = rx.try_recv() {
            latest = Some(snap);
        }

        let now = Instant::now();
        let t_ms: u32 = u32::try_from(now.duration_since(start).as_millis()).unwrap_or(u32::MAX);

        if now.duration_since(last_hb) >= Duration::from_secs(1) {
            sink.send_heartbeat().await.ok();
            last_hb = now;
        }
        if let Some(ref s) = latest {
            if now.duration_since(last_att) >= Duration::from_millis(20) {
                sink.send_attitude(t_ms, s.attitude, s.body_rate).await.ok();
                last_att = now;
            }
            if now.duration_since(last_pos) >= Duration::from_millis(200) {
                sink.send_global_position_int(
                    t_ms,
                    47.397_742,  // Zurich (Agilicious's home)
                    8.545_594,
                    408.0,
                    s.position,
                    s.velocity,
                    s.heading,
                )
                .await
                .ok();
                last_pos = now;
            }
        }

        // If the sim task ever exited, bail.
        if sim_task.is_finished() {
            tracing::info!("sim task finished, exiting");
            break;
        }

        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct Snapshot {
    attitude: nalgebra::Quaternion<f32>,
    body_rate: Vector3<f32>,
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    heading: f32,
}

fn run_sim(tx: mpsc::UnboundedSender<Snapshot>) {
    let sim_cfg = SimConfig::realistic_dynamics(Vector3::zeros());
    let mut sim_state = SimState {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        ..SimState::default()
    };
    let mut rng = SimRng::new(42);
    let mut app_cfg = default_config_250g();
    let mut flight = FlightState::default();
    let _ = apply_baro_measurement(
        &mut flight,
        &sense_baro(&sim_cfg, &sim_state, &mut rng),
    );
    let _ = apply_gps_measurement(
        &mut flight,
        &sense_gps(&sim_cfg, &sim_state, &mut rng),
    );
    let setpoint = Setpoint {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        ..Setpoint::default()
    };
    let dt = 0.001_f32;
    let step_duration = Duration::from_millis(1);
    let mut next_wake = Instant::now();
    let mut i: u64 = 0;

    loop {
        let accel_w = accel_world(&sim_cfg, &sim_state);
        let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
        let out = outer_step(&mut app_cfg, &mut flight, imu, dt, &setpoint);
        step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

        if i % 200 == 0 {
            let _ = apply_gps_measurement(
                &mut flight,
                &sense_gps(&sim_cfg, &sim_state, &mut rng),
            );
        }
        if i % 40 == 0 {
            let _ = apply_mag_measurement(
                &mut flight,
                &sense_mag(&sim_cfg, &sim_state, &mut rng),
            );
        }
        if i % 20 == 0 {
            let _ = apply_baro_measurement(
                &mut flight,
                &sense_baro(&sim_cfg, &sim_state, &mut rng),
            );
        }

        // Publish a snapshot at ~100 Hz (every 10 ticks).
        if i % 10 == 0 {
            let (_roll, _pitch, yaw) = comms_mavlink::quaternion_to_euler(flight.state.attitude);
            let snap = Snapshot {
                attitude: flight.state.attitude,
                body_rate: flight.last_gyro_filtered,
                position: flight.state.position_ned,
                velocity: flight.state.velocity_ned,
                heading: yaw,
            };
            if tx.send(snap).is_err() {
                // Receiver dropped — main exited; wind down.
                return;
            }
        }

        i = i.wrapping_add(1);

        // Keep the sim running in wall-clock real time so MAVLink rates
        // look natural in QGC. If we fall behind just catch up.
        next_wake += step_duration;
        let now = Instant::now();
        if next_wake > now {
            std::thread::sleep(next_wake - now);
        } else {
            next_wake = now;
        }
        let _ = GRAVITY_M_S2; // silence unused warning if profile tweaks it
    }
}
