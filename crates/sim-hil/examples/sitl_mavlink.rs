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
//!   * Emits STATUSTEXT alerts, edge-triggered:
//!       - CRITICAL "MOTOR N FAILED" on motor-fault declaration
//!       - ERROR "PREFLIGHT: <reason>" when preflight starts /
//!         changes failure mode
//!       - WARNING / CRITICAL / EMERGENCY "HEALTH: <level>" when
//!         the sensor-rollup overall_health escalates
//!     QGC surfaces these as HUD toast notifications.
//!   * Holds the vehicle at an (0, 0, -1 m) hover setpoint forever —
//!     kill with Ctrl-C.

use algo_ekf::GRAVITY_M_S2;
use algo_nmpc::Setpoint;
use algo_fdir::HealthLevel;
use app_copter::{
    ArmState, FlightMode, FlightState, LandingState, PreflightReject, RtlPhase, TakeoffState,
    apply_baro_measurement, apply_gps_measurement, apply_mag_measurement, default_config_250g,
    outer_step, preflight_check,
};
use comms_mavlink::MavSeverity;
use mavlink::common::{MavCmd, MavMessage, MavResult};
use nalgebra::Vector3;
use sim_hil::{
    SimConfig, SimRng, SimState, accel_world,
    mavlink_udp::{
        MavlinkUdpSink, arm_change_from_mav_message, land_request_from_mav_message,
        rtl_request_from_mav_message, setpoint_from_mav_message, takeoff_request_from_mav_message,
    },
    sense_baro, sense_gps, sense_imu, sense_mag, step,
};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::mpsc;

const DEFAULT_GCS: &str = "127.0.0.1:14550";

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    let target = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_GCS.into());
    tracing_subscriber::fmt::init();
    tracing::info!(%target, "sitl_mavlink: sending MAVLink telemetry");

    let sink = MavlinkUdpSink::bind("0.0.0.0:0", &target).await?;

    // Channel carries (time_ms, attitude, body_rate, pos_ned, vel_ned,
    // heading). Sim thread produces; telemetry task consumes & fans out
    // at per-message rates.
    let (tx, mut rx) = mpsc::unbounded_channel::<Snapshot>();

    // Shared setpoint that the sim thread reads each tick. Incoming
    // SET_POSITION_TARGET_LOCAL_NED messages overwrite it.
    let setpoint = Arc::new(Mutex::new(Setpoint {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        ..Setpoint::default()
    }));
    let setpoint_sim = Arc::clone(&setpoint);

    // Shared arm state. Demo starts disarmed so the GCS must actually
    // send MAV_CMD_COMPONENT_ARM_DISARM before motors spin up.
    let arm_state = Arc::new(Mutex::new(ArmState::Disarmed));
    let arm_state_sim = Arc::clone(&arm_state);

    // Shared landing state. LAND command from GCS flips it to Landing;
    // touchdown detector inside outer_step flips it back and auto-disarms.
    let landing_state = Arc::new(Mutex::new(LandingState::Idle));
    let landing_state_sim = Arc::clone(&landing_state);

    // Shared takeoff state. TAKEOFF command flips it to TakingOff with
    // a target z; altitude-reached detector in outer_step flips back.
    let takeoff_state = Arc::new(Mutex::new(TakeoffState::Idle));
    let takeoff_state_sim = Arc::clone(&takeoff_state);

    // Shared RTL phase. RTL command from GCS starts `Climbing`; outer_step
    // drives through the phases and hands off to Landing at the end.
    let rtl_phase = Arc::new(Mutex::new(RtlPhase::Idle));
    let rtl_phase_sim = Arc::clone(&rtl_phase);

    // Shared preflight status. Sim thread writes it each tick by calling
    // `preflight_check`. UDP task reads it when deciding how to respond
    // to ARM / TAKEOFF commands.
    let preflight: Arc<Mutex<Option<PreflightReject>>> = Arc::new(Mutex::new(None));
    let preflight_sim = Arc::clone(&preflight);

    let sim_task = tokio::task::spawn_blocking(move || {
        run_sim(
            tx,
            setpoint_sim,
            arm_state_sim,
            landing_state_sim,
            takeoff_state_sim,
            rtl_phase_sim,
            preflight_sim,
        )
    });

    let start = Instant::now();
    let mut last_hb = Instant::now() - Duration::from_secs(1);
    let mut last_att = Instant::now();
    let mut last_pos = Instant::now();
    let mut latest: Option<Snapshot> = None;
    // Edge-triggered alert state: remember the previous snapshot's
    // motor_alive mask so we only emit a STATUSTEXT on the tick a
    // motor actually transitions from alive to dead (not every tick
    // while it stays dead, which would flood the MAVLink link).
    let mut prev_motor_alive: [bool; 4] = [true; 4];
    // Retry queue for CRITICAL / EMERGENCY alerts. UDP can drop
    // packets silently, so for events the pilot MUST hear we queue
    // two extra retransmits at +1 s and +2 s from the first send.
    // INFO / WARNING don't retry (not worth the link bandwidth).
    let mut alert_retries: Vec<(Instant, MavSeverity, String)> = Vec::new();
    // Edge detection for preflight: on None→Some(reason), emit a
    // STATUSTEXT. On Some(a)→Some(b) where a != b, re-emit (the
    // failing subsystem changed, pilot should know).
    let mut prev_preflight: Option<PreflightReject> = None;
    // Edge detection for overall health: any bump up in severity
    // emits a STATUSTEXT. Drops in severity never emit — levels
    // only recover on the ground (see HealthLevel::reset_on_ground)
    // so a drop should never happen in flight anyway, but silencing
    // "health improved!" toasts is the right default.
    let mut prev_health = HealthLevel::Healthy;
    // Edge detection for landing: Landing→Idle === touchdown
    // detector fired + auto-disarm happened. Emit "LANDED" so the
    // pilot knows the motors are safely off. Unlike the mode-change
    // alerts we deliberately suppress (RTL activated, takeoff
    // reached), this one is pilot-actionable: throttle stick can
    // be moved without spinning motors.
    let mut prev_landing = LandingState::Idle;

    loop {
        // Drain any incoming snapshots to keep `latest` fresh.
        while let Ok(snap) = rx.try_recv() {
            latest = Some(snap);
        }
        // Drain any incoming MAVLink frames. Route each recognised
        // message to the appropriate shared state: SET_POSITION_TARGET
        // → setpoint, COMMAND_LONG(ARM_DISARM) → arm_state,
        // COMMAND_LONG(NAV_LAND) → landing_state. Every COMMAND_LONG
        // gets a COMMAND_ACK back so QGC's UI spinners settle.
        while let Ok(Some((_hdr, msg, _src))) = sink.try_recv() {
            if let Some(new_sp) = setpoint_from_mav_message(&msg) {
                if let Ok(mut sp) = setpoint.lock() {
                    *sp = new_sp;
                    tracing::info!(
                        target_ned = ?new_sp.position_ned,
                        "received new setpoint from GCS"
                    );
                }
            }
            if let Some(arm) = arm_change_from_mav_message(&msg) {
                // DISARM always permitted — failing to disarm is much
                // worse than failing to arm. ARM gated on preflight.
                let new = if arm {
                    ArmState::Armed
                } else {
                    ArmState::Disarmed
                };
                let preflight_err = if arm {
                    preflight.lock().ok().and_then(|g| *g)
                } else {
                    None
                };
                if let Some(reason) = preflight_err {
                    tracing::warn!(reason = reason.reason_str(), "ARM rejected by preflight");
                } else if let Ok(mut a) = arm_state.lock() {
                    if *a != new {
                        *a = new;
                        tracing::info!(?new, "arm state changed by GCS");
                    }
                }
            }
            if land_request_from_mav_message(&msg) {
                if let Ok(mut l) = landing_state.lock() {
                    if *l != LandingState::Landing {
                        *l = LandingState::Landing;
                        tracing::info!("LAND command received — starting autoland");
                    }
                }
            }
            if rtl_request_from_mav_message(&msg) {
                if let Ok(mut r) = rtl_phase.lock() {
                    if *r == RtlPhase::Idle {
                        *r = RtlPhase::Climbing;
                        tracing::info!("RTL command received — returning to launch");
                    }
                }
            }
            if let Some(altitude_m) = takeoff_request_from_mav_message(&msg) {
                // TAKEOFF implies ARM. Both are gated on preflight —
                // don't auto-arm into a known-unhealthy state.
                let preflight_err = preflight.lock().ok().and_then(|g| *g);
                if let Some(reason) = preflight_err {
                    tracing::warn!(
                        reason = reason.reason_str(),
                        "TAKEOFF rejected by preflight"
                    );
                } else {
                    if let Ok(mut a) = arm_state.lock() {
                        *a = ArmState::Armed;
                    }
                    if let Ok(mut t) = takeoff_state.lock() {
                        *t = TakeoffState::TakingOff {
                            target_z_ned: -altitude_m,
                        };
                        tracing::info!(altitude_m, "TAKEOFF command received — climbing to target");
                    }
                }
            }
            // Ack any COMMAND_LONG so the GCS doesn't sit waiting. We
            // accept commands we understand, TEMPORARILY_REJECTED for
            // preflight-gated commands that failed their checks, and
            // UNSUPPORTED for anything else.
            if let MavMessage::COMMAND_LONG(data) = &msg {
                let preflight_err = preflight.lock().ok().and_then(|g| *g);
                // ARM with param1>=0.5 means arm; disarm is never gated.
                let arm_request = matches!(data.command, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM)
                    && data.param1 >= 0.5;
                let result = match data.command {
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                        if arm_request && preflight_err.is_some() {
                            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
                        } else {
                            MavResult::MAV_RESULT_ACCEPTED
                        }
                    }
                    MavCmd::MAV_CMD_NAV_TAKEOFF => {
                        if preflight_err.is_some() {
                            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
                        } else {
                            MavResult::MAV_RESULT_ACCEPTED
                        }
                    }
                    MavCmd::MAV_CMD_NAV_LAND | MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => {
                        MavResult::MAV_RESULT_ACCEPTED
                    }
                    _ => MavResult::MAV_RESULT_UNSUPPORTED,
                };
                sink.send_command_ack(data.command, result).await.ok();
            }
        }

        let now = Instant::now();
        let t_ms: u32 = u32::try_from(now.duration_since(start).as_millis()).unwrap_or(u32::MAX);

        // M25 retry pump: any queued retransmit whose due-time has
        // passed goes out now. Preserves relative ordering because
        // enqueues always push future-dated entries.
        let mut retry_i = 0;
        while retry_i < alert_retries.len() {
            if alert_retries[retry_i].0 <= now {
                let (_, sev, text) = alert_retries.remove(retry_i);
                sink.send_statustext(sev, &text).await.ok();
                tracing::debug!(text = %text, "retransmitted STATUSTEXT");
            } else {
                retry_i += 1;
            }
        }

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
                    t_ms, 47.397_742, // Zurich (Agilicious's home)
                    8.545_594, 408.0, s.position, s.velocity, s.heading,
                )
                .await
                .ok();
                last_pos = now;
            }
            // Edge-triggered motor-failure alert: any motor that just
            // transitioned from alive → dead emits one STATUSTEXT.
            // CRITICAL severity → M25 retry policy applies (two
            // retransmits at +1 s and +2 s).
            for (idx, (&was_alive, &is_alive)) in
                prev_motor_alive.iter().zip(s.motor_alive.iter()).enumerate()
            {
                if was_alive && !is_alive {
                    // Example is host-only — std::format is fine here.
                    // On the embedded target a `heapless::String<50>` +
                    // `core::fmt::write!` would be the equivalent.
                    let text = format!("MOTOR {idx} FAILED");
                    let severity = MavSeverity::MAV_SEVERITY_CRITICAL;
                    sink.send_statustext(severity, &text).await.ok();
                    alert_retries.push((now + Duration::from_secs(1), severity, text.clone()));
                    alert_retries.push((now + Duration::from_secs(2), severity, text));
                    tracing::warn!(motor = idx, "emitted STATUSTEXT: MOTOR FAILED");
                }
            }
            prev_motor_alive = s.motor_alive;

            // Edge-triggered preflight alert: re-emit whenever the
            // failing reason changes (None → Some, or Some(a) → Some(b)).
            // Rate-limit via the equality check — no flood while a
            // single reason persists.
            if s.preflight != prev_preflight {
                if let Some(reason) = s.preflight {
                    let text = format!("PREFLIGHT: {}", reason.reason_str());
                    sink.send_statustext(MavSeverity::MAV_SEVERITY_ERROR, &text)
                        .await
                        .ok();
                    tracing::warn!(reason = reason.reason_str(), "emitted STATUSTEXT: PREFLIGHT");
                }
                prev_preflight = s.preflight;
            }

            // Edge-triggered health escalation. Only climb in severity
            // emits — HealthLevel never recovers in flight so a drop
            // shouldn't happen; if it does we silently accept it
            // (flight stack bug, not a pilot-actionable alert). CRITICAL
            // and EMERGENCY trigger the M25 retry policy.
            if s.overall_health.severity() > prev_health.severity() {
                let (severity, label) = match s.overall_health {
                    HealthLevel::Healthy => (MavSeverity::MAV_SEVERITY_INFO, "HEALTHY"),
                    HealthLevel::Degraded => (MavSeverity::MAV_SEVERITY_WARNING, "DEGRADED"),
                    HealthLevel::Emergency => (MavSeverity::MAV_SEVERITY_CRITICAL, "EMERGENCY"),
                    HealthLevel::Failed => (MavSeverity::MAV_SEVERITY_EMERGENCY, "FAILED"),
                };
                let text = format!("HEALTH: {label}");
                sink.send_statustext(severity, &text).await.ok();
                if matches!(
                    severity,
                    MavSeverity::MAV_SEVERITY_CRITICAL | MavSeverity::MAV_SEVERITY_EMERGENCY
                ) {
                    alert_retries.push((now + Duration::from_secs(1), severity, text.clone()));
                    alert_retries.push((now + Duration::from_secs(2), severity, text));
                }
                tracing::warn!(level = label, "emitted STATUSTEXT: HEALTH");
            }
            prev_health = s.overall_health;

            // Landing completion edge. Auto-disarm fires
            // synchronously with Landing → Idle, so the pilot needs
            // to know NOW (before they reach for the throttle).
            if prev_landing == LandingState::Landing && s.landing_state == LandingState::Idle {
                sink.send_statustext(MavSeverity::MAV_SEVERITY_INFO, "LANDED")
                    .await
                    .ok();
                tracing::info!("emitted STATUSTEXT: LANDED");
            }
            prev_landing = s.landing_state;
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
    /// Per-motor liveness latched by the rate loop (AND of
    /// FlightState.motor_alive + detector output). The telemetry
    /// task does edge detection on this to emit a STATUSTEXT
    /// alert on the tick a motor first goes from alive to dead.
    motor_alive: [bool; 4],
    /// Preflight gate result at this tick: `Some(reason)` if any
    /// arm-gating check is failing, `None` if good-to-arm. Edge-
    /// detected by the telemetry task to surface "PREFLIGHT
    /// REJECTED: <reason>" to the pilot.
    preflight: Option<PreflightReject>,
    /// Sensor-subsystem rollup health. Edge-detected for WARNING
    /// (→ Degraded) / CRITICAL (→ Emergency) STATUSTEXTs.
    overall_health: HealthLevel,
    /// Landing mode state. Edge-detected for `Landing → Idle`
    /// (touchdown detector fired + auto-disarm) to emit a "LANDED"
    /// STATUSTEXT.
    landing_state: LandingState,
    /// Derived top-level flight mode from `FlightState::flight_mode()`.
    /// Not currently used by the telemetry task, but exposed so a
    /// future MODE-CHANGE MAVLink export can pick it up without
    /// re-deriving from the other fields.
    flight_mode: FlightMode,
}

#[allow(clippy::too_many_arguments)]
fn run_sim(
    tx: mpsc::UnboundedSender<Snapshot>,
    setpoint: Arc<Mutex<Setpoint>>,
    arm_state: Arc<Mutex<ArmState>>,
    landing_state: Arc<Mutex<LandingState>>,
    takeoff_state: Arc<Mutex<TakeoffState>>,
    rtl_phase: Arc<Mutex<RtlPhase>>,
    preflight: Arc<Mutex<Option<PreflightReject>>>,
) {
    let sim_cfg = SimConfig::realistic_dynamics(Vector3::zeros());
    let mut sim_state = SimState {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        ..SimState::default()
    };
    let mut rng = SimRng::new(42);
    let mut app_cfg = default_config_250g();
    let mut flight = FlightState::default();
    let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
    let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));
    let dt = 0.001_f32;
    let step_duration = Duration::from_millis(1);
    let mut next_wake = Instant::now();
    let mut i: u64 = 0;

    loop {
        let accel_w = accel_world(&sim_cfg, &sim_state);
        let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
        let current_sp = setpoint.lock().map(|sp| *sp).unwrap_or_else(|_| Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        });
        // Pull externally-driven state into the flight struct.
        flight.arm_state = arm_state.lock().map(|a| *a).unwrap_or(ArmState::Disarmed);
        flight.landing_state = landing_state
            .lock()
            .map(|l| *l)
            .unwrap_or(LandingState::Idle);
        flight.takeoff_state = takeoff_state
            .lock()
            .map(|t| *t)
            .unwrap_or(TakeoffState::Idle);
        flight.rtl_phase = rtl_phase.lock().map(|r| *r).unwrap_or(RtlPhase::Idle);

        let out = outer_step(&mut app_cfg, &mut flight, imu, dt, &current_sp);

        // outer_step may auto-disarm, clear landing, finish takeoff, or
        // advance RTL — push all four back so the telemetry task and
        // the GCS-facing shared state reflect the latest reality.
        if let Ok(mut shared) = arm_state.lock() {
            *shared = flight.arm_state;
        }
        if let Ok(mut shared) = landing_state.lock() {
            *shared = flight.landing_state;
        }
        if let Ok(mut shared) = takeoff_state.lock() {
            *shared = flight.takeoff_state;
        }
        if let Ok(mut shared) = rtl_phase.lock() {
            *shared = flight.rtl_phase;
        }
        // Update preflight snapshot each tick so the UDP task sees a
        // fresh answer when the GCS sends ARM / TAKEOFF.
        if let Ok(mut shared) = preflight.lock() {
            *shared = preflight_check(&flight).err();
        }
        step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

        if i % 200 == 0 {
            let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));
        }
        if i % 40 == 0 {
            let _ = apply_mag_measurement(&mut flight, &sense_mag(&sim_cfg, &sim_state, &mut rng));
        }
        if i % 20 == 0 {
            let _ =
                apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
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
                motor_alive: flight.motor_alive,
                preflight: preflight_check(&flight).err(),
                overall_health: flight.overall_health(),
                landing_state: flight.landing_state,
                flight_mode: flight.flight_mode(),
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
