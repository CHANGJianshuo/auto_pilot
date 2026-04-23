# Alerts (MAVLink STATUSTEXT)

Every pilot-facing alert the flight stack emits. All go out as
MAVLink `STATUSTEXT` — the common-denominator message that every GCS
(QGC / Mission Planner / MAVProxy) surfaces as a HUD toast.

Each row lists:

- **Text** — the string emitted (Rust `format!` template). 50-byte
  MAVLink field, null-padded; overflow truncates to 49 bytes.
- **Severity** — the MAVLink severity enum value. GCS styling keys
  off this (EMERGENCY red flashing, INFO grey small).
- **Trigger** — the in-flight-stack edge that fires the alert.
- **Next tick cleanup** — what the pilot / operator should do.

Emission rule across the board: **edge-triggered**. One STATUSTEXT
per real state change, never per tick. Duplicates while the same
state persists are suppressed.

## Motor subsystem

| Text | Severity | Trigger | Next tick cleanup |
|---|---|---|---|
| `MOTOR N FAILED` | `CRITICAL` | `flight.motor_alive[N]` transitions `true → false`. Sources: (a) `MotorFaultDetector` declared after ω̇-residual persistence, or (b) a higher-layer override. | Vehicle has switched to 3-motor failover allocation and is yaw-spinning. Commit to landing immediately — don't attempt position hold beyond tens of seconds. Post-flight: inspect ESC, prop, battery voltage at failure time (logs). |

## Preflight

| Text | Severity | Trigger | Next tick cleanup |
|---|---|---|---|
| `PREFLIGHT: GPS unhealthy` | `ERROR` | `flight.gps_health.level() != Healthy` — 10+ consecutive EKF rejects in the GPS stream. | Wait for better sky view or check antenna cable. ARM will keep failing while this persists. |
| `PREFLIGHT: baro unhealthy` | `ERROR` | `flight.baro_health.level() != Healthy`. | Suspect temperature gradient or sensor contamination. Power-cycle and wait for warm-up. |
| `PREFLIGHT: mag unhealthy` | `ERROR` | `flight.mag_health.level() != Healthy`. | Move away from ferrous metal / power cables. If persistent, calibrate on a non-magnetic surface. |
| `PREFLIGHT: EKF not converged` | `ERROR` | `trace(P_position_block) > PREFLIGHT_POS_TRACE_MAX_M2 (= 5 m²)` — filter hasn't settled yet. | Give it another 5-10 s; the EKF needs a fix + a few GPS updates to converge. |

Preflight alerts re-fire when the *failing subsystem changes* (e.g.
GPS recovers but baro starts failing → two STATUSTEXTs). Same
reason persisting does not re-fire.

## Sensor-health rollup

`flight.overall_health()` is the `max`-severity of all sensor
`SensorRejectionCounter` levels. Escalations emit; improvements do
not (per `HealthLevel`'s no-recovery-in-flight contract).

| Text | Severity | Trigger | Next tick cleanup |
|---|---|---|---|
| `HEALTH: DEGRADED` | `WARNING` | At least one sensor has 10+ consecutive EKF rejects. Mission may continue but with margin eaten. | Reduce mission ambition (don't fly into obstacles, don't attempt RTL across difficult terrain). |
| `HEALTH: EMERGENCY` | `CRITICAL` | At least one sensor has 50+ consecutive EKF rejects. Mission abort. | Land at the nearest safe location — don't try to return to launch if it requires non-trivial navigation. |
| `HEALTH: FAILED` | `EMERGENCY` | A subsystem reached the absorbing Failed state. | Immediate power-off if on ground; if in air, emergency descent (parachute or cut throttle). Vehicle is no longer trustable. |

## Flight mode

Motion-mode entries (TAKEOFF / LAND / RTL) **deliberately don't**
emit STATUSTEXTs — the pilot just sent that command; they're
watching the result in ATTITUDE / GLOBAL_POSITION_INT. Adding "RTL
ACTIVATED" confirmations would pollute the alert channel.

Exception — **landing complete** emits `LANDED` because auto-disarm
happens synchronously with touchdown and the pilot needs to know
before they touch the throttle stick:

| Text | Severity | Trigger | Next tick cleanup |
|---|---|---|---|
| `LANDED` | `INFO` | `flight.landing_state` transitions `Landing → Idle` (touchdown detector fired; `rate_loop_step` has already auto-disarmed). | Vehicle is safe to approach. Power-cycle if the flight logs need pulling. |

## Implementation notes

- Edge detection lives in the telemetry task of the `sitl_mavlink`
  example (`crates/sim-hil/examples/sitl_mavlink.rs`). The firmware
  entry (`crates/app-copter/src/bin/firmware.rs`) will gain the same
  pattern at Phase II with `heapless::String<50>` in place of
  `std::format!`.
- UDP transport means **STATUSTEXT can be lost** without the sender
  knowing. For `CRITICAL` + `EMERGENCY` alerts the firmware will
  (M25, planned) retransmit 3× at 1 s intervals to cross any
  plausible burst-loss window. Best-effort even so; pilots must
  corroborate with the HUD values if an alert seems overdue.
- The 50-byte MAVLink text field is a hard ceiling. Every alert
  template above fits within 49 bytes for all possible parameters
  (longest: `"PREFLIGHT: EKF not converged"` = 28 bytes).

## Severity convention

Mapped onto the PX4 / ArduPilot norm so GCS users see familiar
styling:

- `MAV_SEVERITY_EMERGENCY` (0) — airframe failure, parachute
- `MAV_SEVERITY_ALERT` (1) — (unused by auto_pilot today)
- `MAV_SEVERITY_CRITICAL` (2) — mission abort recommended
- `MAV_SEVERITY_ERROR` (3) — mission cannot start / continue
- `MAV_SEVERITY_WARNING` (4) — safety margin reduced
- `MAV_SEVERITY_NOTICE` (5) — (unused)
- `MAV_SEVERITY_INFO` (6) — (reserved for future ACK-style events)
- `MAV_SEVERITY_DEBUG` (7) — (unused in production)

Unused levels are intentional reservations — adding new alert
classes should use the severity ladder rather than squeezing into
already-used levels.
