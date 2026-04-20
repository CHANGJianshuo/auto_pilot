# Message topic catalog

Canonical ROS 2 topic names used across the firmware. The in-firmware bus
(Zenoh-Pico) publishes on these names 1:1 so the same subscribers work
whether they're onboard or on a ROS 2 companion.

| Topic | Rate | Publisher | Payload (TBD IDL) |
|---|---|---|---|
| `auto_pilot/imu/raw` | 1 kHz | `core-hal::imu` | gyro + accel + temp |
| `auto_pilot/estimator/attitude` | 250 Hz | `algo-ekf` | quaternion + cov |
| `auto_pilot/estimator/velocity_ned` | 250 Hz | `algo-ekf` | Vec3 + cov |
| `auto_pilot/estimator/position_ned` | 250 Hz | `algo-ekf` | Vec3 + cov |
| `auto_pilot/control/setpoint_position` | 20–50 Hz | `algo-nmpc` (in) | from nav |
| `auto_pilot/control/actuator_cmd` | 1 kHz | `algo-alloc` | N-channel command |
| `auto_pilot/system/health` | 10 Hz | `algo-fdir` | health level + flags |

Message IDL files will live in `docs/idl/` when the schemas are frozen.
