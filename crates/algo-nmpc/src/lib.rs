#![no_std]

//! Outer position loop.
//!
//! M2 provides a **cascade P–P baseline**: `position_error → velocity_cmd →
//! accel_cmd → (q_desired, thrust_n)`. This is the controller any multirotor
//! needs for M1-level flight testing.
//!
//! M4 will add a real NMPC here: constraint-aware MPC over a 10–20 step
//! horizon with [Clarabel.rs](https://github.com/oxfordcontrol/Clarabel.rs)
//! solving the QP. The `Setpoint` / output types below are designed to
//! accept either controller without changing the call site.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

/// Standard gravity (m/s²), NED convention so `g_vector = (0, 0, +9.80665)`.
pub const GRAVITY_M_S2: f32 = 9.80665;

/// Desired position + velocity + accel feedforward + yaw.
#[derive(Clone, Copy, Debug, Default)]
pub struct Setpoint {
    pub position_ned: Vector3<f32>,
    pub velocity_ned: Vector3<f32>,
    pub accel_ned: Vector3<f32>,
    pub yaw_rad: f32,
}

/// Cascade-P gains.
#[derive(Clone, Copy, Debug)]
pub struct PositionGains {
    pub k_pos: Vector3<f32>,
    pub k_vel: Vector3<f32>,
    /// Saturation on the commanded NED acceleration magnitude (m/s²). Keeps
    /// aggressive setpoints from producing tilt commands that exceed the
    /// feasible thrust envelope.
    pub max_accel: f32,
}

impl Default for PositionGains {
    fn default() -> Self {
        Self {
            k_pos: Vector3::new(1.0, 1.0, 2.0),
            k_vel: Vector3::new(3.0, 3.0, 5.0),
            max_accel: 8.0,
        }
    }
}

/// Output consumed by the attitude loop.
#[derive(Clone, Copy, Debug)]
pub struct AttitudeAndThrust {
    /// Desired body attitude (unit quaternion).
    pub q_desired: Quaternion<f32>,
    /// Total thrust magnitude (N, positive).
    pub thrust_n: f32,
}

/// Baseline cascade controller.
///
/// Returns `(q_desired, thrust_n)` so the attitude loop can take over.
/// Never panics — when the thrust vector or heading is degenerate it
/// falls back to level attitude pointing north with zero thrust.
#[must_use]
pub fn position_to_attitude_thrust(
    setpoint: &Setpoint,
    current_position: Vector3<f32>,
    current_velocity: Vector3<f32>,
    mass_kg: f32,
    gains: &PositionGains,
) -> AttitudeAndThrust {
    // 1. P–P cascade to get commanded acceleration.
    let pos_err = setpoint.position_ned - current_position;
    let vel_cmd = gains.k_pos.component_mul(&pos_err) + setpoint.velocity_ned;
    let vel_err = vel_cmd - current_velocity;
    let accel_cmd = gains.k_vel.component_mul(&vel_err) + setpoint.accel_ned;

    // 2. Saturate acceleration magnitude.
    let accel_mag = accel_cmd.norm();
    let accel_cmd = if accel_mag.is_finite() && accel_mag > gains.max_accel {
        accel_cmd * (gains.max_accel / accel_mag)
    } else {
        accel_cmd
    };

    // 3. Force balance: the vehicle must generate thrust `F = m·(a - g)`.
    //    Thrust acts in the -body_z direction, so desired body_z direction
    //    is −F̂.
    let g_vec = Vector3::new(0.0, 0.0, GRAVITY_M_S2);
    let thrust_vec_ned = mass_kg * (accel_cmd - g_vec); // want body to produce this
    let thrust_mag = thrust_vec_ned.norm();
    let zb_des = if thrust_mag > 1.0e-3 && thrust_mag.is_finite() {
        -thrust_vec_ned / thrust_mag
    } else {
        // Degenerate (free-fall, NaN, or near-zero): fall back to level.
        Vector3::new(0.0, 0.0, -1.0)
    };

    // 4. Project desired heading onto body xy-plane.
    let cy = libm::cosf(setpoint.yaw_rad);
    let sy = libm::sinf(setpoint.yaw_rad);
    let heading = Vector3::new(cy, sy, 0.0);
    let xb_raw = heading - heading.dot(&zb_des) * zb_des;
    let xb_norm = xb_raw.norm();
    let xb_des = if xb_norm > 1.0e-3 {
        xb_raw / xb_norm
    } else {
        // Near singular: pitch-vertical. Pick an arbitrary perpendicular.
        Vector3::new(1.0, 0.0, 0.0)
    };
    let yb_des = zb_des.cross(&xb_des);

    // 5. Assemble rotation matrix (columns are body axes in world NED),
    //    convert to quaternion.
    let r_des = Matrix3::from_columns(&[xb_des, yb_des, zb_des]);
    let q_des = UnitQuaternion::from_matrix(&r_des).into_inner();

    AttitudeAndThrust {
        q_desired: q_des,
        thrust_n: thrust_mag,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    fn default_mass() -> f32 {
        0.25
    }

    #[test]
    fn hover_at_origin_yields_level_attitude_and_hover_thrust() {
        let sp = Setpoint::default(); // pos=0, vel=0, accel=0, yaw=0
        let gains = PositionGains::default();
        let out = position_to_attitude_thrust(
            &sp,
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
            &gains,
        );
        // Expected: thrust = m·g, q = identity
        let expected_thrust = default_mass() * GRAVITY_M_S2;
        assert!(
            (out.thrust_n - expected_thrust).abs() < 1.0e-3,
            "thrust {} expected {}", out.thrust_n, expected_thrust
        );
        // q should be approximately identity: (1, 0, 0, 0) or (-1, 0, 0, 0).
        let q = out.q_desired;
        assert!(
            (q.w.abs() - 1.0).abs() < 1.0e-3,
            "q.w = {} (expected ±1)", q.w
        );
        assert!(q.i.abs() < 1.0e-3);
        assert!(q.j.abs() < 1.0e-3);
        assert!(q.k.abs() < 1.0e-3);
    }

    #[test]
    fn position_error_forward_tilts_nose_down() {
        // Target 1 m forward → vehicle must tilt nose-DOWN (negative pitch)
        // so that thrust has a +x component. NED pitch convention: +pitch is
        // nose-up, so nose-down → q.j < 0.
        let sp = Setpoint {
            position_ned: Vector3::new(1.0, 0.0, 0.0),
            ..Setpoint::default()
        };
        let gains = PositionGains::default();
        let out = position_to_attitude_thrust(
            &sp,
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
            &gains,
        );
        assert!(out.q_desired.j < 0.0, "q.j = {}", out.q_desired.j);
        assert!(out.thrust_n > 0.0);
    }

    #[test]
    fn position_error_below_increases_thrust() {
        // Target at 1 m altitude (z=-1 in NED, down is +z).
        let sp = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };
        let gains = PositionGains::default();
        let out = position_to_attitude_thrust(
            &sp,
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
            &gains,
        );
        // Need to climb → more thrust than hover.
        let hover = default_mass() * GRAVITY_M_S2;
        assert!(out.thrust_n > hover, "thrust {} should > hover {}", out.thrust_n, hover);
    }

    #[test]
    fn saturation_bounds_commanded_accel() {
        // Very far setpoint → accel command saturates.
        let sp = Setpoint {
            position_ned: Vector3::new(1000.0, 0.0, 0.0),
            ..Setpoint::default()
        };
        let gains = PositionGains {
            max_accel: 5.0,
            ..PositionGains::default()
        };
        let out = position_to_attitude_thrust(
            &sp,
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
            &gains,
        );
        // Effective accel should obey max_accel; thrust magnitude should be
        // bounded by m * sqrt(max_accel² + g²).
        let max_thrust = default_mass()
            * (gains.max_accel.powi(2) + GRAVITY_M_S2.powi(2)).sqrt();
        assert!(
            out.thrust_n <= max_thrust + 1.0e-3,
            "thrust {} > saturation bound {}", out.thrust_n, max_thrust
        );
    }

    #[test]
    fn quaternion_is_unit_for_reasonable_inputs() {
        use proptest::prelude::*;
        // Spot-check — full proptest below.
        let sp = Setpoint {
            position_ned: Vector3::new(3.0, -2.0, -4.0),
            velocity_ned: Vector3::new(0.5, 0.0, -0.5),
            accel_ned: Vector3::new(0.1, 0.0, 0.0),
            yaw_rad: 0.3,
        };
        let gains = PositionGains::default();
        let out = position_to_attitude_thrust(
            &sp,
            Vector3::new(1.0, 1.0, -1.0),
            Vector3::new(0.1, 0.0, 0.0),
            default_mass(),
            &gains,
        );
        let n = out.q_desired.norm();
        assert!((n - 1.0).abs() < 1.0e-4, "‖q‖ = {n}");
        let _ = prop::collection::vec(0..1, 0..1); // silence proptest-unused warn
    }

    use proptest::prelude::*;

    proptest! {
        /// For any finite setpoint within realistic bounds, the output
        /// quaternion is unit-norm and the thrust is finite non-negative.
        #[test]
        fn output_is_always_valid(
            sx in -20.0f32..20.0,
            sy in -20.0f32..20.0,
            sz in -20.0f32..20.0,
            yaw in -core::f32::consts::PI..core::f32::consts::PI,
            cx in -20.0f32..20.0,
            cy in -20.0f32..20.0,
            cz in -20.0f32..20.0,
            vx in -10.0f32..10.0,
            vy in -10.0f32..10.0,
            vz in -5.0f32..5.0,
        ) {
            let sp = Setpoint {
                position_ned: Vector3::new(sx, sy, sz),
                velocity_ned: Vector3::zeros(),
                accel_ned: Vector3::zeros(),
                yaw_rad: yaw,
            };
            let out = position_to_attitude_thrust(
                &sp,
                Vector3::new(cx, cy, cz),
                Vector3::new(vx, vy, vz),
                default_mass(),
                &PositionGains::default(),
            );
            prop_assert!(out.thrust_n.is_finite());
            prop_assert!(out.thrust_n >= 0.0);
            let n = out.q_desired.norm();
            prop_assert!((n - 1.0).abs() < 1.0e-3, "‖q‖ = {}", n);
        }
    }
}
