#![no_std]

//! Control allocation — from **virtual commands** (torque + collective thrust)
//! to **per-motor thrusts** for a multirotor.
//!
//! # Math
//!
//! For `N` motors, the effectiveness matrix `E ∈ ℝ^{4×N}` maps a column of
//! motor thrusts to the 4-vector `[τ_roll, τ_pitch, τ_yaw, F_total]`:
//!
//! ```text
//!   E[:, i] = [ -y_i, +x_i, yaw_coef_i, 1 ]ᵀ
//! ```
//!
//! where `(x_i, y_i)` is the motor's body-frame position (forward-right
//! coordinates) and `yaw_coef_i` is the per-unit-thrust yaw reaction torque
//! (positive for a motor whose thrust increase yaws the airframe +z-down).
//!
//! The allocator inverts `E`:
//!
//! ```text
//!   motor_thrusts = E⁺ · virtual_cmd
//! ```
//!
//! For `N = 4` (quadrotor) `E` is square — use `E⁻¹`.
//! For `N > 4` (hex / octa) the caller provides a Moore–Penrose pseudoinverse
//! computed once at boot.
//!
//! # What this module provides
//!
//! * [`MotorGeometry`] — one row of the motor layout.
//! * [`VirtualCommand`] — target torque + thrust.
//! * [`build_effectiveness`] — assemble `E` from a motor array.
//! * [`allocate`] — apply a pre-computed `E⁺` to a virtual command.
//! * [`saturate`] — clip per-motor thrusts to `[min_n, max_n]`.

use nalgebra::{Matrix4, SMatrix, SVector, Vector3, Vector4};

/// Geometry of a single motor in the body frame.
///
/// Convention: body `+x` = forward, `+y` = right, `+z` = down (NED).
/// Motor thrust acts in `-body_z` direction (pushes vehicle up).
#[derive(Clone, Copy, Debug)]
pub struct MotorGeometry {
    /// Motor position in body frame (m). Only `x, y` matter for torque math;
    /// `z` is carried for completeness and future-proofing.
    pub position_m: Vector3<f32>,
    /// Per-unit-thrust yaw-torque coefficient (Nm / N).
    ///
    /// Positive: thrust increase produces `+yaw` body torque.
    /// CCW propellers (viewed from above in NED) typically have **negative**
    /// coefficients because their aero reaction torque opposes the spin
    /// direction.
    pub yaw_torque_per_thrust: f32,
}

/// Target torque + collective thrust from the controller.
#[derive(Clone, Copy, Debug, Default)]
pub struct VirtualCommand {
    /// Body-frame torque (Nm): roll, pitch, yaw.
    pub torque: Vector3<f32>,
    /// Total upward thrust (N). `sum(motor_thrusts) = thrust_n`.
    pub thrust_n: f32,
}

impl VirtualCommand {
    /// Pack into the `[roll, pitch, yaw, thrust]` column used by `E⁺`.
    #[must_use]
    pub fn as_vector4(&self) -> Vector4<f32> {
        Vector4::new(self.torque.x, self.torque.y, self.torque.z, self.thrust_n)
    }
}

/// Assemble the `4 × N` effectiveness matrix `E` for a motor array.
#[must_use]
pub fn build_effectiveness<const N: usize>(motors: &[MotorGeometry; N]) -> SMatrix<f32, 4, N> {
    let mut e: SMatrix<f32, 4, N> = SMatrix::zeros();
    for (i, m) in motors.iter().enumerate() {
        // Column vector [-y, +x, yaw_coef, 1].
        let col = Vector4::new(
            -m.position_m.y,
            m.position_m.x,
            m.yaw_torque_per_thrust,
            1.0,
        );
        e.fixed_view_mut::<4, 1>(0, i).copy_from(&col);
    }
    e
}

/// Invert `E` for the `N = 4` (quadrotor) square case.
///
/// Returns `None` if the matrix is singular (degenerate geometry) — the
/// caller must fall back to a safer control law in that case.
#[must_use]
pub fn invert_quad_effectiveness(effectiveness: &Matrix4<f32>) -> Option<Matrix4<f32>> {
    effectiveness.try_inverse()
}

/// Apply a pre-computed allocator matrix `E_pinv` to a virtual command.
///
/// Result is the target thrust for each of the `N` motors, **before**
/// saturation.
#[must_use]
pub fn allocate<const N: usize>(
    e_pinv: &SMatrix<f32, N, 4>,
    cmd: &VirtualCommand,
) -> SVector<f32, N> {
    e_pinv * cmd.as_vector4()
}

/// Clip every entry of `thrusts` to `[min_n, max_n]` and return the
/// clipped vector. No redistribution of saturated axes — that is the
/// QP allocator's job (M2.1b).
#[must_use]
pub fn saturate<const N: usize>(
    thrusts: SVector<f32, N>,
    min_n: f32,
    max_n: f32,
) -> SVector<f32, N> {
    thrusts.map(|t| t.clamp(min_n, max_n))
}

/// Convenience: the canonical **X-quadrotor** layout.
///
/// Arm length `L` in meters, yaw torque coefficient `k_yaw` in Nm/N.
/// Motors numbered 1..=4 clockwise from the front-right:
///
/// ```text
///         +x (forward)
///          │
///    M3 ← CCW   CW → M1
///          │
///    M2 ← CW    CCW → M4
///          │
///         −x
/// ```
///
/// Spin directions alternate so that pure-thrust (all equal) produces
/// zero net yaw torque.
#[must_use]
pub fn standard_x_quad(arm_m: f32, k_yaw: f32) -> [MotorGeometry; 4] {
    let h = arm_m / core::f32::consts::SQRT_2;
    [
        MotorGeometry {
            position_m: Vector3::new(h, h, 0.0), // front-right, CW
            yaw_torque_per_thrust: k_yaw,
        },
        MotorGeometry {
            position_m: Vector3::new(-h, -h, 0.0), // rear-left, CW
            yaw_torque_per_thrust: k_yaw,
        },
        MotorGeometry {
            position_m: Vector3::new(h, -h, 0.0), // front-left in +x/-y terms (unusual numbering — see struct)
            yaw_torque_per_thrust: -k_yaw,
        },
        MotorGeometry {
            position_m: Vector3::new(-h, h, 0.0),
            yaw_torque_per_thrust: -k_yaw,
        },
    ]
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn test_motors() -> [MotorGeometry; 4] {
        standard_x_quad(0.15, 0.016)
    }

    #[test]
    fn effectiveness_bottom_row_is_ones() {
        let e = build_effectiveness(&test_motors());
        for i in 0..4 {
            let v = e.fixed_view::<1, 1>(3, i).to_scalar();
            assert!((v - 1.0).abs() < 1.0e-9);
        }
    }

    #[test]
    fn effectiveness_is_invertible_for_valid_x_quad() {
        let e = build_effectiveness(&test_motors());
        let e_mat: Matrix4<f32> = e.into_owned();
        let inv = invert_quad_effectiveness(&e_mat);
        assert!(inv.is_some());
        let prod = inv.unwrap() * e_mat;
        assert!((prod - Matrix4::identity()).norm() < 1.0e-4);
    }

    fn cell<const N: usize>(v: &SVector<f32, N>, i: usize) -> f32 {
        v.fixed_view::<1, 1>(i, 0).to_scalar()
    }
    fn cell4(v: &Vector4<f32>, i: usize) -> f32 {
        v.fixed_view::<1, 1>(i, 0).to_scalar()
    }

    #[test]
    fn allocate_pure_thrust_spreads_equally() {
        let e = build_effectiveness(&test_motors());
        let e_mat: Matrix4<f32> = e.into_owned();
        let e_inv = invert_quad_effectiveness(&e_mat).unwrap();
        let cmd = VirtualCommand {
            torque: Vector3::zeros(),
            thrust_n: 8.0, // hover total
        };
        let t = allocate(&e_inv, &cmd);
        // All four motors should give ~2 N each.
        for i in 0..4 {
            let ti = cell(&t, i);
            assert!((ti - 2.0).abs() < 1.0e-4, "motor {i} = {ti}");
        }
    }

    #[test]
    fn allocate_roll_torque_asymmetric_thrust() {
        let e = build_effectiveness(&test_motors());
        let e_mat: Matrix4<f32> = e.into_owned();
        let e_inv = invert_quad_effectiveness(&e_mat).unwrap();
        let cmd = VirtualCommand {
            torque: Vector3::new(0.1, 0.0, 0.0), // positive roll (right-wing down)
            thrust_n: 8.0,
        };
        let t = allocate(&e_inv, &cmd);
        // With τ_roll = -y·T:  +τ_roll needs y>0 motors to drop, y<0 motors to rise.
        // Motor layout in standard_x_quad:
        //   M1 (idx 0) at (+h, +h): y>0 → less
        //   M2 (idx 1) at (-h, -h): y<0 → more
        //   M3 (idx 2) at (+h, -h): y<0 → more
        //   M4 (idx 3) at (-h, +h): y>0 → less
        assert!(cell(&t, 0) < 2.0);
        assert!(cell(&t, 1) > 2.0);
        assert!(cell(&t, 2) > 2.0);
        assert!(cell(&t, 3) < 2.0);
    }

    #[test]
    fn saturate_clips_values() {
        let v = Vector4::new(-0.5, 0.2, 1.5, 3.0);
        let c = saturate(v, 0.0, 2.0);
        assert!((cell4(&c, 0) - 0.0).abs() < 1.0e-9);
        assert!((cell4(&c, 1) - 0.2).abs() < 1.0e-6);
        assert!((cell4(&c, 2) - 1.5).abs() < 1.0e-6);
        assert!((cell4(&c, 3) - 2.0).abs() < 1.0e-9);
    }

    proptest! {
        /// Unsaturated round-trip: allocating then re-applying E yields the
        /// original virtual command within 1 mN·m / 1 mN.
        #[test]
        fn allocate_roundtrip_matches_virtual(
            roll in -0.05f32..0.05,
            pitch in -0.05f32..0.05,
            yaw  in -0.01f32..0.01,
            thrust in 2.0f32..12.0,
        ) {
            let e = build_effectiveness(&test_motors());
            let e_mat: Matrix4<f32> = e.into_owned();
            let e_inv = invert_quad_effectiveness(&e_mat).unwrap();
            let cmd = VirtualCommand {
                torque: Vector3::new(roll, pitch, yaw),
                thrust_n: thrust,
            };
            let thrusts = allocate(&e_inv, &cmd);
            // Reconstruct virtual command from thrusts.
            let v_back = e_mat * thrusts;
            prop_assert!((cell4(&v_back, 0) - roll).abs() < 1.0e-3);
            prop_assert!((cell4(&v_back, 1) - pitch).abs() < 1.0e-3);
            prop_assert!((cell4(&v_back, 2) - yaw).abs() < 1.0e-3);
            prop_assert!((cell4(&v_back, 3) - thrust).abs() < 1.0e-3);
        }

        /// Saturate is idempotent and stays in range.
        #[test]
        fn saturate_idempotent_and_bounded(
            a in -10.0f32..10.0,
            b in -10.0f32..10.0,
            c in -10.0f32..10.0,
            d in -10.0f32..10.0,
        ) {
            let v = Vector4::new(a, b, c, d);
            let s1 = saturate(v, 0.0, 5.0);
            let s2 = saturate(s1, 0.0, 5.0);
            prop_assert_eq!(s1, s2);
            for i in 0..4 {
                let val = cell4(&s1, i);
                prop_assert!((0.0..=5.0).contains(&val));
            }
        }
    }
}
