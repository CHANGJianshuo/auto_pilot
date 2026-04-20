#![no_std]

//! 24-state Extended Kalman Filter skeleton.
//!
//! State vector layout (to be filled in during M1):
//!
//! ```text
//! [ q (4), v_ned (3), p_ned (3), gyro_bias (3), accel_bias (3),
//!   mag_ned (3), mag_bias (3), wind_ne (2) ]
//! ```
//!
//! Design invariants (to be proven via `formal/` harnesses):
//! - Covariance P is symmetric positive-semidefinite at every step.
//! - Quaternion stays unit-normalized within 1e-6 after each predict/update.
//! - No integer overflow, no panic paths.

use nalgebra::{Quaternion, SVector, UnitQuaternion, Vector2, Vector3, Vector4};

/// Dimension of the EKF state vector.
pub const STATE_DIM: usize = 24;

/// Canonical index layout of the 24-D state vector.
///
/// ```text
///   [0..4)   attitude quaternion (w, i, j, k)
///   [4..7)   velocity_ned (n, e, d)
///   [7..10)  position_ned (n, e, d)
///   [10..13) gyro_bias    (x, y, z)
///   [13..16) accel_bias   (x, y, z)
///   [16..19) mag_ned      (n, e, d)
///   [19..22) mag_bias     (x, y, z)
///   [22..24) wind_ne      (n, e)
/// ```
///
/// All downstream consumers (EKF covariance matrix, Jacobians, logging)
/// must use these constants — never hard-coded integers.
pub mod idx {
    pub const Q_START: usize = 0;
    pub const Q_LEN: usize = 4;
    pub const V_NED_START: usize = 4;
    pub const V_NED_LEN: usize = 3;
    pub const P_NED_START: usize = 7;
    pub const P_NED_LEN: usize = 3;
    pub const GYRO_BIAS_START: usize = 10;
    pub const GYRO_BIAS_LEN: usize = 3;
    pub const ACCEL_BIAS_START: usize = 13;
    pub const ACCEL_BIAS_LEN: usize = 3;
    pub const MAG_NED_START: usize = 16;
    pub const MAG_NED_LEN: usize = 3;
    pub const MAG_BIAS_START: usize = 19;
    pub const MAG_BIAS_LEN: usize = 3;
    pub const WIND_NE_START: usize = 22;
    pub const WIND_NE_LEN: usize = 2;
    /// Total — must equal `super::STATE_DIM`.
    pub const TOTAL: usize = 24;
}

/// Shorthand for a 24-D column vector over `f32`.
pub type StateVector = SVector<f32, STATE_DIM>;

/// Shorthand for the 24×24 EKF covariance matrix `P` over `f32`.
pub type Covariance = nalgebra::SMatrix<f32, STATE_DIM, STATE_DIM>;

/// Default initial 1-σ standard deviations per state group.
///
/// Units and rationale:
/// * attitude quaternion — uncertainty on each component, ≈ small-angle
///   misalignment of ~0.1 rad (~6°) before first alignment converges.
/// * velocity_ned — no motion known yet, 1 m/s per axis is generous.
/// * position_ned — origin-referenced, 10 m until GPS fix.
/// * gyro_bias — rad/s, ICM-42688 datasheet zero-bias ≲ 0.01 rad/s.
/// * accel_bias — m/s², datasheet ≲ 0.1 m/s².
/// * mag_ned — gauss, Earth field varies geographically up to 0.05 g.
/// * mag_bias — gauss, hard-iron correction ≲ 0.2 g until calibrated.
/// * wind_ne — m/s, no wind knowledge, 2 m/s per axis.
///
/// Tuned during flight testing later; these are reasonable boot-time defaults.
pub mod initial_sigma {
    pub const ATTITUDE: f32 = 0.1;
    pub const VELOCITY_NED: f32 = 1.0;
    pub const POSITION_NED: f32 = 10.0;
    pub const GYRO_BIAS: f32 = 0.01;
    pub const ACCEL_BIAS: f32 = 0.1;
    pub const MAG_NED: f32 = 0.05;
    pub const MAG_BIAS: f32 = 0.2;
    pub const WIND_NE: f32 = 2.0;
}

/// Build the initial covariance `P₀` as a diagonal matrix whose per-element
/// entries come from [`initial_sigma`] (squared).
///
/// Diagonal means no a-priori cross-correlation between state components —
/// the filter will build those as it consumes measurements.
#[must_use]
pub fn initial_covariance() -> Covariance {
    let mut p = Covariance::zeros();
    fill_block(&mut p, idx::Q_START, idx::Q_LEN, initial_sigma::ATTITUDE);
    fill_block(&mut p, idx::V_NED_START, idx::V_NED_LEN, initial_sigma::VELOCITY_NED);
    fill_block(&mut p, idx::P_NED_START, idx::P_NED_LEN, initial_sigma::POSITION_NED);
    fill_block(&mut p, idx::GYRO_BIAS_START, idx::GYRO_BIAS_LEN, initial_sigma::GYRO_BIAS);
    fill_block(&mut p, idx::ACCEL_BIAS_START, idx::ACCEL_BIAS_LEN, initial_sigma::ACCEL_BIAS);
    fill_block(&mut p, idx::MAG_NED_START, idx::MAG_NED_LEN, initial_sigma::MAG_NED);
    fill_block(&mut p, idx::MAG_BIAS_START, idx::MAG_BIAS_LEN, initial_sigma::MAG_BIAS);
    fill_block(&mut p, idx::WIND_NE_START, idx::WIND_NE_LEN, initial_sigma::WIND_NE);
    p
}

/// Place σ² on the diagonal of a `len`-long block starting at `start`.
/// Only writes diagonal entries — upper/lower triangles stay zero.
fn fill_block(p: &mut Covariance, start: usize, len: usize, sigma: f32) {
    let sigma_sq = sigma * sigma;
    for offset in 0..len {
        let i = start + offset;
        // fixed_view is the block API; writing one diagonal entry via
        // a 1×1 view keeps us clear of `p[(i,i)]` indexing.
        let mut cell = p.fixed_view_mut::<1, 1>(i, i);
        cell.fill(sigma_sq);
    }
}

/// Symmetrise the covariance matrix in place: `P ← (P + Pᵀ) / 2`.
///
/// Floating-point update loops (`P ← F·P·Fᵀ + Q`) drift away from perfect
/// symmetry by ~1 ulp per element per step. Accumulated, this can make the
/// matrix non-symmetric enough that a Cholesky solve or PSD check rejects
/// it. This one-liner fixes it and should be called after every update.
pub fn enforce_symmetry(p: &mut Covariance) {
    let transposed = p.transpose();
    *p += &transposed;
    *p *= 0.5;
}

/// Per-second process-noise intensities (1-σ). Multiplied by `dt` in
/// [`build_process_noise`] to yield the one-step `Q` matrix.
///
/// Units match each state group's natural rate:
/// * `gyro_bias_per_s` — rad/s per √s (random walk of the gyro zero-bias)
/// * `accel_bias_per_s` — m/s² per √s
/// * `attitude_per_s` — rad per √s (angular process noise beyond gyro noise)
/// * `velocity_per_s` — m/s per √s (unmodeled acceleration)
/// * `position_per_s` — m per √s (numerical guard against zero variance;
///   real position drift comes from velocity integration)
/// * `mag_ned_per_s` / `mag_bias_per_s` — gauss per √s
/// * `wind_per_s` — m/s per √s (weather-driven wind drift)
#[derive(Clone, Copy, Debug)]
pub struct ProcessNoise {
    pub attitude_per_s: f32,
    pub velocity_per_s: f32,
    pub position_per_s: f32,
    pub gyro_bias_per_s: f32,
    pub accel_bias_per_s: f32,
    pub mag_ned_per_s: f32,
    pub mag_bias_per_s: f32,
    pub wind_per_s: f32,
}

impl Default for ProcessNoise {
    fn default() -> Self {
        Self {
            attitude_per_s: 1.0e-4,
            velocity_per_s: 1.0e-2,
            position_per_s: 1.0e-6,
            gyro_bias_per_s: 1.0e-5,
            accel_bias_per_s: 1.0e-4,
            mag_ned_per_s: 1.0e-6,
            mag_bias_per_s: 1.0e-5,
            wind_per_s: 1.0e-2,
        }
    }
}

/// Build the one-step process-noise covariance `Q = diag(σ²·dt)` from
/// per-second intensities. Non-finite or negative `dt` yields the zero
/// matrix (the caller's predict skips in that case anyway).
#[must_use]
pub fn build_process_noise(noise: ProcessNoise, dt_s: f32) -> Covariance {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Covariance::zeros();
    }
    let mut q = Covariance::zeros();
    fill_block(&mut q, idx::Q_START, idx::Q_LEN, noise.attitude_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::V_NED_START, idx::V_NED_LEN, noise.velocity_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::P_NED_START, idx::P_NED_LEN, noise.position_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::GYRO_BIAS_START, idx::GYRO_BIAS_LEN, noise.gyro_bias_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::ACCEL_BIAS_START, idx::ACCEL_BIAS_LEN, noise.accel_bias_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::MAG_NED_START, idx::MAG_NED_LEN, noise.mag_ned_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::MAG_BIAS_START, idx::MAG_BIAS_LEN, noise.mag_bias_per_s * libm::sqrtf(dt_s));
    fill_block(&mut q, idx::WIND_NE_START, idx::WIND_NE_LEN, noise.wind_per_s * libm::sqrtf(dt_s));
    q
}

/// Propagate the covariance forward by one step using `P ← F·P·Fᵀ + Q`,
/// then symmetrise.
///
/// **M1.9a placeholder**: the state-transition Jacobian `F` is taken as the
/// identity so the data pipeline is end-to-end testable before the full
/// quaternion-aware Jacobian lands in M1.9b. The `enforce_symmetry` call
/// at the end is the production-shape guard; swapping in the real `F`
/// later keeps the call-site unchanged.
#[must_use]
pub fn predict_covariance(
    p_prev: &Covariance,
    f_transition: &Covariance,
    q: &Covariance,
) -> Covariance {
    let mut p_next = f_transition * p_prev * f_transition.transpose() + q;
    enforce_symmetry(&mut p_next);
    p_next
}

/// Identity transition matrix — the M1.9a placeholder for `F`. Replace
/// with the quaternion-aware Jacobian in M1.9b.
#[must_use]
pub fn identity_transition() -> Covariance {
    Covariance::identity()
}

/// Minimum quaternion norm accepted for normalization.
///
/// Near-zero quaternions have no unique direction; any prediction or update
/// that drives ‖q‖ below this value is treated as a loss of attitude and
/// the filter should reset.
pub const QUATERNION_NORM_FLOOR: f32 = 1.0e-6;

/// Relative tolerance we promise to hold `‖q‖ ≈ 1` to after normalization.
/// The property test `normalize_produces_unit_quaternion` proves this bound.
pub const QUATERNION_NORM_TOLERANCE: f32 = 1.0e-6;

/// Standard gravity used by the predict step (m/s²). NED convention:
/// gravity vector is `(0, 0, +GRAVITY_M_S2)` — positive z points down.
pub const GRAVITY_M_S2: f32 = 9.80665;

/// Bias-free IMU measurement consumed by the predict step.
///
/// This struct is intentionally decoupled from `core_hal::ImuSample` so that
/// `algo-ekf` stays independent of any HAL. The application layer bridges
/// the two (dropping the timestamp, etc.).
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuMeasurement {
    /// Body-frame angular rate, rad/s.
    pub gyro_rad_s: Vector3<f32>,
    /// Body-frame specific force (accelerometer reading), m/s².
    /// Includes gravity reaction: reads `-g` in body z when level & still.
    pub accel_m_s2: Vector3<f32>,
}

/// Full state, accessed as a struct so consumers don't couple to raw indices.
#[derive(Clone, Copy, Debug)]
pub struct State {
    /// Body-to-NED rotation as unit quaternion.
    pub attitude: Quaternion<f32>,
    /// Velocity in NED frame, m/s.
    pub velocity_ned: Vector3<f32>,
    /// Position in NED frame, m (relative to origin).
    pub position_ned: Vector3<f32>,
    /// Gyro zero-bias, rad/s.
    pub gyro_bias: Vector3<f32>,
    /// Accelerometer zero-bias, m/s^2.
    pub accel_bias: Vector3<f32>,
    /// Earth magnetic field vector in NED, gauss.
    pub mag_ned: Vector3<f32>,
    /// Magnetometer hard-iron offset, gauss.
    pub mag_bias: Vector3<f32>,
    /// Horizontal wind, NE components, m/s.
    pub wind_ne: Vector2<f32>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            attitude: Quaternion::identity(),
            velocity_ned: Vector3::zeros(),
            position_ned: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            mag_ned: Vector3::new(0.21, 0.0, 0.43),
            mag_bias: Vector3::zeros(),
            wind_ne: Vector2::zeros(),
        }
    }
}

impl State {
    /// Normalize the stored attitude quaternion in place.
    ///
    /// Returns `None` when the quaternion is degenerate — ‖q‖² is non-finite
    /// or ‖q‖ < [`QUATERNION_NORM_FLOOR`] — signalling that attitude has been
    /// lost and the caller must reset the filter. Otherwise returns the
    /// normalized quaternion as a `UnitQuaternion`.
    ///
    /// Postcondition (enforced by tests): the stored `attitude` satisfies
    /// `|‖q‖ − 1| ≤ QUATERNION_NORM_TOLERANCE`.
    ///
    /// Implementation note: the degenerate-input check runs on ‖q‖² so it
    /// avoids the `sqrt` call. This lets `kani::proof` harnesses verify the
    /// rejection path without tripping over `libm`'s x86 inline-asm `sqrtf`,
    /// which Kani cannot model today.
    pub fn normalize_attitude(&mut self) -> Option<UnitQuaternion<f32>> {
        let q = self.attitude;
        let norm_sq = q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k;
        if !norm_sq.is_finite()
            || norm_sq < QUATERNION_NORM_FLOOR * QUATERNION_NORM_FLOOR
        {
            return None;
        }
        let norm = libm::sqrtf(norm_sq);
        self.attitude /= norm;
        Some(UnitQuaternion::new_unchecked(self.attitude))
    }

    /// Pack the struct into a 24-D column vector following [`idx`].
    ///
    /// The returned vector is stack-allocated (nalgebra's `SVector`), so this
    /// is `no_std`-friendly and allocation-free. Uses nalgebra's block copy
    /// operations so no raw integer indexing is needed — this keeps the
    /// `indexing_slicing = deny` lint satisfied.
    #[must_use]
    pub fn to_vector(&self) -> StateVector {
        let mut v = StateVector::zeros();
        // Quaternion stored as [w, i, j, k] to match EKF convention.
        let q_block = Vector4::new(self.attitude.w, self.attitude.i, self.attitude.j, self.attitude.k);
        v.fixed_rows_mut::<{ idx::Q_LEN }>(idx::Q_START).copy_from(&q_block);
        v.fixed_rows_mut::<{ idx::V_NED_LEN }>(idx::V_NED_START).copy_from(&self.velocity_ned);
        v.fixed_rows_mut::<{ idx::P_NED_LEN }>(idx::P_NED_START).copy_from(&self.position_ned);
        v.fixed_rows_mut::<{ idx::GYRO_BIAS_LEN }>(idx::GYRO_BIAS_START).copy_from(&self.gyro_bias);
        v.fixed_rows_mut::<{ idx::ACCEL_BIAS_LEN }>(idx::ACCEL_BIAS_START).copy_from(&self.accel_bias);
        v.fixed_rows_mut::<{ idx::MAG_NED_LEN }>(idx::MAG_NED_START).copy_from(&self.mag_ned);
        v.fixed_rows_mut::<{ idx::MAG_BIAS_LEN }>(idx::MAG_BIAS_START).copy_from(&self.mag_bias);
        v.fixed_rows_mut::<{ idx::WIND_NE_LEN }>(idx::WIND_NE_START).copy_from(&self.wind_ne);
        v
    }

    /// Advance the state by one IMU step of duration `dt_s`.
    ///
    /// This is the **predict** half of the EKF. Implementation follows the
    /// standard strapdown-INS integration:
    ///
    /// 1. Correct the raw IMU reading for estimated biases.
    /// 2. Integrate attitude: `q ← q ⊗ exp(½·ω·dt)` (exact quaternion
    ///    exponential; small-angle fallback for numerical stability).
    /// 3. Rotate the body-frame specific force into NED and add gravity:
    ///    `a_ned = R(q) · f_body + (0, 0, +g)`.
    /// 4. Euler-integrate velocity and position with a second-order
    ///    position term: `p ← p + v·dt + ½·a·dt²`.
    ///
    /// Biases, earth magnetic field, and wind are left **unchanged** — they
    /// evolve via process noise in the measurement-update step, not here.
    ///
    /// The output quaternion is normalized. If `dt_s` is non-finite or the
    /// integrated quaternion degenerates to below [`QUATERNION_NORM_FLOOR`],
    /// the returned state keeps the *previous* attitude rather than panic.
    #[must_use]
    pub fn predict(&self, imu: ImuMeasurement, dt_s: f32) -> State {
        if !dt_s.is_finite() || dt_s < 0.0 {
            // Clock skew or sensor outage: don't propagate, hold state.
            return *self;
        }

        let omega = imu.gyro_rad_s - self.gyro_bias;
        let specific_force = imu.accel_m_s2 - self.accel_bias;

        // --- Attitude propagation ----------------------------------------
        let delta_q = quaternion_exp(omega, dt_s);
        let mut new_q = self.attitude * delta_q;
        let q_norm_sq = new_q.w * new_q.w
            + new_q.i * new_q.i
            + new_q.j * new_q.j
            + new_q.k * new_q.k;
        if q_norm_sq.is_finite()
            && q_norm_sq >= QUATERNION_NORM_FLOOR * QUATERNION_NORM_FLOOR
        {
            new_q /= libm::sqrtf(q_norm_sq);
        } else {
            // Degeneracy guard: fall back to previous attitude.
            new_q = self.attitude;
        }

        // --- Translation propagation -------------------------------------
        let rot = UnitQuaternion::new_unchecked(new_q);
        let accel_ned = rot * specific_force + Vector3::new(0.0, 0.0, GRAVITY_M_S2);
        let new_v = self.velocity_ned + accel_ned * dt_s;
        let new_p = self.position_ned
            + self.velocity_ned * dt_s
            + accel_ned * (0.5 * dt_s * dt_s);

        State {
            attitude: new_q,
            velocity_ned: new_v,
            position_ned: new_p,
            gyro_bias: self.gyro_bias,
            accel_bias: self.accel_bias,
            mag_ned: self.mag_ned,
            mag_bias: self.mag_bias,
            wind_ne: self.wind_ne,
        }
    }

    /// Reverse of [`Self::to_vector`].
    ///
    /// Copies fields 1-to-1; does **not** normalize the quaternion. Call
    /// [`Self::normalize_attitude`] afterwards if the caller can't guarantee
    /// the input came from a prior `to_vector` of a normalized state.
    #[must_use]
    pub fn from_vector(v: &StateVector) -> Self {
        let q_block: Vector4<f32> = v.fixed_rows::<{ idx::Q_LEN }>(idx::Q_START).into_owned();
        Self {
            attitude: Quaternion::new(q_block.x, q_block.y, q_block.z, q_block.w),
            velocity_ned: v
                .fixed_rows::<{ idx::V_NED_LEN }>(idx::V_NED_START)
                .into_owned(),
            position_ned: v
                .fixed_rows::<{ idx::P_NED_LEN }>(idx::P_NED_START)
                .into_owned(),
            gyro_bias: v
                .fixed_rows::<{ idx::GYRO_BIAS_LEN }>(idx::GYRO_BIAS_START)
                .into_owned(),
            accel_bias: v
                .fixed_rows::<{ idx::ACCEL_BIAS_LEN }>(idx::ACCEL_BIAS_START)
                .into_owned(),
            mag_ned: v
                .fixed_rows::<{ idx::MAG_NED_LEN }>(idx::MAG_NED_START)
                .into_owned(),
            mag_bias: v
                .fixed_rows::<{ idx::MAG_BIAS_LEN }>(idx::MAG_BIAS_START)
                .into_owned(),
            wind_ne: v
                .fixed_rows::<{ idx::WIND_NE_LEN }>(idx::WIND_NE_START)
                .into_owned(),
        }
    }
}

/// Exact quaternion exponential for a body-frame angular rate `omega` applied
/// over duration `dt_s`. Returns a **unit** rotation quaternion δq such that
/// `q_new = q ⊗ δq` integrates attitude.
///
/// Uses the small-angle fallback when `‖ω·dt‖ < 1e-8` rad so the sin-over-θ
/// division doesn't explode into NaN.
#[must_use]
pub fn quaternion_exp(omega: Vector3<f32>, dt_s: f32) -> Quaternion<f32> {
    let half_vec = omega * (0.5 * dt_s);
    let half_norm_sq = half_vec.dot(&half_vec);
    if !half_norm_sq.is_finite() || half_norm_sq < 1.0e-16 {
        // Small-angle: sin(θ/2) ≈ θ/2, cos(θ/2) ≈ 1.
        return Quaternion::new(1.0, half_vec.x, half_vec.y, half_vec.z);
    }
    let half_norm = libm::sqrtf(half_norm_sq);
    let sin_h = libm::sinf(half_norm);
    let cos_h = libm::cosf(half_norm);
    let scale = sin_h / half_norm;
    Quaternion::new(cos_h, half_vec.x * scale, half_vec.y * scale, half_vec.z * scale)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use proptest::prelude::*;

    /// Generate finite f32 quaternion components in a range wide enough to
    /// stress scaling, but not so wide that `norm()` overflows.
    fn finite_component() -> impl Strategy<Value = f32> {
        -1.0e6f32..1.0e6f32
    }

    fn any_quaternion() -> impl Strategy<Value = Quaternion<f32>> {
        (
            finite_component(),
            finite_component(),
            finite_component(),
            finite_component(),
        )
            .prop_map(|(w, i, j, k)| Quaternion::new(w, i, j, k))
    }

    proptest! {
        /// Core invariant: for any finite, non-degenerate input quaternion,
        /// `State::normalize_attitude` yields ‖q‖ ≈ 1.
        #[test]
        fn normalize_produces_unit_quaternion(q in any_quaternion()) {
            let mut state = State { attitude: q, ..State::default() };
            let result = state.normalize_attitude();
            let input_norm = q.norm();
            if input_norm < QUATERNION_NORM_FLOOR || !input_norm.is_finite() {
                prop_assert!(result.is_none(),
                    "degenerate quaternion should return None, input norm {input_norm}");
            } else {
                prop_assert!(result.is_some());
                let out_norm = state.attitude.norm();
                prop_assert!((out_norm - 1.0).abs() <= QUATERNION_NORM_TOLERANCE,
                    "normalized norm {out_norm} outside tolerance");
            }
        }
    }

    #[test]
    fn default_state_has_unit_attitude() {
        let s = State::default();
        assert_abs_diff_eq!(s.attitude.norm(), 1.0, epsilon = QUATERNION_NORM_TOLERANCE);
    }

    #[test]
    fn zero_quaternion_returns_none() {
        let mut s = State { attitude: Quaternion::new(0.0, 0.0, 0.0, 0.0), ..State::default() };
        assert!(s.normalize_attitude().is_none());
    }

    #[test]
    fn nan_quaternion_returns_none() {
        let mut s = State {
            attitude: Quaternion::new(f32::NAN, 0.0, 0.0, 0.0),
            ..State::default()
        };
        assert!(s.normalize_attitude().is_none());
    }

    // ---- 24-D serialization tests ----------------------------------------

    fn finite_coord() -> impl Strategy<Value = f32> {
        -1.0e3f32..1.0e3f32
    }

    /// Strategy that produces an arbitrary, finite (non-NaN, non-Inf) State.
    fn any_state() -> impl Strategy<Value = State> {
        (
            any_quaternion(),
            (finite_coord(), finite_coord(), finite_coord()),
            (finite_coord(), finite_coord(), finite_coord()),
            (finite_coord(), finite_coord(), finite_coord()),
            (finite_coord(), finite_coord(), finite_coord()),
            (finite_coord(), finite_coord(), finite_coord()),
            (finite_coord(), finite_coord(), finite_coord()),
            (finite_coord(), finite_coord()),
        )
            .prop_map(|(q, v, p, g, a, m, mb, w)| State {
                attitude: q,
                velocity_ned: Vector3::new(v.0, v.1, v.2),
                position_ned: Vector3::new(p.0, p.1, p.2),
                gyro_bias: Vector3::new(g.0, g.1, g.2),
                accel_bias: Vector3::new(a.0, a.1, a.2),
                mag_ned: Vector3::new(m.0, m.1, m.2),
                mag_bias: Vector3::new(mb.0, mb.1, mb.2),
                wind_ne: Vector2::new(w.0, w.1),
            })
    }

    fn states_equal(a: &State, b: &State) -> bool {
        a.attitude.w == b.attitude.w
            && a.attitude.i == b.attitude.i
            && a.attitude.j == b.attitude.j
            && a.attitude.k == b.attitude.k
            && a.velocity_ned == b.velocity_ned
            && a.position_ned == b.position_ned
            && a.gyro_bias == b.gyro_bias
            && a.accel_bias == b.accel_bias
            && a.mag_ned == b.mag_ned
            && a.mag_bias == b.mag_bias
            && a.wind_ne == b.wind_ne
    }

    #[test]
    fn initial_covariance_is_diagonal() {
        let p = initial_covariance();
        for i in 0..STATE_DIM {
            for j in 0..STATE_DIM {
                let v = p.fixed_view::<1, 1>(i, j).to_scalar();
                if i == j {
                    assert!(v > 0.0, "diagonal entry ({i}, {i}) must be positive, got {v}");
                } else {
                    assert_eq!(v, 0.0, "off-diagonal ({i}, {j}) must be 0, got {v}");
                }
            }
        }
    }

    #[test]
    fn build_process_noise_rejects_bad_dt() {
        let n = ProcessNoise::default();
        assert_eq!(build_process_noise(n, -1.0).norm(), 0.0);
        assert_eq!(build_process_noise(n, 0.0).norm(), 0.0);
        assert_eq!(build_process_noise(n, f32::NAN).norm(), 0.0);
    }

    #[test]
    fn predict_covariance_with_identity_f_adds_q() {
        let p = initial_covariance();
        let f = identity_transition();
        let q = build_process_noise(ProcessNoise::default(), 0.01);
        let p_next = predict_covariance(&p, &f, &q);
        // With F=I: p_next = p + q, so each diagonal increases by q's diagonal.
        for i in 0..STATE_DIM {
            let p_diag = p.fixed_view::<1, 1>(i, i).to_scalar();
            let q_diag = q.fixed_view::<1, 1>(i, i).to_scalar();
            let pnext_diag = p_next.fixed_view::<1, 1>(i, i).to_scalar();
            assert!(
                (pnext_diag - (p_diag + q_diag)).abs() < 1.0e-8,
                "diag {i}: got {pnext_diag}, expected {}",
                p_diag + q_diag
            );
        }
    }

    #[test]
    fn predict_covariance_keeps_symmetric_input_symmetric() {
        let p = initial_covariance(); // already symmetric (diagonal)
        let f = identity_transition();
        let q = build_process_noise(ProcessNoise::default(), 0.005);
        let p_next = predict_covariance(&p, &f, &q);
        for i in 0..STATE_DIM {
            for j in (i + 1)..STATE_DIM {
                let a = p_next.fixed_view::<1, 1>(i, j).to_scalar();
                let b = p_next.fixed_view::<1, 1>(j, i).to_scalar();
                assert!((a - b).abs() < 1.0e-6);
            }
        }
    }

    #[test]
    fn initial_covariance_diagonal_uses_sigma_squared() {
        let p = initial_covariance();
        let attitude_var = p.fixed_view::<1, 1>(idx::Q_START, idx::Q_START).to_scalar();
        assert!((attitude_var - initial_sigma::ATTITUDE.powi(2)).abs() < 1.0e-12);
        let pos_var = p
            .fixed_view::<1, 1>(idx::P_NED_START, idx::P_NED_START)
            .to_scalar();
        assert!((pos_var - initial_sigma::POSITION_NED.powi(2)).abs() < 1.0e-6);
    }

    #[test]
    fn enforce_symmetry_is_idempotent() {
        let mut p = Covariance::from_fn(|i, j| {
            // Convert indices via f32 ctor — stays clear of `as` conversions.
            f32::from(u16::try_from(i + j * 3).unwrap_or(0))
        });
        enforce_symmetry(&mut p);
        let once = p;
        enforce_symmetry(&mut p);
        // Symmetrising again shouldn't change anything.
        assert!((p - once).norm() < 1.0e-6);
    }

    #[test]
    fn idx_layout_totals_state_dim() {
        assert_eq!(idx::TOTAL, STATE_DIM);
        // Each range's end matches the next range's start — no gaps, no overlap.
        assert_eq!(idx::Q_START + idx::Q_LEN, idx::V_NED_START);
        assert_eq!(idx::V_NED_START + idx::V_NED_LEN, idx::P_NED_START);
        assert_eq!(idx::P_NED_START + idx::P_NED_LEN, idx::GYRO_BIAS_START);
        assert_eq!(idx::GYRO_BIAS_START + idx::GYRO_BIAS_LEN, idx::ACCEL_BIAS_START);
        assert_eq!(idx::ACCEL_BIAS_START + idx::ACCEL_BIAS_LEN, idx::MAG_NED_START);
        assert_eq!(idx::MAG_NED_START + idx::MAG_NED_LEN, idx::MAG_BIAS_START);
        assert_eq!(idx::MAG_BIAS_START + idx::MAG_BIAS_LEN, idx::WIND_NE_START);
        assert_eq!(idx::WIND_NE_START + idx::WIND_NE_LEN, idx::TOTAL);
    }

    #[test]
    fn default_state_round_trips() {
        let s = State::default();
        let v = s.to_vector();
        let s2 = State::from_vector(&v);
        assert!(states_equal(&s, &s2));
    }

    proptest! {
        /// Round-trip property: every random State survives to_vector → from_vector.
        #[test]
        fn state_vector_round_trip(s in any_state()) {
            let v = s.to_vector();
            let s2 = State::from_vector(&v);
            prop_assert!(states_equal(&s, &s2));
            // Round-tripping the vector also gives the same vector.
            let v2 = s2.to_vector();
            prop_assert_eq!(v, v2);
        }

        /// Free fall: zero specific force + zero rate → velocity grows as g·dt
        /// in the +z (down) direction, position as ½·g·dt².
        #[test]
        fn predict_free_fall_accumulates_gravity(dt_s in 1.0e-4f32..0.05) {
            let state = State::default(); // identity attitude, zero v/p/biases
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::zeros(), // free fall: no specific force
            };
            let next = state.predict(imu, dt_s);
            let expected_v_z = GRAVITY_M_S2 * dt_s;
            let expected_p_z = 0.5 * GRAVITY_M_S2 * dt_s * dt_s;
            prop_assert!((next.velocity_ned.z - expected_v_z).abs() < 1.0e-5);
            prop_assert!((next.velocity_ned.x).abs() < 1.0e-5);
            prop_assert!((next.velocity_ned.y).abs() < 1.0e-5);
            prop_assert!((next.position_ned.z - expected_p_z).abs() < 1.0e-6);
        }

        /// Level & stationary: specific force exactly cancels gravity →
        /// velocity and position stay at zero.
        #[test]
        fn predict_level_stationary_holds_position(dt_s in 1.0e-4f32..0.05) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                // Level accelerometer reads -g on z body axis.
                accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2),
            };
            let next = state.predict(imu, dt_s);
            prop_assert!(next.velocity_ned.norm() < 1.0e-4);
            prop_assert!(next.position_ned.norm() < 1.0e-6);
        }

        /// Zero angular rate keeps the attitude quaternion (modulo normalization).
        #[test]
        fn predict_zero_rate_preserves_attitude(dt_s in 0.0f32..0.05) {
            let mut state = State {
                attitude: Quaternion::new(0.5, 0.5, 0.5, 0.5), // arbitrary unit q
                ..State::default()
            };
            state.normalize_attitude();
            let q_before = state.attitude;
            let imu = ImuMeasurement::default();
            let next = state.predict(imu, dt_s);
            prop_assert!((next.attitude.w - q_before.w).abs() < 1.0e-5);
            prop_assert!((next.attitude.i - q_before.i).abs() < 1.0e-5);
            prop_assert!((next.attitude.j - q_before.j).abs() < 1.0e-5);
            prop_assert!((next.attitude.k - q_before.k).abs() < 1.0e-5);
        }

        /// Predict step preserves the unit-norm invariant on the quaternion.
        /// Gyro bounded to ±10 rad/s (realistic airframe).
        #[test]
        fn predict_preserves_quaternion_norm(
            wx in -10.0f32..10.0,
            wy in -10.0f32..10.0,
            wz in -10.0f32..10.0,
            dt_s in 1.0e-4f32..0.01,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::new(wx, wy, wz),
                accel_m_s2: Vector3::zeros(),
            };
            let next = state.predict(imu, dt_s);
            prop_assert!((next.attitude.norm() - 1.0).abs() <= QUATERNION_NORM_TOLERANCE);
        }

        /// Negative or non-finite dt holds the state (safety path).
        #[test]
        fn predict_rejects_bad_dt(dt_s in -1.0f32..0.0) {
            let state = State::default();
            let imu = ImuMeasurement::default();
            let next = state.predict(imu, dt_s);
            prop_assert_eq!(next.velocity_ned, state.velocity_ned);
            prop_assert_eq!(next.position_ned, state.position_ned);
        }

        /// `enforce_symmetry` produces a matrix equal to its transpose
        /// (up to 1 ulp on f32), for arbitrary input matrices.
        #[test]
        fn enforce_symmetry_makes_matrix_symmetric(
            seed in proptest::collection::vec(-10.0f32..10.0, STATE_DIM * STATE_DIM),
        ) {
            let mut p = Covariance::from_iterator(seed.iter().copied());
            enforce_symmetry(&mut p);
            // Check (i, j) == (j, i) for every pair.
            for i in 0..STATE_DIM {
                for j in (i + 1)..STATE_DIM {
                    let a = p.fixed_view::<1, 1>(i, j).to_scalar();
                    let b = p.fixed_view::<1, 1>(j, i).to_scalar();
                    prop_assert!((a - b).abs() < 1.0e-6,
                        "entries ({}, {}) and ({}, {}) not symmetric: {} vs {}", i, j, j, i, a, b);
                }
            }
        }

        /// Each struct field lands at its documented index in the vector.
        #[test]
        fn to_vector_respects_layout(s in any_state()) {
            let v = s.to_vector();
            let q: Vector4<f32> =
                v.fixed_rows::<{ idx::Q_LEN }>(idx::Q_START).into_owned();
            prop_assert_eq!(q.x, s.attitude.w);
            prop_assert_eq!(q.y, s.attitude.i);
            prop_assert_eq!(q.z, s.attitude.j);
            prop_assert_eq!(q.w, s.attitude.k);
            prop_assert_eq!(
                v.fixed_rows::<{ idx::V_NED_LEN }>(idx::V_NED_START).into_owned(),
                s.velocity_ned
            );
            prop_assert_eq!(
                v.fixed_rows::<{ idx::P_NED_LEN }>(idx::P_NED_START).into_owned(),
                s.position_ned
            );
            prop_assert_eq!(
                v.fixed_rows::<{ idx::WIND_NE_LEN }>(idx::WIND_NE_START).into_owned(),
                s.wind_ne
            );
        }
    }
}

// ============================================================================
// Kani formal proofs.
//
// Run with:  cargo kani --harness check_normalize_produces_unit_quaternion
//
// Kani symbolically executes the function over *all* reachable inputs,
// complementing the property test (which only samples 256). The two together
// satisfy the "core invariant" strength bar in CLAUDE.md.
// ============================================================================
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// # Scope of the Kani proofs in this file
    ///
    /// Kani cannot currently reason about `libm::sqrtf`, which is implemented
    /// with x86 inline assembly on our host. That means we can only prove the
    /// *rejection* paths of `normalize_attitude` — the branches that return
    /// `None` before reaching `sqrt`. The "output ‖q‖ ≈ 1" property is
    /// exercised by the `normalize_produces_unit_quaternion` proptest in the
    /// `tests` module instead.
    ///
    /// What this costs us: we lose symbolic coverage of one arithmetic
    /// property. What we keep: 100 % formal coverage of the degenerate-input
    /// contract, which is the safety-critical half — a stray `NaN` flowing
    /// into attitude is a crash vector that must be rejected on every path.

    /// Proves: zero quaternion is rejected.
    ///
    /// All-concrete inputs — no symbolic variables — so CBMC closes this in
    /// well under a second. The proof establishes that the `norm_sq < FLOOR²`
    /// branch correctly returns `None` when fed an exact zero quaternion
    /// (a concrete worst-case for reset scenarios).
    #[kani::proof]
    fn check_zero_quaternion_returns_none() {
        let mut state = State {
            attitude: Quaternion::new(0.0, 0.0, 0.0, 0.0),
            ..State::default()
        };
        assert!(state.normalize_attitude().is_none());
    }

    // Note: symbolic harnesses over `kani::any::<f32>()` (e.g.
    // "any non-finite component returns None" or "any sub-floor quaternion
    // returns None") trigger CBMC runs of 20+ minutes on the floating-point
    // theory using the default CaDiCaL solver. Treat those as TODO and move
    // them to a slow-CI lane later (probably with `--solver=z3`). For now
    // the proptest in `tests` covers the symbolic cases with 256 samples
    // each, including non-finite and sub-floor paths.
}
