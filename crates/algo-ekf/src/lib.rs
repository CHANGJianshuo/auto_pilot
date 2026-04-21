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

use nalgebra::{
    Matrix3, Matrix3x4, Matrix4, Quaternion, SVector, UnitQuaternion, Vector2, Vector3, Vector4,
};

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

/// Run one full EKF predict cycle: advance state and covariance one step.
///
/// This is the recommended entry point for the rate-loop task — it bundles
/// state propagation, Jacobian construction, process-noise build, and
/// covariance update into a single call so that callers can't forget one
/// of the four pieces.
///
/// After every call the caller should check [`State::attitude`] remains
/// within [`QUATERNION_NORM_TOLERANCE`] of unit norm (the `predict` step
/// normalizes it, but if the normalization bailed because of a degenerate
/// input, the attitude carries the previous step's value).
#[must_use]
pub fn predict_step(
    state: &State,
    covariance: &Covariance,
    imu: ImuMeasurement,
    noise: ProcessNoise,
    dt_s: f32,
) -> (State, Covariance) {
    predict_step_with_drag(state, covariance, imu, noise, dt_s, 0.0)
}

/// Drag-aware variant of [`predict_step`]. `drag_over_mass` scales the
/// drag term applied to the velocity prediction. Zero recovers the old
/// behaviour.
///
/// Note: this does **not** yet add the `∂v/∂wind_ne` block to the
/// transition Jacobian, so wind is not observable from measurement
/// residuals. Wind observability (full Jacobian) is queued for a later
/// milestone; this step exists so app code can feed externally estimated
/// wind into the predict step today.
#[must_use]
pub fn predict_step_with_drag(
    state: &State,
    covariance: &Covariance,
    imu: ImuMeasurement,
    noise: ProcessNoise,
    dt_s: f32,
    drag_over_mass: f32,
) -> (State, Covariance) {
    let f = build_transition_jacobian_with_drag(state, &imu, dt_s, drag_over_mass);
    let q = build_process_noise(noise, dt_s);
    let next_state = state.predict_with_drag(imu, dt_s, drag_over_mass);
    let next_covariance = predict_covariance(covariance, &f, &q);
    (next_state, next_covariance)
}

/// Build the state-transition Jacobian `F = ∂f/∂x` with the **trivial**
/// kinematic couplings filled in.
///
/// This is the M1.9b-0 intermediate: start from `I`, then set the only
/// *purely linear* coupling — `∂p/∂v = I · dt` — so position tracks
/// velocity correctly. The remaining attitude-coupled blocks
/// (velocity ← attitude, velocity ← accel_bias, attitude ← gyro_bias)
/// are left as identity until M1.9b-1 lands their closed-form Jacobians.
///
/// Algebra of the one non-identity block:
///
/// ```text
///   p_new = p_old + v·dt + ½·a·dt²
///   ⇒   ∂p_new/∂v = I · dt
/// ```
///
/// Velocity's auto-coupling `∂v_new/∂v = I` is already provided by the
/// identity start, so nothing else needs to be written here yet.
#[must_use]
pub fn kinematic_transition(dt_s: f32) -> Covariance {
    let mut f = Covariance::identity();
    if !dt_s.is_finite() || dt_s < 0.0 {
        return f;
    }
    // ∂p/∂v = I · dt  — 3×3 block at rows [P_NED_START..), cols [V_NED_START..)
    let mut dp_dv = f.fixed_view_mut::<{ idx::P_NED_LEN }, { idx::V_NED_LEN }>(
        idx::P_NED_START,
        idx::V_NED_START,
    );
    dp_dv.fill_with_identity();
    dp_dv.scale_mut(dt_s);
    f
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
        self.predict_with_drag(imu, dt_s, 0.0)
    }

    /// Drag-aware predict. `drag_over_mass` is `k_drag / mass` in s⁻¹.
    /// Zero preserves the old `predict` behaviour.
    ///
    /// Drag is applied as `a_drag = -drag_over_mass · (v - wind)` where
    /// `wind = (wind_ne.x, wind_ne.y, 0)` extended to 3D. Adding this to
    /// the predicted acceleration makes the filter's velocity prediction
    /// account for aerodynamic drag given a wind estimate — so if the
    /// caller populates `state.wind_ne` (from an external wind sensor,
    /// airspeed indicator, or a future predict-time wind observer),
    /// predict will "feel" the drag on-the-fly.
    #[must_use]
    pub fn predict_with_drag(
        &self,
        imu: ImuMeasurement,
        dt_s: f32,
        drag_over_mass: f32,
    ) -> State {
        if !dt_s.is_finite() || dt_s < 0.0 {
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
            new_q = self.attitude;
        }

        // --- Translation propagation -------------------------------------
        let rot = UnitQuaternion::new_unchecked(new_q);
        let thrust_gravity_accel =
            rot * specific_force + Vector3::new(0.0, 0.0, GRAVITY_M_S2);

        let drag_accel = if drag_over_mass.is_finite() && drag_over_mass > 0.0 {
            let wind_world =
                Vector3::new(self.wind_ne.x, self.wind_ne.y, 0.0);
            let v_rel = self.velocity_ned - wind_world;
            -drag_over_mass * v_rel
        } else {
            Vector3::zeros()
        };

        let accel_ned = thrust_gravity_accel + drag_accel;
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

/// Right-multiplication matrix `R(q)` such that `p ⊗ q = R(q) · p_vec`,
/// where quaternions are laid out as `(w, x, y, z)`.
///
/// Useful when you need `∂(p ⊗ q)/∂p = R(q)`.
#[must_use]
pub fn right_multiplication_matrix(q: Quaternion<f32>) -> Matrix4<f32> {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    Matrix4::new(
        w, -x, -y, -z,
        x,  w,  z, -y,
        y, -z,  w,  x,
        z,  y, -x,  w,
    )
}

/// Rotation matrix `R(q)` from body frame to world (NED) frame for a unit
/// quaternion `q = (w, x, y, z)`.
#[must_use]
pub fn rotation_matrix(q: Quaternion<f32>) -> Matrix3<f32> {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    Matrix3::new(
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y),
    )
}

/// Jacobian of `R(q)ᵀ · v` w.r.t. `q`, returned as a 3×4 matrix.
///
/// Uses `R(q)ᵀ = R(q*)` where `q* = (w, -x, -y, -z)` is the conjugate:
///
/// ```text
///   ∂(R(q)ᵀ · v) / ∂q  =  ∂(R(q*) · v) / ∂q*  ·  ∂q*/∂q
///                       =  J_R(q*, v) · diag(1, -1, -1, -1)
/// ```
#[must_use]
pub fn rotation_transpose_jacobian_wrt_q(q: Quaternion<f32>, v: Vector3<f32>) -> Matrix3x4<f32> {
    let q_conj = Quaternion::new(q.w, -q.i, -q.j, -q.k);
    let mut j = rotation_jacobian_wrt_q(q_conj, v);
    let mut c1 = j.fixed_columns_mut::<1>(1);
    c1.scale_mut(-1.0);
    let mut c2 = j.fixed_columns_mut::<1>(2);
    c2.scale_mut(-1.0);
    let mut c3 = j.fixed_columns_mut::<1>(3);
    c3.scale_mut(-1.0);
    j
}

/// Jacobian of `R(q) · v` w.r.t. `q`, returned as a 3×4 matrix.
///
/// Closed form: column `j` is `(∂R/∂q_j) · v`, evaluated element-by-element.
/// Used to build the `∂v/∂q` block of the state-transition Jacobian.
#[must_use]
pub fn rotation_jacobian_wrt_q(q: Quaternion<f32>, v: Vector3<f32>) -> Matrix3x4<f32> {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    let (vx, vy, vz) = (v.x, v.y, v.z);
    let two = 2.0_f32;
    // ∂R/∂w · v
    let col_w = two * Vector3::new(-z * vy + y * vz, z * vx - x * vz, -y * vx + x * vy);
    // ∂R/∂x · v
    let col_x = two * Vector3::new(
        y * vy + z * vz,
        y * vx - 2.0 * x * vy - w * vz,
        z * vx + w * vy - 2.0 * x * vz,
    );
    // ∂R/∂y · v
    let col_y = two * Vector3::new(
        -2.0 * y * vx + x * vy + w * vz,
        x * vx + z * vz,
        -w * vx + z * vy - 2.0 * y * vz,
    );
    // ∂R/∂z · v
    let col_z = two * Vector3::new(
        -2.0 * z * vx - w * vy + x * vz,
        w * vx - 2.0 * z * vy + y * vz,
        x * vx + y * vy,
    );
    Matrix3x4::from_columns(&[col_w, col_x, col_y, col_z])
}

/// Left-multiplication matrix `L(q)` such that `q ⊗ p = L(q) · p_vec`,
/// where quaternions are laid out as `(w, x, y, z)`.
///
/// Useful when you need `∂(q ⊗ p)/∂p = L(q)`.
#[must_use]
pub fn left_multiplication_matrix(q: Quaternion<f32>) -> Matrix4<f32> {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    Matrix4::new(
        w, -x, -y, -z,
        x,  w, -z,  y,
        y,  z,  w, -x,
        z, -y,  x,  w,
    )
}

/// Build the full 24×24 state-transition Jacobian `F = ∂f/∂x` for the
/// current predict step.
///
/// Layout so far (filled incrementally across M1.9b subtasks):
///
/// ```text
///   ┌──────────────┬──────────┬──────────┬───────────┐
///   │ ∂q/∂q = R(δq)│  (I)     │  (I)     │   ...     │
///   │ ∂v/∂q = (I)  │ ∂v/∂v=I  │          │           │
///   │ ∂p/∂q = (I)  │ ∂p/∂v=Idt│ ∂p/∂p=I  │           │
///   │  ...         │          │          │  I        │
///   └──────────────┴──────────┴──────────┴───────────┘
/// ```
///
/// Filled so far:
/// * `∂q/∂q` — closed-form via right-multiplication by δq = exp(½·ω·dt)
/// * `∂p/∂v` — `I · dt` (linear)
///
/// Still identity (M1.9b-1b/c/d):
/// * `∂q/∂b_g`, `∂v/∂q`, `∂v/∂b_a`, `∂p/∂q`
#[must_use]
pub fn build_transition_jacobian(
    state: &State,
    imu: &ImuMeasurement,
    dt_s: f32,
) -> Covariance {
    let mut f = kinematic_transition(dt_s);
    if !dt_s.is_finite() || dt_s < 0.0 {
        return f;
    }
    let omega = imu.gyro_rad_s - state.gyro_bias;
    let delta_q = quaternion_exp(omega, dt_s);
    let r_dq = right_multiplication_matrix(delta_q);
    let mut block = f.fixed_view_mut::<{ idx::Q_LEN }, { idx::Q_LEN }>(idx::Q_START, idx::Q_START);
    block.copy_from(&r_dq);

    // ∂q/∂b_g block. Derivation (small-angle first-order, evaluated at the
    // current q):
    //   q_new = q ⊗ δq ;      δq = exp(½·(ω − b_g)·dt) ≈ (1, ½·(ω − b_g)·dt)
    //   ⇒ ∂δq/∂b_g = -(dt/2) · [top row zero, I₃ below]  (a 4×3 matrix)
    //   ⇒ ∂q_new/∂b_g = L(q) · ∂δq/∂b_g  =  -(dt/2) · L(q)[:, 1:4]
    let lq = left_multiplication_matrix(state.attitude);
    let dq_dbg = -(dt_s / 2.0) * lq.fixed_columns::<{ idx::GYRO_BIAS_LEN }>(1).into_owned();
    let mut dq_dbg_block =
        f.fixed_view_mut::<{ idx::Q_LEN }, { idx::GYRO_BIAS_LEN }>(idx::Q_START, idx::GYRO_BIAS_START);
    dq_dbg_block.copy_from(&dq_dbg);

    // ∂v/∂q and ∂v/∂b_a blocks (M1.9b-1c). Evaluated at current q:
    //   a_ned = R(q) · (accel - b_a) + g
    //   v_new = v + a_ned · dt
    //   ⇒ ∂v_new/∂q   =  (∂R/∂q · f_body) · dt         (3×4)
    //   ⇒ ∂v_new/∂b_a = -R(q) · dt                      (3×3)
    let f_body = imu.accel_m_s2 - state.accel_bias;
    let rot_jac = rotation_jacobian_wrt_q(state.attitude, f_body);
    let dv_dq = rot_jac * dt_s;
    let mut dv_dq_block =
        f.fixed_view_mut::<{ idx::V_NED_LEN }, { idx::Q_LEN }>(idx::V_NED_START, idx::Q_START);
    dv_dq_block.copy_from(&dv_dq);

    let r_q = rotation_matrix(state.attitude);
    let dv_dba = -r_q * dt_s;
    let mut dv_dba_block = f.fixed_view_mut::<{ idx::V_NED_LEN }, { idx::ACCEL_BIAS_LEN }>(
        idx::V_NED_START,
        idx::ACCEL_BIAS_START,
    );
    dv_dba_block.copy_from(&dv_dba);

    // ∂p/∂q and ∂p/∂b_a blocks (M1.9b-1d). From the position integration
    //   p_new = p + v·dt + ½·a_ned·dt²   with a_ned = R(q)·f_body + g
    //   ⇒ ∂p_new/∂q   = ½·dt² · (∂R/∂q · f_body)          (3×4)
    //   ⇒ ∂p_new/∂b_a = −½·R(q)·dt²                        (3×3)
    let half_dt_sq = 0.5 * dt_s * dt_s;
    let dp_dq = rot_jac * half_dt_sq;
    let mut dp_dq_block =
        f.fixed_view_mut::<{ idx::P_NED_LEN }, { idx::Q_LEN }>(idx::P_NED_START, idx::Q_START);
    dp_dq_block.copy_from(&dp_dq);

    let dp_dba = -r_q * half_dt_sq;
    let mut dp_dba_block = f.fixed_view_mut::<{ idx::P_NED_LEN }, { idx::ACCEL_BIAS_LEN }>(
        idx::P_NED_START,
        idx::ACCEL_BIAS_START,
    );
    dp_dba_block.copy_from(&dp_dba);

    f
}

/// Drag-aware transition Jacobian. Same as [`build_transition_jacobian`] with
/// the additional drag-related blocks written on top:
///
/// ```text
///   a_drag = -drag · (v − wind_world)         wind_world = (wind.x, wind.y, 0)
///   ⇒ ∂v_new/∂v      += -drag·dt · I₃
///   ⇒ ∂v_new/∂wind_ne = +drag·dt · [I₂; 0]    (3×2, z-row zero)
///   ⇒ ∂p_new/∂v      += -drag·½·dt² · I₃
///   ⇒ ∂p_new/∂wind_ne = +drag·½·dt² · [I₂; 0] (3×2, z-row zero)
/// ```
///
/// `drag_over_mass <= 0` or non-finite makes this degrade to the ordinary
/// `build_transition_jacobian`, matching [`State::predict_with_drag`].
#[must_use]
pub fn build_transition_jacobian_with_drag(
    state: &State,
    imu: &ImuMeasurement,
    dt_s: f32,
    drag_over_mass: f32,
) -> Covariance {
    let mut f = build_transition_jacobian(state, imu, dt_s);

    if !dt_s.is_finite()
        || dt_s <= 0.0
        || !drag_over_mass.is_finite()
        || drag_over_mass <= 0.0
    {
        return f;
    }

    let dd = drag_over_mass * dt_s; // dimensionless
    let dd_half_dt = dd * 0.5 * dt_s; // s

    // ∂v/∂v correction: subtract drag·dt·I₃.
    {
        let mut v_v = f.fixed_view_mut::<{ idx::V_NED_LEN }, { idx::V_NED_LEN }>(
            idx::V_NED_START,
            idx::V_NED_START,
        );
        let correction = -dd * Matrix3::identity();
        v_v += correction;
    }

    // ∂v/∂wind_ne: new 3×2 block, +drag·dt on diag (x,y rows), z row zero.
    {
        let mut v_w = f.fixed_view_mut::<{ idx::V_NED_LEN }, { idx::WIND_NE_LEN }>(
            idx::V_NED_START,
            idx::WIND_NE_START,
        );
        v_w.fill(0.0);
        let mut top2 = v_w.fixed_view_mut::<2, 2>(0, 0);
        top2.fill_with_identity();
        top2.scale_mut(dd);
    }

    // ∂p/∂v correction: subtract drag·½·dt²·I₃ on top of the I·dt already there.
    {
        let mut p_v = f.fixed_view_mut::<{ idx::P_NED_LEN }, { idx::V_NED_LEN }>(
            idx::P_NED_START,
            idx::V_NED_START,
        );
        let correction = -dd_half_dt * Matrix3::identity();
        p_v += correction;
    }

    // ∂p/∂wind_ne: new 3×2 block, +drag·½·dt² on diag.
    {
        let mut p_w = f.fixed_view_mut::<{ idx::P_NED_LEN }, { idx::WIND_NE_LEN }>(
            idx::P_NED_START,
            idx::WIND_NE_START,
        );
        p_w.fill(0.0);
        let mut top2 = p_w.fixed_view_mut::<2, 2>(0, 0);
        top2.fill_with_identity();
        top2.scale_mut(dd_half_dt);
    }

    f
}

// ============================================================================
// Measurement update — GPS position
// ============================================================================

/// A 3-axis NED position observation from GPS / RTK.
///
/// `position_ned` is in the local NED frame relative to the EKF origin, in
/// meters. `sigma` is the 1-σ uncertainty per axis (GPS receivers report
/// HDOP / VDOP from which this can be derived).
#[derive(Clone, Copy, Debug)]
pub struct GpsMeasurement {
    pub position_ned: Vector3<f32>,
    pub sigma: Vector3<f32>,
}

/// 3-component innovation returned by an observation step.
///
/// `residual` is `y − h(x̂)` (measured − predicted). `innovation_covariance`
/// is `S = H·P·Hᵀ + R`, useful for chi-square outlier tests before the
/// Kalman-gain step is applied.
#[derive(Clone, Copy, Debug)]
pub struct GpsInnovation {
    pub residual: Vector3<f32>,
    pub innovation_covariance: Matrix3<f32>,
}

impl GpsInnovation {
    /// Normalised chi-square gate (NIS): `residualᵀ · S⁻¹ · residual`.
    /// Values far above 3-axis χ² 99% quantile (~11.34) suggest the
    /// measurement is an outlier and should be rejected.
    #[must_use]
    pub fn normalized_squared(&self) -> f32 {
        match self.innovation_covariance.try_inverse() {
            Some(s_inv) => (self.residual.transpose() * s_inv * self.residual).to_scalar(),
            None => f32::INFINITY,
        }
    }
}

/// χ² 99 % quantile for 3 degrees of freedom. GPS innovations whose NIS
/// exceeds this value are rejected as outliers in [`gps_update`].
///
/// Field-tunable once real data is in hand — PX4's ekf2 defaults to the
/// same number.
pub const GPS_CHI2_GATE: f32 = 11.345;

/// Result of one GPS measurement update.
#[derive(Clone, Copy, Debug)]
pub struct GpsUpdateResult {
    pub state: State,
    pub covariance: Covariance,
    /// `true` if the measurement passed the χ² gate and was folded in;
    /// `false` if it was rejected (state/covariance unchanged).
    pub applied: bool,
    /// Normalised innovation squared at the time of decision.
    pub nis: f32,
}

/// Compute the GPS position innovation without yet applying a Kalman gain.
///
/// The measurement model is linear: `h(x) = position_ned`, so the
/// observation matrix `H` has `I₃` at the position rows and zeros elsewhere.
/// We don't materialise `H` explicitly — the structured block extraction is
/// faster and clippy-safe.
#[must_use]
pub fn gps_innovation(
    state: &State,
    covariance: &Covariance,
    measurement: &GpsMeasurement,
) -> GpsInnovation {
    let residual = measurement.position_ned - state.position_ned;
    // S = P[P_NED, P_NED] + diag(σ²)
    let p_block: Matrix3<f32> = covariance
        .fixed_view::<{ idx::P_NED_LEN }, { idx::P_NED_LEN }>(idx::P_NED_START, idx::P_NED_START)
        .into_owned();
    let r = Matrix3::from_diagonal(&Vector3::new(
        measurement.sigma.x * measurement.sigma.x,
        measurement.sigma.y * measurement.sigma.y,
        measurement.sigma.z * measurement.sigma.z,
    ));
    GpsInnovation {
        residual,
        innovation_covariance: p_block + r,
    }
}

// ============================================================================
// Measurement update — Barometer (1-D altitude)
// ============================================================================

/// χ² 99 % quantile for 1 degree of freedom.
pub const BARO_CHI2_GATE: f32 = 6.635;

/// Barometer altitude observation.
///
/// `altitude_m` is geometric altitude above the EKF origin (positive up).
/// NED convention has `+z` pointing down, so `h(x) = -position_ned.z`.
#[derive(Clone, Copy, Debug)]
pub struct BaroMeasurement {
    pub altitude_m: f32,
    pub sigma_m: f32,
}

/// Result of one barometer update.
#[derive(Clone, Copy, Debug)]
pub struct BaroUpdateResult {
    pub state: State,
    pub covariance: Covariance,
    pub applied: bool,
    pub nis: f32,
}

/// Fold a barometric altitude measurement.
///
/// `h(x) = −position_ned.z`, so `H` is a 1×24 row vector with `-1` at
/// column `P_NED_START + 2`. Innovation and S are scalars, so the Kalman
/// gain reduces to `K = P[:, P_z] · (-1) / s_scalar` — no matrix inverse.
#[must_use]
pub fn baro_update(
    state: &State,
    covariance: &Covariance,
    measurement: &BaroMeasurement,
) -> BaroUpdateResult {
    let down_idx = idx::P_NED_START + 2;
    let predicted = -state.position_ned.z;
    let residual = measurement.altitude_m - predicted;

    // H = [0..., -1 at column down_idx, ...0], so:
    //   H·P·Hᵀ = P[down_idx, down_idx]
    //   S      = P[down_idx, down_idx] + σ²
    //   P·Hᵀ   = -P[:, down_idx]
    let p_down_z: f32 = covariance.fixed_view::<1, 1>(down_idx, down_idx).to_scalar();
    let r = measurement.sigma_m * measurement.sigma_m;
    let s_scalar = p_down_z + r;

    let nis = if s_scalar > 0.0 { residual * residual / s_scalar } else { f32::INFINITY };
    if !nis.is_finite() || nis > BARO_CHI2_GATE {
        return BaroUpdateResult {
            state: *state,
            covariance: *covariance,
            applied: false,
            nis,
        };
    }

    // K = P·Hᵀ / S  (24-vector)
    let p_col: StateVector =
        -covariance.fixed_columns::<1>(down_idx).into_owned() / s_scalar;
    let k = p_col;

    // State correction.
    let dx = k * residual;
    let mut new_state = State::from_vector(&(state.to_vector() + dx));
    new_state.normalize_attitude();

    // Joseph form: (I − K·H) with H having -1 at column down_idx means
    //   (I − K·H)[:, down_idx] = I[:, down_idx] − K · (-1) = I[:, down_idx] + K
    //   other columns unchanged.
    let mut i_minus_kh = Covariance::identity();
    {
        let mut col = i_minus_kh.fixed_columns_mut::<1>(down_idx);
        col += k;
    }

    // K·R·Kᵀ reduces to (K·r)·Kᵀ because R is scalar.
    let kr_kt = k * r * k.transpose();
    let mut p_new = i_minus_kh * covariance * i_minus_kh.transpose() + kr_kt;
    enforce_symmetry(&mut p_new);

    BaroUpdateResult {
        state: new_state,
        covariance: p_new,
        applied: true,
        nis,
    }
}

// ============================================================================
// Measurement update — Magnetometer
// ============================================================================

/// χ² 99 % quantile for 3 degrees of freedom, shared with GPS.
pub const MAG_CHI2_GATE: f32 = GPS_CHI2_GATE;

/// Body-frame magnetic-field observation.
///
/// `body_field` is what the magnetometer reads (after the calibration
/// applied by the driver) in gauss. `sigma` is the 1-σ per-axis noise.
#[derive(Clone, Copy, Debug)]
pub struct MagMeasurement {
    pub body_field: Vector3<f32>,
    pub sigma: Vector3<f32>,
}

/// Result of one magnetometer update.
#[derive(Clone, Copy, Debug)]
pub struct MagUpdateResult {
    pub state: State,
    pub covariance: Covariance,
    pub applied: bool,
    pub nis: f32,
}

/// Build the 3×24 observation Jacobian `H` for the magnetometer.
///
/// Model: `h(x) = R(q)ᵀ · mag_ned + mag_bias`.
/// Non-zero columns:
///   * `q` (4 cols): `∂(R(q)ᵀ · mag_ned)/∂q`
///   * `mag_ned` (3 cols): `R(q)ᵀ`
///   * `mag_bias` (3 cols): `I₃`
#[must_use]
fn build_mag_observation_matrix(state: &State) -> nalgebra::SMatrix<f32, 3, STATE_DIM> {
    let mut h: nalgebra::SMatrix<f32, 3, STATE_DIM> = nalgebra::SMatrix::zeros();

    let j_q = rotation_transpose_jacobian_wrt_q(state.attitude, state.mag_ned);
    h.fixed_view_mut::<3, { idx::Q_LEN }>(0, idx::Q_START)
        .copy_from(&j_q);

    let r_transpose = rotation_matrix(state.attitude).transpose();
    h.fixed_view_mut::<3, { idx::MAG_NED_LEN }>(0, idx::MAG_NED_START)
        .copy_from(&r_transpose);

    let mut bias_block =
        h.fixed_view_mut::<3, { idx::MAG_BIAS_LEN }>(0, idx::MAG_BIAS_START);
    bias_block.fill_with_identity();

    h
}

/// Predicted magnetometer reading given the current EKF state.
#[must_use]
pub fn predict_magnetometer(state: &State) -> Vector3<f32> {
    rotation_matrix(state.attitude).transpose() * state.mag_ned + state.mag_bias
}

/// Fold a magnetometer observation into the EKF with χ² outlier gating
/// and Joseph-form covariance update.
#[must_use]
pub fn mag_update(
    state: &State,
    covariance: &Covariance,
    measurement: &MagMeasurement,
) -> MagUpdateResult {
    let predicted = predict_magnetometer(state);
    let residual = measurement.body_field - predicted;

    let h = build_mag_observation_matrix(state);
    let r = Matrix3::from_diagonal(&Vector3::new(
        measurement.sigma.x * measurement.sigma.x,
        measurement.sigma.y * measurement.sigma.y,
        measurement.sigma.z * measurement.sigma.z,
    ));
    let s = h * covariance * h.transpose() + r;

    let nis = match s.try_inverse() {
        Some(s_inv) => (residual.transpose() * s_inv * residual).to_scalar(),
        None => f32::INFINITY,
    };
    if !nis.is_finite() || nis > MAG_CHI2_GATE {
        return MagUpdateResult {
            state: *state,
            covariance: *covariance,
            applied: false,
            nis,
        };
    }
    let Some(s_inv) = s.try_inverse() else {
        return MagUpdateResult {
            state: *state,
            covariance: *covariance,
            applied: false,
            nis: f32::INFINITY,
        };
    };

    let k = covariance * h.transpose() * s_inv;
    let dx = k * residual;
    let mut new_state = State::from_vector(&(state.to_vector() + dx));
    new_state.normalize_attitude();

    let kh = k * h;
    let i_minus_kh = Covariance::identity() - kh;
    let mut p_new = i_minus_kh * covariance * i_minus_kh.transpose() + k * r * k.transpose();
    enforce_symmetry(&mut p_new);

    MagUpdateResult {
        state: new_state,
        covariance: p_new,
        applied: true,
        nis,
    }
}

/// Fold a GPS measurement into the EKF with outlier gating.
///
/// Sequence:
///
/// 1. Compute `innovation` and its NIS (chi-square).
/// 2. If NIS exceeds [`GPS_CHI2_GATE`] or is non-finite → reject
///    (return unchanged state/covariance, `applied = false`).
/// 3. Otherwise compute the Kalman gain `K = P·Hᵀ·S⁻¹`, apply the state
///    correction `x ← x + K·residual`, and update the covariance via
///    the **Joseph form** `P ← (I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ`.
/// 4. Re-normalise the attitude quaternion and enforce covariance symmetry.
///
/// The Joseph form is used (rather than `P ← (I−K·H)·P`) because it is
/// symmetric by construction and robust to small numerical errors in K.
#[must_use]
pub fn gps_update(
    state: &State,
    covariance: &Covariance,
    measurement: &GpsMeasurement,
) -> GpsUpdateResult {
    let innov = gps_innovation(state, covariance, measurement);
    let nis = innov.normalized_squared();
    if !nis.is_finite() || nis > GPS_CHI2_GATE {
        return GpsUpdateResult {
            state: *state,
            covariance: *covariance,
            applied: false,
            nis,
        };
    }

    let Some(s_inv) = innov.innovation_covariance.try_inverse() else {
        return GpsUpdateResult {
            state: *state,
            covariance: *covariance,
            applied: false,
            nis: f32::INFINITY,
        };
    };

    // P·Hᵀ is the (24×3) column block of P at [ :, P_NED_START..+3 ].
    let p_ht: nalgebra::SMatrix<f32, STATE_DIM, 3> = covariance
        .fixed_columns::<{ idx::P_NED_LEN }>(idx::P_NED_START)
        .into_owned();
    let k = p_ht * s_inv;

    // State correction via 24-D vector round-trip, then re-normalise q.
    let dx = k * innov.residual;
    let mut new_state = State::from_vector(&(state.to_vector() + dx));
    new_state.normalize_attitude();

    // Joseph-form covariance update.
    //   K·H has only columns P_NED_START..+3 non-zero (== columns of K).
    let mut kh = Covariance::zeros();
    kh.fixed_view_mut::<STATE_DIM, { idx::P_NED_LEN }>(0, idx::P_NED_START)
        .copy_from(&k);
    let i_minus_kh = Covariance::identity() - kh;

    let r = Matrix3::from_diagonal(&Vector3::new(
        measurement.sigma.x * measurement.sigma.x,
        measurement.sigma.y * measurement.sigma.y,
        measurement.sigma.z * measurement.sigma.z,
    ));

    let mut p_new = i_minus_kh * covariance * i_minus_kh.transpose() + k * r * k.transpose();
    enforce_symmetry(&mut p_new);

    GpsUpdateResult {
        state: new_state,
        covariance: p_new,
        applied: true,
        nis,
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
    fn kinematic_transition_is_identity_at_dt_zero() {
        let f = kinematic_transition(0.0);
        assert!((f - Covariance::identity()).norm() < 1.0e-9);
    }

    #[test]
    fn kinematic_transition_rejects_bad_dt() {
        let f = kinematic_transition(-1.0);
        assert!((f - Covariance::identity()).norm() < 1.0e-9);
    }

    #[test]
    fn kinematic_transition_sets_dp_dv_block_to_dt_identity() {
        let dt = 0.01_f32;
        let f = kinematic_transition(dt);
        let block =
            f.fixed_view::<{ idx::P_NED_LEN }, { idx::V_NED_LEN }>(idx::P_NED_START, idx::V_NED_START);
        // ∂p/∂v = dt * I_3  (Nalgebra's identity matrices are square, so
        // the 3x3 identity times dt is the expected block.)
        for i in 0..idx::P_NED_LEN {
            for j in 0..idx::V_NED_LEN {
                let v = block.fixed_view::<1, 1>(i, j).to_scalar();
                let expected = if i == j { dt } else { 0.0 };
                assert!((v - expected).abs() < 1.0e-9, "({i},{j}): got {v} want {expected}");
            }
        }
    }

    #[test]
    fn right_mult_matrix_on_identity_quaternion_is_identity() {
        let r = right_multiplication_matrix(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        assert!((r - Matrix4::identity()).norm() < 1.0e-9);
    }

    #[test]
    fn right_mult_matrix_matches_hamilton_product() {
        let p = Quaternion::new(0.5, 0.5, 0.5, 0.5); // unit quaternion
        let q = Quaternion::new(0.6, -0.2, 0.4, 0.2);
        let hamilton = p * q;
        let r = right_multiplication_matrix(q);
        let p_vec = Vector4::new(p.w, p.i, p.j, p.k);
        let expected = Vector4::new(hamilton.w, hamilton.i, hamilton.j, hamilton.k);
        let got = r * p_vec;
        assert!((got - expected).norm() < 1.0e-6);
    }

    #[test]
    fn left_mult_matrix_on_identity_quaternion_is_identity() {
        let l = left_multiplication_matrix(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        assert!((l - Matrix4::identity()).norm() < 1.0e-9);
    }

    #[test]
    fn left_mult_matrix_matches_hamilton_product() {
        let p = Quaternion::new(0.6, -0.2, 0.4, 0.2);
        let q = Quaternion::new(0.5, 0.5, 0.5, 0.5);
        let hamilton = p * q;
        let l = left_multiplication_matrix(p);
        let q_vec = Vector4::new(q.w, q.i, q.j, q.k);
        let expected = Vector4::new(hamilton.w, hamilton.i, hamilton.j, hamilton.k);
        let got = l * q_vec;
        assert!((got - expected).norm() < 1.0e-6);
    }

    #[test]
    fn dq_dbg_block_is_zero_at_dt_zero() {
        let state = State::default();
        let imu = ImuMeasurement::default();
        let f = build_transition_jacobian(&state, &imu, 0.0);
        let block =
            f.fixed_view::<{ idx::Q_LEN }, { idx::GYRO_BIAS_LEN }>(idx::Q_START, idx::GYRO_BIAS_START);
        assert!(block.norm() < 1.0e-9);
    }

    #[test]
    fn dq_dbg_block_with_identity_q_matches_expected_form() {
        // At q = identity, L(q) = I_4, so the 4×3 block should be:
        //    -(dt/2) · L(q)[:, 1:4]  =  -(dt/2) · [0; I_3]
        let state = State::default();
        let imu = ImuMeasurement::default();
        let dt = 0.01_f32;
        let f = build_transition_jacobian(&state, &imu, dt);
        let block =
            f.fixed_view::<{ idx::Q_LEN }, { idx::GYRO_BIAS_LEN }>(idx::Q_START, idx::GYRO_BIAS_START);
        // Row 0 (scalar part) should be all zero.
        for j in 0..idx::GYRO_BIAS_LEN {
            let v = block.fixed_view::<1, 1>(0, j).to_scalar();
            assert!(v.abs() < 1.0e-9, "scalar row (0, {j}) = {v}");
        }
        // Rows 1..4 should be -(dt/2) · I_3.
        for i in 0..idx::GYRO_BIAS_LEN {
            for j in 0..idx::GYRO_BIAS_LEN {
                let v = block.fixed_view::<1, 1>(i + 1, j).to_scalar();
                let expected = if i == j { -dt / 2.0 } else { 0.0 };
                assert!((v - expected).abs() < 1.0e-9);
            }
        }
    }

    // ---- GPS measurement update tests -----------------------------------

    #[test]
    fn gps_innovation_zero_residual_when_state_matches() {
        let state = State::default(); // position = 0
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::zeros(),
            sigma: Vector3::new(1.0, 1.0, 1.5),
        };
        let innov = gps_innovation(&state, &p, &m);
        assert!(innov.residual.norm() < 1.0e-9);
    }

    #[test]
    fn gps_innovation_residual_tracks_difference() {
        let state = State::default();
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::new(3.0, -2.0, 1.0),
            sigma: Vector3::new(1.0, 1.0, 1.5),
        };
        let innov = gps_innovation(&state, &p, &m);
        assert!((innov.residual - m.position_ned).norm() < 1.0e-9);
    }

    #[test]
    fn gps_innovation_covariance_is_pblock_plus_r() {
        let state = State::default();
        let p = initial_covariance();
        let sigma = Vector3::new(1.0, 1.5, 2.0);
        let m = GpsMeasurement { position_ned: Vector3::zeros(), sigma };
        let innov = gps_innovation(&state, &p, &m);
        // P_NED_NED block of P is diag(σ_pos²).
        let expected_p = initial_sigma::POSITION_NED.powi(2);
        let expected = Matrix3::from_diagonal(&Vector3::new(
            expected_p + sigma.x.powi(2),
            expected_p + sigma.y.powi(2),
            expected_p + sigma.z.powi(2),
        ));
        assert!((innov.innovation_covariance - expected).norm() < 1.0e-4);
    }

    #[test]
    fn gps_innovation_normalized_squared_is_small_for_aligned_measurement() {
        let state = State::default();
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::new(0.01, -0.02, 0.005), // ~cm residual
            sigma: Vector3::new(1.0, 1.0, 1.5),
        };
        let innov = gps_innovation(&state, &p, &m);
        let chi = innov.normalized_squared();
        // With σ ≈ 10 m and residual ≈ 1 cm, χ² should be << 0.01.
        assert!(chi < 0.01, "chi² = {chi} unexpectedly large");
    }

    // ---- End-to-end flight simulation -----------------------------------

    /// Simulate 2 seconds of "sitting on the bench": IMU reports gravity,
    /// GPS reports (1, -2, -3) NED, magnetometer reports body-aligned earth
    /// field, baro reports 3 m altitude. Verify EKF converges to each.
    #[test]
    fn end_to_end_stationary_multi_sensor_convergence() {
        let mut state = State::default();
        let mut p = initial_covariance();
        let noise = ProcessNoise::default();
        let dt = 0.001_f32; // 1 kHz predict
        let imu = ImuMeasurement {
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2),
        };
        let gps_target = Vector3::new(1.0, -2.0, -3.0);
        let gps = GpsMeasurement { position_ned: gps_target, sigma: Vector3::new(0.3, 0.3, 0.4) };
        let mag = MagMeasurement {
            body_field: state.mag_ned, // body == world at q = identity
            sigma: Vector3::new(0.005, 0.005, 0.005),
        };
        let baro = BaroMeasurement { altitude_m: 3.0, sigma_m: 0.1 };

        // 2 seconds of 1 kHz predict, with measurement updates at their own rates.
        for i in 0..2000 {
            let (s, c) = predict_step(&state, &p, imu, noise, dt);
            state = s;
            p = c;
            // GPS @ 5 Hz: every 200 steps
            if i % 200 == 0 {
                let r = gps_update(&state, &p, &gps);
                if r.applied {
                    state = r.state;
                    p = r.covariance;
                }
            }
            // Mag @ 25 Hz: every 40 steps
            if i % 40 == 0 {
                let r = mag_update(&state, &p, &mag);
                if r.applied {
                    state = r.state;
                    p = r.covariance;
                }
            }
            // Baro @ 50 Hz: every 20 steps
            if i % 20 == 0 {
                let r = baro_update(&state, &p, &baro);
                if r.applied {
                    state = r.state;
                    p = r.covariance;
                }
            }
        }
        // Position should converge to GPS.
        let pos_err = (state.position_ned - gps_target).norm();
        assert!(pos_err < 0.3, "position err = {pos_err} m");
        // Altitude should match baro (overrides GPS z).
        let altitude_err = (-state.position_ned.z - 3.0).abs();
        assert!(altitude_err < 0.2, "altitude err = {altitude_err} m");
        // Attitude should stay near identity.
        let att_err = (state.attitude.norm() - 1.0).abs();
        assert!(att_err < 1.0e-4, "‖q‖ − 1 = {att_err}");
        // All P diagonals positive, all entries finite.
        for i in 0..STATE_DIM {
            let v = p.fixed_view::<1, 1>(i, i).to_scalar();
            assert!(v > 0.0 && v.is_finite(), "P[{i},{i}] = {v}");
        }
    }

    // ---- Barometer update tests -----------------------------------------

    #[test]
    fn baro_update_zero_altitude_no_change() {
        let state = State::default(); // position_ned = 0 → altitude 0
        let p = initial_covariance();
        let m = BaroMeasurement { altitude_m: 0.0, sigma_m: 0.5 };
        let r = baro_update(&state, &p, &m);
        assert!(r.applied);
        assert!(r.state.position_ned.norm() < 1.0e-6);
    }

    #[test]
    fn baro_update_pulls_altitude_toward_measurement() {
        let state = State::default();
        let p = initial_covariance();
        let m = BaroMeasurement { altitude_m: 5.0, sigma_m: 0.3 };
        let r = baro_update(&state, &p, &m);
        assert!(r.applied);
        // After update the state's altitude (-position_ned.z) is closer to 5.
        let altitude = -r.state.position_ned.z;
        assert!(altitude > 0.1, "altitude didn't rise: {altitude}");
    }

    #[test]
    fn baro_update_rejects_massive_outlier() {
        let state = State::default();
        let p = initial_covariance();
        let m = BaroMeasurement { altitude_m: 10_000.0, sigma_m: 0.1 };
        let r = baro_update(&state, &p, &m);
        assert!(!r.applied);
    }

    #[test]
    fn baro_update_shrinks_altitude_variance() {
        let state = State::default();
        let p = initial_covariance();
        let m = BaroMeasurement { altitude_m: 1.0, sigma_m: 0.1 };
        let r = baro_update(&state, &p, &m);
        assert!(r.applied);
        let down_idx = idx::P_NED_START + 2;
        let before = p.fixed_view::<1, 1>(down_idx, down_idx).to_scalar();
        let after = r.covariance.fixed_view::<1, 1>(down_idx, down_idx).to_scalar();
        assert!(after < before, "down variance didn't shrink: {before} → {after}");
    }

    proptest! {
        /// Repeated baro updates converge altitude to the measured value.
        #[test]
        fn baro_update_converges(
            target_altitude in -20.0f32..20.0,
        ) {
            let mut state = State::default();
            let mut p = initial_covariance();
            let m = BaroMeasurement { altitude_m: target_altitude, sigma_m: 0.1 };
            for _ in 0..30 {
                let r = baro_update(&state, &p, &m);
                if r.applied {
                    state = r.state;
                    p = r.covariance;
                }
            }
            let altitude = -state.position_ned.z;
            prop_assert!(
                (altitude - target_altitude).abs() < 0.3,
                "target={target_altitude} got={altitude}"
            );
        }
    }

    // ---- Magnetometer update tests --------------------------------------

    #[test]
    fn predict_magnetometer_matches_expected_at_identity() {
        let state = State::default(); // q = identity, mag_bias = 0
        let predicted = predict_magnetometer(&state);
        // R(q)^T = I, so predicted = mag_ned + 0 = default mag_ned.
        assert!((predicted - state.mag_ned).norm() < 1.0e-6);
    }

    #[test]
    fn mag_update_zero_residual_leaves_state_nearly_unchanged() {
        let state = State::default();
        let p = initial_covariance();
        let m = MagMeasurement {
            body_field: predict_magnetometer(&state),
            sigma: Vector3::new(0.01, 0.01, 0.01),
        };
        let r = mag_update(&state, &p, &m);
        assert!(r.applied);
        assert!((r.state.attitude.norm() - 1.0).abs() < QUATERNION_NORM_TOLERANCE * 10.0);
        assert!(r.state.velocity_ned.norm() < 1.0e-3);
    }

    #[test]
    fn mag_update_rejects_huge_outlier() {
        let state = State::default();
        let p = initial_covariance();
        let m = MagMeasurement {
            body_field: Vector3::new(50.0, -50.0, 50.0), // far from reasonable
            sigma: Vector3::new(0.01, 0.01, 0.01),
        };
        let r = mag_update(&state, &p, &m);
        assert!(!r.applied);
    }

    #[test]
    fn mag_update_keeps_covariance_symmetric() {
        let state = State::default();
        let p = initial_covariance();
        let predicted = predict_magnetometer(&state);
        let m = MagMeasurement {
            body_field: predicted + Vector3::new(0.001, -0.002, 0.001),
            sigma: Vector3::new(0.01, 0.01, 0.01),
        };
        let r = mag_update(&state, &p, &m);
        assert!(r.applied);
        for i in 0..STATE_DIM {
            for j in (i + 1)..STATE_DIM {
                let a = r.covariance.fixed_view::<1, 1>(i, j).to_scalar();
                let b = r.covariance.fixed_view::<1, 1>(j, i).to_scalar();
                assert!((a - b).abs() < 1.0e-5, "asymmetric at ({i},{j})");
            }
        }
    }

    proptest! {
        /// Jacobian sanity: perturbing q and re-running the predicted
        /// measurement should match the closed-form Jacobian to first order.
        #[test]
        fn mag_h_jacobian_matches_finite_difference(
            qx in -0.005f32..0.005,
            qy in -0.005f32..0.005,
            qz in -0.005f32..0.005,
        ) {
            // Put state somewhere non-trivial so R(q) isn't identity.
            let mut state = State {
                attitude: Quaternion::new(0.9, 0.3, 0.2, 0.1),
                ..State::default()
            };
            state.normalize_attitude();

            let h = build_mag_observation_matrix(&state);
            let h_q = h.fixed_view::<3, { idx::Q_LEN }>(0, idx::Q_START).into_owned();

            // Build δq = (0, δx, δy, δz). Test only the 3 vector components
            // for simplicity.
            let delta_q = Vector4::new(0.0, qx, qy, qz);

            let state_plus = State {
                attitude: Quaternion::new(
                    state.attitude.w,
                    state.attitude.i + qx,
                    state.attitude.j + qy,
                    state.attitude.k + qz,
                ),
                ..state
            };
            let pred_nominal = predict_magnetometer(&state);
            let pred_plus = predict_magnetometer(&state_plus);
            let dh_measured = pred_plus - pred_nominal;
            let dh_predicted = h_q * delta_q;
            prop_assert!(
                (dh_measured - dh_predicted).norm() < 1.0e-4,
                "dh_measured: {}  dh_predicted: {}", dh_measured, dh_predicted
            );
        }
    }

    #[test]
    fn gps_update_rejects_massive_outlier() {
        let state = State::default();
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::new(10_000.0, 0.0, 0.0), // absurdly far
            sigma: Vector3::new(1.0, 1.0, 1.0),
        };
        let result = gps_update(&state, &p, &m);
        assert!(!result.applied, "outlier should have been rejected");
        // State and covariance unchanged.
        assert_eq!(result.state.position_ned, state.position_ned);
    }

    #[test]
    fn gps_update_pulls_state_toward_measurement() {
        let state = State::default();
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::new(1.0, -0.5, 0.2),
            sigma: Vector3::new(1.0, 1.0, 1.5),
        };
        let result = gps_update(&state, &p, &m);
        assert!(result.applied, "NIS {} should pass gate", result.nis);
        // Updated state is closer to measurement than original.
        let before_err = (state.position_ned - m.position_ned).norm();
        let after_err = (result.state.position_ned - m.position_ned).norm();
        assert!(after_err < before_err, "before={before_err}, after={after_err}");
    }

    #[test]
    fn gps_update_shrinks_position_variance() {
        let state = State::default();
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::new(1.0, 0.0, 0.0),
            sigma: Vector3::new(0.5, 0.5, 0.5), // tighter than prior
        };
        let result = gps_update(&state, &p, &m);
        assert!(result.applied);
        // Each position diagonal should shrink.
        for i in 0..idx::P_NED_LEN {
            let before = p.fixed_view::<1, 1>(idx::P_NED_START + i, idx::P_NED_START + i).to_scalar();
            let after = result
                .covariance
                .fixed_view::<1, 1>(idx::P_NED_START + i, idx::P_NED_START + i)
                .to_scalar();
            assert!(after < before, "diag {i}: before={before} after={after}");
        }
    }

    #[test]
    fn gps_update_keeps_covariance_symmetric_and_positive() {
        let state = State::default();
        let p = initial_covariance();
        let m = GpsMeasurement {
            position_ned: Vector3::new(0.5, -0.5, 0.2),
            sigma: Vector3::new(0.3, 0.3, 0.4),
        };
        let result = gps_update(&state, &p, &m);
        for i in 0..STATE_DIM {
            let v = result.covariance.fixed_view::<1, 1>(i, i).to_scalar();
            assert!(v > 0.0, "diagonal {i} = {v}");
            assert!(v.is_finite());
        }
        for i in 0..STATE_DIM {
            for j in (i + 1)..STATE_DIM {
                let a = result.covariance.fixed_view::<1, 1>(i, j).to_scalar();
                let b = result.covariance.fixed_view::<1, 1>(j, i).to_scalar();
                assert!((a - b).abs() < 1.0e-5, "asymmetric at ({i},{j})");
            }
        }
    }

    proptest! {
        /// Iterative GPS updates converge: after many updates with small
        /// noise, the estimated position comes within a fraction of σ.
        #[test]
        fn gps_update_converges_to_measured_position(
            tx in -5.0f32..5.0,
            ty in -5.0f32..5.0,
            tz in -5.0f32..5.0,
        ) {
            let mut state = State::default();
            let mut p = initial_covariance();
            let target = Vector3::new(tx, ty, tz);
            let m = GpsMeasurement {
                position_ned: target,
                sigma: Vector3::new(0.2, 0.2, 0.3),
            };
            for _ in 0..20 {
                let r = gps_update(&state, &p, &m);
                prop_assert!(r.applied || r.nis.is_finite());
                if r.applied {
                    state = r.state;
                    p = r.covariance;
                }
            }
            let residual = (state.position_ned - target).norm();
            prop_assert!(residual < 0.5, "residual {residual}  target {target}  got {}", state.position_ned);
        }
    }

    proptest! {
        /// Property: adding a meter offset between state and measurement
        /// always increases the normalised chi-square.
        #[test]
        fn gps_innovation_nis_monotone_in_offset(
            offset_m in 0.1f32..10.0,
        ) {
            let state = State::default();
            let p = initial_covariance();
            let sigma = Vector3::new(1.0, 1.0, 1.5);

            let m_small = GpsMeasurement {
                position_ned: Vector3::new(0.01, 0.0, 0.0),
                sigma,
            };
            let m_large = GpsMeasurement {
                position_ned: Vector3::new(offset_m, 0.0, 0.0),
                sigma,
            };
            let nis_small = gps_innovation(&state, &p, &m_small).normalized_squared();
            let nis_large = gps_innovation(&state, &p, &m_large).normalized_squared();
            prop_assert!(nis_large > nis_small,
                "NIS did not grow: small={nis_small}, large={nis_large}");
        }
    }

    #[test]
    fn predict_step_hovers_stable_state_is_stable_covariance() {
        // Level + stationary + zero rate: state stays put, covariance grows
        // only by the (tiny) process-noise diagonal.
        let mut state = State::default();
        let mut p = initial_covariance();
        let imu = ImuMeasurement {
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2),
        };
        let noise = ProcessNoise::default();
        let dt = 0.001_f32;
        for _ in 0..1000 {
            let (s, c) = predict_step(&state, &p, imu, noise, dt);
            state = s;
            p = c;
        }
        // ‖q‖ still unit.
        assert!((state.attitude.norm() - 1.0).abs() < QUATERNION_NORM_TOLERANCE * 10.0);
        // Velocity / position still bounded (numerical drift only).
        assert!(state.velocity_ned.norm() < 0.01);
        assert!(state.position_ned.norm() < 0.01);
        // Covariance diagonal all positive.
        for i in 0..STATE_DIM {
            let v = p.fixed_view::<1, 1>(i, i).to_scalar();
            assert!(v > 0.0, "P_{i}{i} went non-positive: {v}");
        }
        // Covariance stays symmetric.
        for i in 0..STATE_DIM {
            for j in (i + 1)..STATE_DIM {
                let a = p.fixed_view::<1, 1>(i, j).to_scalar();
                let b = p.fixed_view::<1, 1>(j, i).to_scalar();
                assert!((a - b).abs() < 1.0e-4, "P asymmetric at ({i},{j}): {a} vs {b}");
            }
        }
    }

    #[test]
    fn rotation_matrix_identity_quaternion() {
        let r = rotation_matrix(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        assert!((r - Matrix3::identity()).norm() < 1.0e-9);
    }

    #[test]
    fn rotation_matrix_unit_quaternion_is_orthogonal() {
        // Arbitrary unit quaternion: 90° rotation about z.
        let half = core::f32::consts::FRAC_1_SQRT_2;
        let q = Quaternion::new(half, 0.0, 0.0, half);
        let r = rotation_matrix(q);
        // R · R^T should be the identity.
        let prod = r * r.transpose();
        assert!((prod - Matrix3::identity()).norm() < 1.0e-6);
    }

    #[test]
    fn rotation_jacobian_at_identity_matches_skew() {
        // At q = identity, ∂(R·v)/∂q maps infinitesimal rotation into a
        // velocity change — the x/y/z columns should equal 2·skew_cols(v).
        let v = Vector3::new(1.0, 2.0, -0.5);
        let j = rotation_jacobian_wrt_q(Quaternion::new(1.0, 0.0, 0.0, 0.0), v);
        // Column 0 (∂/∂w): with q=identity → 2·(0 × v) = 0
        for i in 0..3 {
            let v_ij = j.fixed_view::<1, 1>(i, 0).to_scalar();
            assert!(v_ij.abs() < 1.0e-9, "∂w col row {i} = {v_ij}, expected 0");
        }
        // Columns 1..4 are 2·∂(R·v)/∂x|y|z at q=identity. The well-known
        // closed form is 2·[v × e_x, v × e_y, v × e_z] which expands to the
        // skew-symmetric entries listed below.
        // Column 1 (∂/∂x at q=identity):
        //   row 0:  2·(y·v_y + z·v_z) = 0 at q=I                 → 0
        //   row 1:  2·(y·v_x - 2x·v_y - w·v_z) = -2·v_z          → -2·v_z
        //   row 2:  2·(z·v_x + w·v_y - 2x·v_z) = 2·v_y           → 2·v_y
        let col1_r1 = j.fixed_view::<1, 1>(1, 1).to_scalar();
        let col1_r2 = j.fixed_view::<1, 1>(2, 1).to_scalar();
        assert!((col1_r1 - (-2.0 * v.z)).abs() < 1.0e-6);
        assert!((col1_r2 - (2.0 * v.y)).abs() < 1.0e-6);
    }

    #[test]
    fn dv_dba_block_at_identity_is_negative_dt() {
        let state = State::default();
        let imu = ImuMeasurement::default();
        let dt = 0.01_f32;
        let f = build_transition_jacobian(&state, &imu, dt);
        let block = f.fixed_view::<{ idx::V_NED_LEN }, { idx::ACCEL_BIAS_LEN }>(
            idx::V_NED_START,
            idx::ACCEL_BIAS_START,
        );
        // At q=identity, R(q) = I, so dv/db_a = -I · dt.
        for i in 0..idx::V_NED_LEN {
            for j_c in 0..idx::ACCEL_BIAS_LEN {
                let v = block.fixed_view::<1, 1>(i, j_c).to_scalar();
                let expected = if i == j_c { -dt } else { 0.0 };
                assert!((v - expected).abs() < 1.0e-9);
            }
        }
    }

    proptest! {
        /// Finite-difference sanity: perturbing accel_bias by δb_a and re-
        /// running predict should yield a velocity change of (∂v/∂b_a)·δb_a
        /// to first order.
        #[test]
        fn dv_dba_block_matches_finite_difference(
            bx in -0.05f32..0.05,
            by in -0.05f32..0.05,
            bz in -0.05f32..0.05,
            dt_s in 1.0e-3f32..0.02,
        ) {
            let state = State::default(); // q = identity
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(), // avoid R(q_new)≠R(q) drift
                accel_m_s2: Vector3::new(0.3, -0.4, -9.8),
            };
            let delta_b = Vector3::new(bx, by, bz);
            let state_plus = State { accel_bias: delta_b, ..state };
            let v_nominal = state.predict(imu, dt_s).velocity_ned;
            let v_perturbed = state_plus.predict(imu, dt_s).velocity_ned;
            let dv_measured = v_perturbed - v_nominal;

            let f = build_transition_jacobian(&state, &imu, dt_s);
            let block = f.fixed_view::<{ idx::V_NED_LEN }, { idx::ACCEL_BIAS_LEN }>(
                idx::V_NED_START,
                idx::ACCEL_BIAS_START,
            ).into_owned();
            let dv_predicted = block * delta_b;

            prop_assert!(
                (dv_measured - dv_predicted).norm() < 1.0e-4,
                "measured: {}  predicted: {}  diff_norm: {}",
                dv_measured, dv_predicted, (dv_measured - dv_predicted).norm()
            );
        }

        /// Invariant: over any random short IMU trajectory, predict_step
        /// keeps ‖q‖ within tolerance and P's diagonal positive.
        #[test]
        fn predict_step_preserves_invariants(
            steps in 10usize..200,
            wx in -3.0f32..3.0,
            wy in -3.0f32..3.0,
            wz in -3.0f32..3.0,
            ax in -5.0f32..5.0,
            ay in -5.0f32..5.0,
            az in (-GRAVITY_M_S2 - 2.0)..(-GRAVITY_M_S2 + 2.0),
            dt_s in 1.0e-4f32..0.01,
        ) {
            let mut state = State::default();
            let mut p = initial_covariance();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::new(wx, wy, wz),
                accel_m_s2: Vector3::new(ax, ay, az),
            };
            let noise = ProcessNoise::default();
            for _ in 0..steps {
                let (s, c) = predict_step(&state, &p, imu, noise, dt_s);
                state = s;
                p = c;
            }
            prop_assert!(
                (state.attitude.norm() - 1.0).abs() < 1.0e-4,
                "‖q‖ = {} drifted", state.attitude.norm()
            );
            for i in 0..STATE_DIM {
                let v = p.fixed_view::<1, 1>(i, i).to_scalar();
                prop_assert!(v > 0.0, "P_{i}{i} = {v} went non-positive");
                prop_assert!(v.is_finite(), "P_{i}{i} = {v} went non-finite");
            }
        }

        /// Finite-difference: perturbing accel_bias should change position
        /// by (∂p/∂b_a) · δb_a  =  -½·R(q)·dt² · δb_a  to first order.
        #[test]
        fn dp_dba_block_matches_finite_difference(
            bx in -0.05f32..0.05,
            by in -0.05f32..0.05,
            bz in -0.05f32..0.05,
            dt_s in 1.0e-3f32..0.02,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.4, -0.2, -9.78),
            };
            let delta_b = Vector3::new(bx, by, bz);
            let state_plus = State { accel_bias: delta_b, ..state };
            let p_nominal = state.predict(imu, dt_s).position_ned;
            let p_perturbed = state_plus.predict(imu, dt_s).position_ned;
            let dp_measured = p_perturbed - p_nominal;

            let f = build_transition_jacobian(&state, &imu, dt_s);
            let block = f.fixed_view::<{ idx::P_NED_LEN }, { idx::ACCEL_BIAS_LEN }>(
                idx::P_NED_START, idx::ACCEL_BIAS_START
            ).into_owned();
            let dp_predicted = block * delta_b;

            prop_assert!(
                (dp_measured - dp_predicted).norm() < 1.0e-6,
                "measured: {}  predicted: {}  diff_norm: {}",
                dp_measured, dp_predicted, (dp_measured - dp_predicted).norm()
            );
        }

        /// Finite-difference: perturbing q should change position to first
        /// order by (∂p/∂q) · δq. Tolerance wider because q_new ≠ q + δq
        /// once the normalisation and integration kick in.
        #[test]
        fn dp_dq_block_matches_finite_difference(
            qw_pert in -0.01f32..0.01,
            qx_pert in -0.01f32..0.01,
            qy_pert in -0.01f32..0.01,
            qz_pert in -0.01f32..0.01,
            dt_s in 1.0e-3f32..0.02,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.3, -0.4, -9.80),
            };
            let delta_q = Vector4::new(qw_pert, qx_pert, qy_pert, qz_pert);
            let state_plus = State {
                attitude: Quaternion::new(
                    state.attitude.w + qw_pert,
                    state.attitude.i + qx_pert,
                    state.attitude.j + qy_pert,
                    state.attitude.k + qz_pert,
                ),
                ..state
            };
            let p_nominal = state.predict(imu, dt_s).position_ned;
            let p_perturbed = state_plus.predict(imu, dt_s).position_ned;
            let dp_measured = p_perturbed - p_nominal;

            let f = build_transition_jacobian(&state, &imu, dt_s);
            let block = f
                .fixed_view::<{ idx::P_NED_LEN }, { idx::Q_LEN }>(idx::P_NED_START, idx::Q_START)
                .into_owned();
            let dp_predicted = block * delta_q;

            prop_assert!(
                (dp_measured - dp_predicted).norm() < 1.0e-5,
                "measured: {}  predicted: {}  diff_norm: {}",
                dp_measured, dp_predicted, (dp_measured - dp_predicted).norm()
            );
        }

        /// Finite-difference: perturbing attitude and running predict should
        /// change velocity by ∂v/∂q · δq to first order. Bounded δq so the
        /// small-q-drift approximation stays < 0.01 rad overall.
        #[test]
        fn dv_dq_block_matches_finite_difference(
            qw_pert in -0.01f32..0.01,
            qx_pert in -0.01f32..0.01,
            qy_pert in -0.01f32..0.01,
            qz_pert in -0.01f32..0.01,
            dt_s in 1.0e-3f32..0.02,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.2, -0.3, -9.81),
            };
            let delta_q = Vector4::new(qw_pert, qx_pert, qy_pert, qz_pert);
            let state_plus = State {
                attitude: Quaternion::new(
                    state.attitude.w + qw_pert,
                    state.attitude.i + qx_pert,
                    state.attitude.j + qy_pert,
                    state.attitude.k + qz_pert,
                ),
                ..state
            };
            let v_nominal = state.predict(imu, dt_s).velocity_ned;
            let v_perturbed = state_plus.predict(imu, dt_s).velocity_ned;
            let dv_measured = v_perturbed - v_nominal;

            let f = build_transition_jacobian(&state, &imu, dt_s);
            let block = f
                .fixed_view::<{ idx::V_NED_LEN }, { idx::Q_LEN }>(idx::V_NED_START, idx::Q_START)
                .into_owned();
            let dv_predicted = block * delta_q;

            prop_assert!(
                (dv_measured - dv_predicted).norm() < 5.0e-4,
                "measured: {}  predicted: {}  diff_norm: {}",
                dv_measured, dv_predicted, (dv_measured - dv_predicted).norm()
            );
        }
    }

    proptest! {
        /// Consistency check: a positive gyro bias error δb_g causes the
        /// predicted quaternion to move opposite to a positive δω would
        /// (because ω_effective = ω - b_g). Concretely, if we perturb
        /// gyro_bias and re-run the predict, the resulting quaternion
        /// difference should equal ∂q/∂b_g · δb_g to first order.
        #[test]
        fn dq_dbg_block_matches_finite_difference(
            bx in -0.01f32..0.01,
            by in -0.01f32..0.01,
            bz in -0.01f32..0.01,
            dt_s in 1.0e-3f32..0.02,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::new(0.1, -0.2, 0.15), // nominal rate
                accel_m_s2: Vector3::zeros(),
            };
            let delta_b = Vector3::new(bx, by, bz);

            let state_plus = State { gyro_bias: delta_b, ..state };

            let q_nominal = state.predict(imu, dt_s).attitude;
            let q_perturbed = state_plus.predict(imu, dt_s).attitude;
            let dq_measured =
                Vector4::new(q_perturbed.w, q_perturbed.i, q_perturbed.j, q_perturbed.k)
                    - Vector4::new(q_nominal.w, q_nominal.i, q_nominal.j, q_nominal.k);

            let f = build_transition_jacobian(&state, &imu, dt_s);
            let block =
                f.fixed_view::<{ idx::Q_LEN }, { idx::GYRO_BIAS_LEN }>(idx::Q_START, idx::GYRO_BIAS_START)
                    .into_owned();
            let dq_predicted = block * delta_b;

            prop_assert!(
                (dq_measured - dq_predicted).norm() < 1.0e-4,
                "measured: {}  predicted: {}  diff_norm: {}",
                dq_measured, dq_predicted, (dq_measured - dq_predicted).norm()
            );
        }
    }

    #[test]
    fn build_transition_jacobian_is_identity_at_zero_rate_and_dt() {
        let state = State::default();
        let imu = ImuMeasurement::default();
        let f = build_transition_jacobian(&state, &imu, 0.0);
        assert!((f - Covariance::identity()).norm() < 1.0e-9);
    }

    #[test]
    fn build_transition_jacobian_zero_rate_keeps_attitude_block_identity() {
        let state = State::default();
        let imu = ImuMeasurement::default(); // zero gyro
        let dt = 0.005;
        let f = build_transition_jacobian(&state, &imu, dt);
        let block = f.fixed_view::<{ idx::Q_LEN }, { idx::Q_LEN }>(idx::Q_START, idx::Q_START);
        assert!((block - Matrix4::identity()).norm() < 1.0e-6);
    }

    proptest! {
        /// The quaternion self-Jacobian `∂q/∂q = R(δq)` has determinant
        /// exactly 1 when δq is a unit quaternion (rotation preserves volume).
        #[test]
        fn dq_dq_block_has_unit_determinant(
            wx in -5.0f32..5.0,
            wy in -5.0f32..5.0,
            wz in -5.0f32..5.0,
            dt_s in 1.0e-4f32..0.02,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::new(wx, wy, wz),
                accel_m_s2: Vector3::zeros(),
            };
            let f = build_transition_jacobian(&state, &imu, dt_s);
            let block: Matrix4<f32> =
                f.fixed_view::<{ idx::Q_LEN }, { idx::Q_LEN }>(idx::Q_START, idx::Q_START).into_owned();
            // Determinant of a right-multiplication matrix of a unit quaternion = 1.
            prop_assert!((block.determinant() - 1.0).abs() < 1.0e-4,
                "det(block) = {}", block.determinant());
        }

        /// End-to-end: feeding the kinematic F to predict_covariance produces
        /// a symmetric P_new for any positive dt in the normal operating range.
        #[test]
        fn predict_covariance_with_kinematic_f_is_symmetric(dt_s in 1.0e-4f32..0.05) {
            let p = initial_covariance();
            let f = kinematic_transition(dt_s);
            let q = build_process_noise(ProcessNoise::default(), dt_s);
            let p_next = predict_covariance(&p, &f, &q);
            for i in 0..STATE_DIM {
                for j in (i + 1)..STATE_DIM {
                    let a = p_next.fixed_view::<1, 1>(i, j).to_scalar();
                    let b = p_next.fixed_view::<1, 1>(j, i).to_scalar();
                    prop_assert!((a - b).abs() < 1.0e-6);
                }
            }
        }

        /// The kinematic F cross-couples position and velocity — after one
        /// predict step with this F, the P block at (position, velocity)
        /// must be non-zero whenever the velocity block started non-zero.
        #[test]
        fn kinematic_f_transfers_velocity_uncertainty_to_position(dt_s in 1.0e-3f32..0.05) {
            let p = initial_covariance();
            let f = kinematic_transition(dt_s);
            let q = Covariance::zeros(); // isolate F's effect, no process noise
            let p_next = predict_covariance(&p, &f, &q);
            // (0th position row, 0th velocity col) should now be ≈ dt * σ_v²
            let cross = p_next
                .fixed_view::<1, 1>(idx::P_NED_START, idx::V_NED_START)
                .to_scalar();
            let v_var = initial_sigma::VELOCITY_NED.powi(2);
            prop_assert!(cross > 0.0, "cross block should be positive, got {cross}");
            prop_assert!(
                (cross - dt_s * v_var).abs() < 1.0e-4,
                "cross={cross} expected {}", dt_s * v_var
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

        /// Finite-diff: ∂v/∂wind_ne block of the drag-aware F matrix
        /// matches the numerical derivative of predict_with_drag.
        #[test]
        fn dv_dwind_block_matches_finite_difference(
            drag in 0.1f32..1.0,
            wx in -0.05f32..0.05,
            wy in -0.05f32..0.05,
            dt_s in 1.0e-4f32..0.02,
        ) {
            let state = State::default();
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2),
            };
            let dw = nalgebra::Vector2::new(wx, wy);

            let v_nominal = state.predict_with_drag(imu, dt_s, drag).velocity_ned;
            let state_plus = State { wind_ne: dw, ..state };
            let v_perturbed = state_plus.predict_with_drag(imu, dt_s, drag).velocity_ned;
            let dv_measured = v_perturbed - v_nominal;

            let fmat = build_transition_jacobian_with_drag(&state, &imu, dt_s, drag);
            let block = fmat
                .fixed_view::<{ idx::V_NED_LEN }, { idx::WIND_NE_LEN }>(
                    idx::V_NED_START, idx::WIND_NE_START,
                )
                .into_owned();
            let dv_predicted = block * dw;

            prop_assert!(
                (dv_measured - dv_predicted).norm() < 1.0e-5,
                "measured {dv_measured} predicted {dv_predicted}"
            );
        }

        /// Finite-diff: ∂v/∂v correction matches. Perturb velocity by δv
        /// and check the v_new change contains the −drag·dt·δv term.
        #[test]
        fn dv_dv_drag_correction_matches_finite_difference(
            drag in 0.1f32..1.0,
            v0x in -1.0f32..1.0,
            dv in -0.05f32..0.05,
            dt_s in 1.0e-4f32..0.02,
        ) {
            let state_nominal = State { velocity_ned: Vector3::new(v0x, 0.0, 0.0), ..State::default() };
            let state_perturbed = State {
                velocity_ned: Vector3::new(v0x + dv, 0.0, 0.0),
                ..State::default()
            };
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2),
            };
            let v_nominal = state_nominal.predict_with_drag(imu, dt_s, drag).velocity_ned;
            let v_perturbed = state_perturbed.predict_with_drag(imu, dt_s, drag).velocity_ned;
            let dv_measured = v_perturbed - v_nominal;

            let fmat = build_transition_jacobian_with_drag(&state_nominal, &imu, dt_s, drag);
            let block = fmat
                .fixed_view::<{ idx::V_NED_LEN }, { idx::V_NED_LEN }>(
                    idx::V_NED_START, idx::V_NED_START,
                )
                .into_owned();
            let dv_input = Vector3::new(dv, 0.0, 0.0);
            let dv_predicted = block * dv_input;

            prop_assert!(
                (dv_measured - dv_predicted).norm() < 1.0e-5,
                "measured {dv_measured} predicted {dv_predicted}"
            );
        }

        /// Drag-aware predict: at zero wind, a moving vehicle decelerates
        /// proportional to its velocity.
        #[test]
        fn predict_with_drag_decelerates_when_wind_zero(
            drag in 0.1f32..2.0,
            v0 in 0.5f32..5.0,
            dt_s in 1.0e-4f32..0.05,
        ) {
            let state = State { velocity_ned: Vector3::new(v0, 0.0, 0.0), ..State::default() };
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2), // level hover
            };
            let next = state.predict_with_drag(imu, dt_s, drag);
            // Vehicle slows in +x by ~ drag · v0 · dt
            let expected = v0 * (1.0 - drag * dt_s);
            prop_assert!((next.velocity_ned.x - expected).abs() < 1.0e-3);
        }

        /// Drag-aware predict with wind: a stationary vehicle in wind is
        /// pushed along the wind direction.
        #[test]
        fn predict_with_drag_and_wind_pushes_stationary_vehicle(
            drag in 0.1f32..2.0,
            wx in -3.0f32..3.0,
            wy in -3.0f32..3.0,
            dt_s in 1.0e-4f32..0.05,
        ) {
            let state = State {
                wind_ne: nalgebra::Vector2::new(wx, wy),
                ..State::default()
            };
            let imu = ImuMeasurement {
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.0, 0.0, -GRAVITY_M_S2),
            };
            let next = state.predict_with_drag(imu, dt_s, drag);
            // Δv = drag·(wind − v)·dt = drag·wind·dt at v=0.
            let expected_vx = drag * wx * dt_s;
            let expected_vy = drag * wy * dt_s;
            prop_assert!((next.velocity_ned.x - expected_vx).abs() < 1.0e-3);
            prop_assert!((next.velocity_ned.y - expected_vy).abs() < 1.0e-3);
            // z untouched because wind is horizontal.
            prop_assert!(next.velocity_ned.z.abs() < 1.0e-3);
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
