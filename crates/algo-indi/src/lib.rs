#![no_std]

//! Incremental Nonlinear Dynamic Inversion (INDI) inner loop.
//!
//! # What INDI does
//!
//! The plant's angular dynamics are `ω̇ = J⁻¹·τ + disturbance(ω)`.
//! Taking increments from the last known state:
//!
//! ```text
//!   Δω̇ ≈ J⁻¹·Δτ                   (first-order linearization)
//!   ⇒  Δτ = J·(ω̇_desired − ω̇_measured)
//! ```
//!
//! where `ω̇_desired = k_rate ⊙ (ω_cmd − ω)`. The increment `Δτ` is the
//! *virtual* torque the control allocator must produce; the allocator then
//! multiplies by `G⁻¹` (actuator-to-torque inverse) to get motor commands.
//!
//! **Compared to PID** this requires no accurate plant model — the
//! disturbance term is absorbed by using measured `ω̇` directly.
//!
//! # This module provides
//!
//! * [`LowPassFilterVec3`] — first-order IIR for the caller's filter bank
//!   (gyro and angular-accel signals must be clean before INDI sees them).
//! * [`IndiController`] — pure compute, no state; takes **already-filtered**
//!   `ω` and `ω̇` plus the vehicle's inertia `J` and gains, returns `Δτ`.

use nalgebra::{Matrix3, Quaternion, Vector3};

/// Per-axis angular-rate setpoint from the attitude loop.
#[derive(Clone, Copy, Debug, Default)]
pub struct RateCommand {
    pub body_rate_rad_s: Vector3<f32>,
}

/// Output of one INDI step — virtual torque increment in body frame, Nm.
#[derive(Clone, Copy, Debug, Default)]
pub struct TorqueIncrement {
    pub body_torque_nm: Vector3<f32>,
}

/// Proportional gain per body axis (x, y, z). Typical multirotor values
/// are 15–40 rad/s².
pub type RateGain = Vector3<f32>;

/// Inertia matrix (moment of inertia tensor) in body frame, kg·m².
///
/// Usually diagonal for a symmetric multirotor; kept as full 3×3 so a
/// future VTOL / tilt-rotor configuration can pass cross-coupling.
pub type Inertia = Matrix3<f32>;

/// Inputs required for one INDI step.
#[derive(Clone, Copy, Debug)]
pub struct IndiInput<'a> {
    /// Rate setpoint from the outer attitude loop.
    pub cmd: RateCommand,
    /// Filtered body-frame angular rate (rad/s).
    pub omega_filtered: Vector3<f32>,
    /// Filtered body-frame angular acceleration (rad/s²).
    pub omega_dot_filtered: Vector3<f32>,
    /// Per-axis proportional gain on the rate error.
    pub k_rate: &'a RateGain,
    /// Vehicle inertia `J` (kg·m²).
    pub inertia: &'a Inertia,
}

/// Compute the virtual torque increment for one INDI step.
///
/// `Δτ = J · (k_rate ⊙ (ω_cmd − ω_filtered) − ω̇_filtered)`
///
/// No state — the caller holds the filter state and any rate-limited
/// integrator.
#[must_use]
pub fn compute_torque_increment(input: &IndiInput) -> TorqueIncrement {
    let rate_error = input.cmd.body_rate_rad_s - input.omega_filtered;
    let desired_omega_dot = input.k_rate.component_mul(&rate_error);
    let delta_omega_dot = desired_omega_dot - input.omega_dot_filtered;
    TorqueIncrement {
        body_torque_nm: input.inertia * delta_omega_dot,
    }
}

// ----------------------------------------------------------------------------
// Attitude loop
// ----------------------------------------------------------------------------

/// Per-axis attitude proportional gain. Typical values 5–15 rad/s per rad.
pub type AttitudeGain = Vector3<f32>;

/// Compute a body-frame rate command that drives `q_current` toward
/// `q_desired`.
///
/// Uses the standard quaternion-error formulation:
///
/// ```text
///   q_err = q_desired ⊗ q_current⁻¹
///   choose the shortest rotation (q_err.w ≥ 0)
///   ω_cmd = 2·k_att ⊙ q_err.vector_part
/// ```
///
/// The `2·vector_part` approximation is exact in the small-angle limit
/// and smoothly degrades for large errors.
///
/// Inputs must be unit quaternions. If the current attitude is degenerate
/// (zero-norm), returns a zero rate command to avoid producing NaNs.
#[must_use]
pub fn attitude_to_rate(
    q_current: Quaternion<f32>,
    q_desired: Quaternion<f32>,
    k_att: &AttitudeGain,
) -> RateCommand {
    let q_current_norm_sq =
        q_current.w * q_current.w + q_current.i * q_current.i + q_current.j * q_current.j + q_current.k * q_current.k;
    if !q_current_norm_sq.is_finite() || q_current_norm_sq < 1.0e-12 {
        return RateCommand::default();
    }
    // Conjugate of a unit quaternion q = (w, x, y, z) is (w, -x, -y, -z).
    let q_current_conj = Quaternion::new(
        q_current.w,
        -q_current.i,
        -q_current.j,
        -q_current.k,
    );
    let q_err = q_desired * q_current_conj;
    // Shortest-rotation: flip so q_err.w ≥ 0.
    let (ex, ey, ez) = if q_err.w >= 0.0 {
        (q_err.i, q_err.j, q_err.k)
    } else {
        (-q_err.i, -q_err.j, -q_err.k)
    };
    let err_vec = Vector3::new(2.0 * ex, 2.0 * ey, 2.0 * ez);
    RateCommand {
        body_rate_rad_s: k_att.component_mul(&err_vec),
    }
}

// ----------------------------------------------------------------------------
// Low-pass filter
// ----------------------------------------------------------------------------

/// First-order IIR low-pass on a 3-vector signal.
///
/// Discrete form: `y[n] = α·x[n] + (1 − α)·y[n−1]`
/// with `α = dt / (τ + dt)` and `τ = 1 / (2π · f_c)`.
///
/// First `update` call initialises the filter to the input (no startup
/// transient on the first sample).
#[derive(Clone, Copy, Debug)]
pub struct LowPassFilterVec3 {
    output: Vector3<f32>,
    alpha: f32,
    initialized: bool,
}

impl LowPassFilterVec3 {
    /// Build a filter with cutoff `cutoff_hz` operating at `sample_rate_hz`.
    ///
    /// `cutoff_hz` must be strictly below `sample_rate_hz / 2` (Nyquist).
    /// Both values must be finite and positive.
    #[must_use]
    pub fn new(cutoff_hz: f32, sample_rate_hz: f32) -> Self {
        let alpha = if cutoff_hz > 0.0
            && sample_rate_hz > 0.0
            && cutoff_hz.is_finite()
            && sample_rate_hz.is_finite()
            && cutoff_hz < sample_rate_hz * 0.5
        {
            let dt = 1.0 / sample_rate_hz;
            let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
            dt / (tau + dt)
        } else {
            1.0 // pass-through on invalid config
        };
        Self {
            output: Vector3::zeros(),
            alpha,
            initialized: false,
        }
    }

    /// Feed one sample and return the filtered value.
    pub fn update(&mut self, input: Vector3<f32>) -> Vector3<f32> {
        if self.initialized {
            self.output = self.alpha * input + (1.0 - self.alpha) * self.output;
        } else {
            self.output = input;
            self.initialized = true;
        }
        self.output
    }

    /// Current filter output without applying a new sample.
    #[must_use]
    pub fn output(&self) -> Vector3<f32> {
        self.output
    }

    /// Force the filter back to zero / uninitialised.
    pub fn reset(&mut self) {
        self.output = Vector3::zeros();
        self.initialized = false;
    }

    /// The smoothing coefficient (exposed for diagnostics / tests).
    #[must_use]
    pub fn alpha(&self) -> f32 {
        self.alpha
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn default_inertia() -> Inertia {
        Matrix3::from_diagonal(&Vector3::new(0.015, 0.015, 0.025)) // 250-g quad
    }

    fn default_gain() -> RateGain {
        Vector3::new(25.0, 25.0, 15.0)
    }

    // ---- compute_torque_increment ----------------------------------------

    #[test]
    fn zero_rate_error_zero_accel_yields_zero_torque() {
        let j = default_inertia();
        let k = default_gain();
        let input = IndiInput {
            cmd: RateCommand::default(),
            omega_filtered: Vector3::zeros(),
            omega_dot_filtered: Vector3::zeros(),
            k_rate: &k,
            inertia: &j,
        };
        let out = compute_torque_increment(&input);
        assert_eq!(out.body_torque_nm, Vector3::zeros());
    }

    #[test]
    fn positive_error_yields_positive_torque_on_same_axis() {
        let j = default_inertia();
        let k = default_gain();
        let input = IndiInput {
            cmd: RateCommand { body_rate_rad_s: Vector3::new(1.0, 0.0, 0.0) },
            omega_filtered: Vector3::zeros(),
            omega_dot_filtered: Vector3::zeros(),
            k_rate: &k,
            inertia: &j,
        };
        let out = compute_torque_increment(&input);
        // τ_x = 0.015 · 25 · 1.0 = 0.375, others 0.
        assert!((out.body_torque_nm.x - 0.375).abs() < 1.0e-6);
        assert!(out.body_torque_nm.y.abs() < 1.0e-9);
        assert!(out.body_torque_nm.z.abs() < 1.0e-9);
    }

    #[test]
    fn measured_angular_accel_cancels_desired() {
        // If the plant is already accelerating at the desired rate, the
        // increment should be zero.
        let j = default_inertia();
        let k = default_gain();
        let rate_err = Vector3::new(1.0, 0.0, 0.0);
        let desired_accel = Vector3::new(k.x * rate_err.x, 0.0, 0.0);
        let input = IndiInput {
            cmd: RateCommand { body_rate_rad_s: rate_err },
            omega_filtered: Vector3::zeros(),
            omega_dot_filtered: desired_accel,
            k_rate: &k,
            inertia: &j,
        };
        let out = compute_torque_increment(&input);
        assert!(out.body_torque_nm.norm() < 1.0e-9);
    }

    proptest! {
        /// Linearity: for any finite input, the torque magnitude is
        /// bounded by the inertia norm × (|k_rate| × |rate_err| + |accel|).
        #[test]
        fn torque_magnitude_is_bounded(
            cx in -5.0f32..5.0,
            cy in -5.0f32..5.0,
            cz in -5.0f32..5.0,
            ox in -5.0f32..5.0,
            oy in -5.0f32..5.0,
            oz in -5.0f32..5.0,
            ax in -50.0f32..50.0,
            ay in -50.0f32..50.0,
            az in -50.0f32..50.0,
        ) {
            let j = default_inertia();
            let k = default_gain();
            let input = IndiInput {
                cmd: RateCommand { body_rate_rad_s: Vector3::new(cx, cy, cz) },
                omega_filtered: Vector3::new(ox, oy, oz),
                omega_dot_filtered: Vector3::new(ax, ay, az),
                k_rate: &k,
                inertia: &j,
            };
            let out = compute_torque_increment(&input);
            let rate_err = Vector3::new(cx - ox, cy - oy, cz - oz);
            let desired_accel = Vector3::new(
                k.x * rate_err.x, k.y * rate_err.y, k.z * rate_err.z,
            );
            let delta_alpha = desired_accel - Vector3::new(ax, ay, az);
            let expected = j * delta_alpha;
            prop_assert!((out.body_torque_nm - expected).norm() < 1.0e-4);
        }
    }

    // ---- Attitude loop ---------------------------------------------------

    fn default_att_gain() -> AttitudeGain {
        Vector3::new(8.0, 8.0, 5.0)
    }

    #[test]
    fn attitude_equal_yields_zero_rate() {
        let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let cmd = attitude_to_rate(q, q, &default_att_gain());
        assert!(cmd.body_rate_rad_s.norm() < 1.0e-9);
    }

    #[test]
    fn attitude_desired_yaws_right_produces_positive_yaw_rate() {
        // q_desired = 20° about +z.
        let theta = 20.0_f32.to_radians();
        let qd = Quaternion::new((theta / 2.0).cos(), 0.0, 0.0, (theta / 2.0).sin());
        let qc = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let cmd = attitude_to_rate(qc, qd, &default_att_gain());
        assert!(cmd.body_rate_rad_s.z > 0.0, "expected +yaw, got {}", cmd.body_rate_rad_s);
        assert!(cmd.body_rate_rad_s.x.abs() < 1.0e-5);
        assert!(cmd.body_rate_rad_s.y.abs() < 1.0e-5);
    }

    #[test]
    fn attitude_takes_shortest_rotation() {
        // q_desired = 350° about z is the same attitude as -10° about z.
        // A correct controller goes -10° (back), not +350° (forward).
        let theta = 350.0_f32.to_radians();
        let qd = Quaternion::new((theta / 2.0).cos(), 0.0, 0.0, (theta / 2.0).sin());
        let qc = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let cmd = attitude_to_rate(qc, qd, &default_att_gain());
        // Expect *negative* yaw rate (shortest path = -10°).
        assert!(cmd.body_rate_rad_s.z < 0.0, "expected -yaw, got {}", cmd.body_rate_rad_s);
    }

    #[test]
    fn attitude_degenerate_quaternion_returns_zero() {
        let qc = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        let qd = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let cmd = attitude_to_rate(qc, qd, &default_att_gain());
        assert_eq!(cmd.body_rate_rad_s, Vector3::zeros());
    }

    proptest! {
        /// Small-angle: for unit q_current and a desired that differs by
        /// a small body-frame rotation, the returned rate approximates
        /// k_att ⊙ rotation_axis.
        #[test]
        fn attitude_small_angle_matches_axis_angle(
            rx in -0.05f32..0.05,
            ry in -0.05f32..0.05,
            rz in -0.05f32..0.05,
        ) {
            let qc = Quaternion::new(1.0, 0.0, 0.0, 0.0);
            // q_desired = exp(½·(rx, ry, rz))  —  small angle
            let half_rot = Vector3::new(rx / 2.0, ry / 2.0, rz / 2.0);
            let w = libm::sqrtf((1.0 - half_rot.norm_squared()).max(0.0));
            let qd = Quaternion::new(w, half_rot.x, half_rot.y, half_rot.z);
            let k = default_att_gain();
            let cmd = attitude_to_rate(qc, qd, &k);
            // Expected: ω ≈ k ⊙ (rx, ry, rz)
            let expected = Vector3::new(k.x * rx, k.y * ry, k.z * rz);
            prop_assert!((cmd.body_rate_rad_s - expected).norm() < 5.0e-3);
        }
    }

    // ---- LowPassFilterVec3 -----------------------------------------------

    #[test]
    fn lpf_invalid_config_is_passthrough() {
        let mut f = LowPassFilterVec3::new(-1.0, 1000.0);
        let v = Vector3::new(1.0, 2.0, 3.0);
        // With alpha=1.0, output = input immediately.
        assert_eq!(f.update(v), v);
    }

    #[test]
    fn lpf_first_sample_sets_output_to_input() {
        let mut f = LowPassFilterVec3::new(50.0, 1000.0);
        let v = Vector3::new(1.0, -2.0, 3.0);
        assert_eq!(f.update(v), v);
    }

    #[test]
    fn lpf_constant_input_settles_to_input() {
        let mut f = LowPassFilterVec3::new(50.0, 1000.0);
        let target = Vector3::new(1.0, 0.0, 0.0);
        for _ in 0..1000 {
            f.update(target);
        }
        assert!((f.output() - target).norm() < 1.0e-6);
    }

    #[test]
    fn lpf_step_response_converges_exponentially() {
        let mut f = LowPassFilterVec3::new(50.0, 1000.0);
        f.update(Vector3::zeros()); // initialize to 0
        f.reset();
        f.update(Vector3::zeros());
        let target = Vector3::new(1.0, 0.0, 0.0);
        let mut err_prev = 1.0;
        for _ in 0..5 {
            let out = f.update(target);
            let err = (target.x - out.x).abs();
            assert!(err < err_prev, "monotonic decay violated: {err_prev} → {err}");
            err_prev = err;
        }
    }

    #[test]
    fn lpf_reset_zeros_state() {
        let mut f = LowPassFilterVec3::new(50.0, 1000.0);
        f.update(Vector3::new(1.0, 2.0, 3.0));
        f.reset();
        assert_eq!(f.output(), Vector3::zeros());
        // Next update re-initialises from the fresh sample.
        let v = Vector3::new(10.0, 20.0, 30.0);
        assert_eq!(f.update(v), v);
    }

    proptest! {
        /// For any constant input, the filter converges to it within N samples.
        #[test]
        fn lpf_converges_to_constant(
            x in -100.0f32..100.0,
            y in -100.0f32..100.0,
            z in -100.0f32..100.0,
        ) {
            let mut f = LowPassFilterVec3::new(50.0, 1000.0);
            let target = Vector3::new(x, y, z);
            for _ in 0..2000 {
                f.update(target);
            }
            prop_assert!((f.output() - target).norm() < 1.0e-3);
        }
    }
}
