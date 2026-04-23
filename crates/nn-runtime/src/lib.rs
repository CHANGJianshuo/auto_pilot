#![no_std]

//! Neural-network / RL policy inference, gated by a safety envelope.
//!
//! Architecture
//! ============
//!
//! The crate hosts two decoupled concerns:
//!
//! 1. **Inference backend** ([`InferenceBackend`]) — pluggable compute
//!    engine that turns a `FeatureVector` into a `Residual`. A tiny
//!    hand-coded affine layer ([`AffineBackend`]) ships with the
//!    crate and is the reference implementation used by tests. Real
//!    deployments bind to `tract` / `candle` / whatever else is
//!    available — the rest of the flight stack doesn't care which.
//!
//! 2. **Policy harness** ([`ResidualPolicy`]) — ties the backend to a
//!    [`SafetyEnvelope`]. Every `.predict()` returns either an
//!    accepted residual accel or a typed rejection reason.
//!
//! Keeping these separate means the policy plumbing stays testable
//! without a real ONNX runtime in the dep graph, while the inference
//! backend can be swapped out when the training pipeline produces
//! actual models.
//!
//! Residual learning, not end-to-end
//! ==================================
//!
//! First release: the NN outputs a **correction** to the NMPC command
//! (a residual NED acceleration), not a raw command. This matches the
//! Swift / UZH RPG approach — the controller stays classical and
//! provably-stable, the NN compensates for model mismatch (aero drag
//! that isn't in the EKF, unmodelled motor dynamics, payload offset).

use heapless::Vec;
use nalgebra::Vector3;

// ---- Feature & output schemas ---------------------------------------------

/// Fixed-length feature vector fed to the policy.
///
/// The size is fixed at compile time so the backend can preallocate
/// buffers and the serde layout (when we ship models over Zenoh) is
/// stable. If a future policy needs more features, bump `FEATURE_LEN`
/// in lock-step with the trained model's input shape.
pub const FEATURE_LEN: usize = 9;

/// Fixed-length residual output — 3 NED-axis accelerations.
pub const RESIDUAL_LEN: usize = 3;

/// Input features: 9-vec of (pos_err_ned, vel_ned, rpy_rad).
///
/// Ordering is deliberate so a debugger dump is human-readable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeatureVector(pub [f32; FEATURE_LEN]);

impl FeatureVector {
    #[must_use]
    pub fn zeros() -> Self {
        Self([0.0; FEATURE_LEN])
    }

    /// Assemble the canonical feature vector from the estimator state + commanded setpoint.
    /// Kept as a pure function so policy tests can synthesise inputs without constructing a full FlightState.
    #[must_use]
    pub fn from_state(
        pos_err_ned: Vector3<f32>,
        velocity_ned: Vector3<f32>,
        rpy_rad: Vector3<f32>,
    ) -> Self {
        Self([
            pos_err_ned.x,
            pos_err_ned.y,
            pos_err_ned.z,
            velocity_ned.x,
            velocity_ned.y,
            velocity_ned.z,
            rpy_rad.x,
            rpy_rad.y,
            rpy_rad.z,
        ])
    }
}

/// Residual accel output — added to the NMPC's `accel_cmd` before
/// force-balance. Units: m/s² in NED.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Residual(pub Vector3<f32>);

impl Residual {
    #[must_use]
    pub const fn zeros() -> Self {
        Self(Vector3::new(0.0, 0.0, 0.0))
    }
}

// ---- Inference backend trait ----------------------------------------------

/// Something that maps a feature vector to a residual.
///
/// Implementations must be deterministic: same input → same output,
/// always. Stochastic policies are out of scope (they'd need explicit
/// RNG state management, and onboard-safety-critical inference
/// shouldn't be stochastic anyway).
pub trait InferenceBackend {
    type Error: core::fmt::Debug;
    fn predict(&self, features: &FeatureVector) -> Result<Residual, Self::Error>;
}

// ---- Reference backend: affine ------------------------------------------

/// Single-layer affine model `y = W · x + b` with clamped output.
///
/// Serves as both a reference implementation and a cheap baseline
/// "NN" — if the policy-plus-controller integration works with this
/// minimal model, swapping in a real trained network is a narrow
/// change (new backend, same trait).
#[derive(Clone, Copy, Debug)]
pub struct AffineBackend {
    weights: [[f32; FEATURE_LEN]; RESIDUAL_LEN],
    bias: [f32; RESIDUAL_LEN],
    /// Per-axis output clamp. Applied after the affine transform so the
    /// backend itself can enforce a sanity range; the outer
    /// `SafetyEnvelope` is a second line of defence.
    output_clamp: f32,
}

impl AffineBackend {
    #[must_use]
    pub const fn new(
        weights: [[f32; FEATURE_LEN]; RESIDUAL_LEN],
        bias: [f32; RESIDUAL_LEN],
        output_clamp: f32,
    ) -> Self {
        Self {
            weights,
            bias,
            output_clamp,
        }
    }

    /// Zero-mean "do nothing" model. Useful baseline for SITL tests —
    /// a residual policy that outputs zeros leaves the NMPC command
    /// untouched, and the vehicle flies exactly as without the NN.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            weights: [[0.0; FEATURE_LEN]; RESIDUAL_LEN],
            bias: [0.0; RESIDUAL_LEN],
            output_clamp: 0.0,
        }
    }
}

/// Errors the affine backend can surface. Empty today; `Infallible`
/// would be more exact but an enum future-proofs the trait impl for
/// when we add e.g. shape-mismatch checking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AffineError {}

impl InferenceBackend for AffineBackend {
    type Error = AffineError;

    fn predict(&self, features: &FeatureVector) -> Result<Residual, Self::Error> {
        let mut out = [0.0_f32; RESIDUAL_LEN];
        for (i, out_i) in out.iter_mut().enumerate().take(RESIDUAL_LEN) {
            let row = self.weights.get(i).copied().unwrap_or([0.0; FEATURE_LEN]);
            let mut acc = self.bias.get(i).copied().unwrap_or(0.0);
            for (j, f) in features.0.iter().enumerate() {
                acc += row.get(j).copied().unwrap_or(0.0) * f;
            }
            if self.output_clamp > 0.0 {
                acc = acc.clamp(-self.output_clamp, self.output_clamp);
            }
            *out_i = acc;
        }
        Ok(Residual(Vector3::new(
            out.first().copied().unwrap_or(0.0),
            out.get(1).copied().unwrap_or(0.0),
            out.get(2).copied().unwrap_or(0.0),
        )))
    }
}

// ---- Safety envelope ------------------------------------------------------

/// Bounds defining the "known-safe" dynamic envelope. Any NN output
/// whose effect takes the vehicle outside this envelope is rejected.
#[derive(Clone, Copy, Debug)]
pub struct SafetyEnvelope {
    pub max_tilt_rad: f32,
    pub max_rate_rad_s: Vector3<f32>,
    pub max_accel_m_s2: f32,
    pub max_velocity_m_s: f32,
}

impl SafetyEnvelope {
    /// Reasonable default for a 250 g inspection multirotor:
    /// * 35° max tilt (avoids loss of lift)
    /// * 10 rad/s per axis (2× aerobatic typical)
    /// * 8 m/s² accel magnitude
    /// * 15 m/s velocity magnitude
    #[must_use]
    pub const fn small_multirotor_default() -> Self {
        Self {
            max_tilt_rad: 0.610_865, // ≈ 35°
            max_rate_rad_s: Vector3::new(10.0, 10.0, 10.0),
            max_accel_m_s2: 8.0,
            max_velocity_m_s: 15.0,
        }
    }

    /// Check a residual accel against the envelope **in the context
    /// of the current vehicle state** — an otherwise-safe residual
    /// magnitude may still be rejected if adding it to the current
    /// velocity would put the vehicle outside `max_velocity_m_s`.
    pub fn check(
        &self,
        residual: Residual,
        current_velocity_ned: Vector3<f32>,
    ) -> Result<Residual, EnvelopeReject> {
        let mag = residual.0.norm();
        if !mag.is_finite() {
            return Err(EnvelopeReject::NonFinite);
        }
        if mag > self.max_accel_m_s2 {
            return Err(EnvelopeReject::AccelTooLarge {
                got: mag,
                limit: self.max_accel_m_s2,
            });
        }
        let projected_velocity = current_velocity_ned + residual.0 * 0.1; // 100 ms horizon
        if projected_velocity.norm() > self.max_velocity_m_s {
            return Err(EnvelopeReject::VelocityExceeded {
                projected: projected_velocity.norm(),
                limit: self.max_velocity_m_s,
            });
        }
        Ok(residual)
    }
}

/// Typed reasons the safety envelope rejected a residual.
///
/// Concrete variants let the policy harness surface specific failure
/// modes (logs, metrics, STATUSTEXT) without flattening them into
/// "rejected".
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EnvelopeReject {
    NonFinite,
    AccelTooLarge { got: f32, limit: f32 },
    VelocityExceeded { projected: f32, limit: f32 },
}

// ---- Policy harness -------------------------------------------------------

/// Binds an inference backend to a safety envelope.
///
/// This is the type the application-layer code holds. `.predict()`
/// runs the backend, then the envelope check; either can reject.
/// Fallback behaviour (e.g. "just use zero residual on reject") is
/// the caller's call — the policy surfaces typed errors, not silent
/// zeros, so policy drop-outs are observable.
pub struct ResidualPolicy<B: InferenceBackend> {
    backend: B,
    envelope: SafetyEnvelope,
    /// Rolling count of rejections; bumped by `.predict()`. The caller
    /// logs / forwards this via the health topic.
    reject_count: u32,
}

impl<B: InferenceBackend> core::fmt::Debug for ResidualPolicy<B> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ResidualPolicy")
            .field("envelope", &self.envelope)
            .field("reject_count", &self.reject_count)
            .finish()
    }
}

impl<B: InferenceBackend> ResidualPolicy<B> {
    pub fn new(backend: B, envelope: SafetyEnvelope) -> Self {
        Self {
            backend,
            envelope,
            reject_count: 0,
        }
    }

    /// Query the policy. Returns `Ok(residual)` when both inference
    /// and the envelope check succeed, `Err` otherwise. On `Err` the
    /// internal reject counter is incremented.
    pub fn predict(
        &mut self,
        features: &FeatureVector,
        current_velocity_ned: Vector3<f32>,
    ) -> Result<Residual, PolicyError<B::Error>> {
        let raw = self
            .backend
            .predict(features)
            .map_err(PolicyError::Inference)?;
        match self.envelope.check(raw, current_velocity_ned) {
            Ok(r) => Ok(r),
            Err(reject) => {
                self.reject_count = self.reject_count.saturating_add(1);
                Err(PolicyError::Envelope(reject))
            }
        }
    }

    #[must_use]
    pub const fn reject_count(&self) -> u32 {
        self.reject_count
    }

    pub fn reset_reject_count(&mut self) {
        self.reject_count = 0;
    }

    #[must_use]
    pub const fn envelope(&self) -> &SafetyEnvelope {
        &self.envelope
    }
}

/// Unified error surfaced by `ResidualPolicy::predict`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PolicyError<E> {
    Inference(E),
    Envelope(EnvelopeReject),
}

// ---- Features buffer helpers ----------------------------------------------

/// Scratch buffer for batched inference calls. Today's pipeline runs
/// one sample per control tick, but future work (e.g. ensemble
/// predictions or dropout averaging) may query a batch.
pub type FeatureBuf = Vec<FeatureVector, 8>;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn feature_vector_from_state_ordering() {
        let f = FeatureVector::from_state(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            Vector3::new(7.0, 8.0, 9.0),
        );
        assert_eq!(f.0, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn affine_zero_backend_outputs_zero() {
        let b = AffineBackend::zero();
        let r = b
            .predict(&FeatureVector::from_state(
                Vector3::new(5.0, -3.0, 0.0),
                Vector3::zeros(),
                Vector3::zeros(),
            ))
            .unwrap();
        assert_eq!(r, Residual::zeros());
    }

    #[test]
    fn affine_backend_applies_weights_and_bias() {
        // Diagonal mapping: pos_err_x → out_0, vel_x → out_1,
        // roll → out_2. Plus per-output bias.
        let mut w = [[0.0_f32; FEATURE_LEN]; RESIDUAL_LEN];
        w[0][0] = 2.0; // pos_err_x
        w[1][3] = 0.5; // vel_x
        w[2][6] = -1.0; // roll
        let b = AffineBackend::new(w, [0.1, -0.2, 0.3], 0.0);
        let feat = FeatureVector::from_state(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(0.5, 0.0, 0.0),
        );
        let r = b.predict(&feat).unwrap().0;
        assert!((r.x - (2.0 * 1.0 + 0.1)).abs() < 1.0e-5);
        assert!((r.y - (0.5 * 4.0 - 0.2)).abs() < 1.0e-5);
        assert!((r.z - (-0.5 + 0.3)).abs() < 1.0e-5);
    }

    #[test]
    fn affine_backend_is_deterministic() {
        let w = [[1.0; FEATURE_LEN]; RESIDUAL_LEN];
        let b = AffineBackend::new(w, [0.0; RESIDUAL_LEN], 0.0);
        let feat = FeatureVector::from_state(
            Vector3::new(1.0, -0.5, 0.3),
            Vector3::new(0.1, 0.2, -0.1),
            Vector3::new(0.0, 0.05, 1.0),
        );
        let r1 = b.predict(&feat).unwrap();
        let r2 = b.predict(&feat).unwrap();
        let r3 = b.predict(&feat).unwrap();
        assert_eq!(r1, r2);
        assert_eq!(r2, r3);
    }

    #[test]
    fn affine_backend_clamps_output() {
        // Weight 100 on a +1 input → 100 * 1 = 100; clamp at 5.
        let mut w = [[0.0_f32; FEATURE_LEN]; RESIDUAL_LEN];
        w[0][0] = 100.0;
        let b = AffineBackend::new(w, [0.0; RESIDUAL_LEN], 5.0);
        let feat = FeatureVector::from_state(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let r = b.predict(&feat).unwrap().0;
        assert_eq!(r.x, 5.0);
    }

    #[test]
    fn envelope_accepts_safe_residual() {
        let env = SafetyEnvelope::small_multirotor_default();
        let r = Residual(Vector3::new(1.0, 0.5, -2.0));
        assert!(env.check(r, Vector3::zeros()).is_ok());
    }

    #[test]
    fn envelope_rejects_non_finite() {
        let env = SafetyEnvelope::small_multirotor_default();
        let r = Residual(Vector3::new(f32::NAN, 0.0, 0.0));
        assert_eq!(
            env.check(r, Vector3::zeros()),
            Err(EnvelopeReject::NonFinite)
        );
    }

    #[test]
    fn envelope_rejects_too_large_accel() {
        let env = SafetyEnvelope::small_multirotor_default();
        let r = Residual(Vector3::new(20.0, 0.0, 0.0));
        assert!(matches!(
            env.check(r, Vector3::zeros()),
            Err(EnvelopeReject::AccelTooLarge { .. })
        ));
    }

    #[test]
    fn envelope_rejects_velocity_runaway() {
        let env = SafetyEnvelope::small_multirotor_default();
        // Small residual but vehicle already fast.
        let r = Residual(Vector3::new(2.0, 0.0, 0.0));
        let v = Vector3::new(16.0, 0.0, 0.0);
        assert!(matches!(
            env.check(r, v),
            Err(EnvelopeReject::VelocityExceeded { .. })
        ));
    }

    #[test]
    fn residual_policy_zero_backend_accepted() {
        let mut p = ResidualPolicy::new(
            AffineBackend::zero(),
            SafetyEnvelope::small_multirotor_default(),
        );
        let out = p
            .predict(&FeatureVector::zeros(), Vector3::zeros())
            .unwrap();
        assert_eq!(out, Residual::zeros());
        assert_eq!(p.reject_count(), 0);
    }

    #[test]
    fn residual_policy_counts_rejections() {
        let mut w = [[0.0_f32; FEATURE_LEN]; RESIDUAL_LEN];
        w[0][0] = 100.0; // large weight → large residual
        let backend = AffineBackend::new(w, [0.0; RESIDUAL_LEN], 0.0);
        let mut p = ResidualPolicy::new(backend, SafetyEnvelope::small_multirotor_default());
        let feat = FeatureVector::from_state(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        // First call: residual x=100 > 8 → reject.
        let r = p.predict(&feat, Vector3::zeros());
        assert!(matches!(r, Err(PolicyError::Envelope(_))));
        assert_eq!(p.reject_count(), 1);
        // Second call bumps again.
        let _ = p.predict(&feat, Vector3::zeros());
        assert_eq!(p.reject_count(), 2);
        // Reset clears the counter.
        p.reset_reject_count();
        assert_eq!(p.reject_count(), 0);
    }
}
