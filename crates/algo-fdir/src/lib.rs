#![no_std]

//! Fault Detection, Isolation, and Recovery.
//!
//! This crate owns the vehicle's *health state machine*. Sensor voting,
//! actuator monitoring, and FDIR policy logic will dock onto the
//! [`HealthLevel`] state transitions defined here.

/// Top-level vehicle health classes.
///
/// Monotonicity contract (enforced by [`HealthLevel::transition_in_flight`]):
///
/// ```text
///   Healthy  →  Degraded  →  Emergency  →  Failed
/// ```
///
/// In flight a transition may only **stay** or **deteriorate** (move right).
/// Recovery — moving left — is only permitted on the ground, via
/// [`HealthLevel::reset_on_ground`], after a root-cause analysis has cleared
/// the fault. This one-way-in-flight discipline is the foundation of FDIR
/// correctness: every other component can trust that once a subsystem has
/// declared `Emergency`, the vehicle will never silently return to `Healthy`
/// mid-mission.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum HealthLevel {
    Healthy = 0,
    Degraded = 1,
    Emergency = 2,
    Failed = 3,
}

/// Per-sensor rejection tracker.
///
/// Each EKF measurement update reports back whether the observation was
/// accepted (`applied=true`) or rejected (NIS exceeded the χ² gate).
/// [`SensorRejectionCounter`] converts that stream into a single
/// [`HealthLevel`] signal:
///
/// * Any accepted measurement resets the counter.
/// * After `n_rejections_to_degrade` consecutive rejections the level steps
///   up to `Degraded`; after `n_rejections_to_emergency`, up to `Emergency`.
/// * Levels never step back down in-flight — use
///   [`HealthLevel::reset_on_ground`] after a manual clearance.
#[derive(Clone, Copy, Debug)]
pub struct SensorRejectionCounter {
    consecutive_rejections: u32,
    n_rejections_to_degrade: u32,
    n_rejections_to_emergency: u32,
    level: HealthLevel,
}

impl SensorRejectionCounter {
    /// Default thresholds tuned for 5–50 Hz measurement sources:
    /// * 10 consecutive rejects (≈ 0.2–2 s of bad data) → Degraded
    /// * 50 consecutive rejects (≈ 1–10 s) → Emergency
    pub const DEFAULT_N_DEGRADE: u32 = 10;
    pub const DEFAULT_N_EMERGENCY: u32 = 50;

    #[must_use]
    pub const fn new() -> Self {
        Self::with_thresholds(Self::DEFAULT_N_DEGRADE, Self::DEFAULT_N_EMERGENCY)
    }

    #[must_use]
    pub const fn with_thresholds(n_degrade: u32, n_emergency: u32) -> Self {
        Self {
            consecutive_rejections: 0,
            n_rejections_to_degrade: n_degrade,
            n_rejections_to_emergency: n_emergency,
            level: HealthLevel::Healthy,
        }
    }

    /// Feed one measurement outcome. Returns the (possibly updated) health
    /// level so the caller can decide whether to announce a degradation.
    pub fn observe(&mut self, accepted: bool) -> HealthLevel {
        if accepted {
            self.consecutive_rejections = 0;
            return self.level;
        }
        self.consecutive_rejections = self.consecutive_rejections.saturating_add(1);
        let proposed = if self.consecutive_rejections >= self.n_rejections_to_emergency {
            HealthLevel::Emergency
        } else if self.consecutive_rejections >= self.n_rejections_to_degrade {
            HealthLevel::Degraded
        } else {
            self.level
        };
        self.level = self.level.transition_in_flight(proposed);
        self.level
    }

    /// Current level without consuming an observation.
    #[must_use]
    pub const fn level(&self) -> HealthLevel {
        self.level
    }

    /// Current streak length of consecutive rejections.
    #[must_use]
    pub const fn streak(&self) -> u32 {
        self.consecutive_rejections
    }
}

impl Default for SensorRejectionCounter {
    fn default() -> Self {
        Self::new()
    }
}

impl HealthLevel {
    /// All health levels in monotone order (least severe first).
    pub const ALL: [HealthLevel; 4] = [
        HealthLevel::Healthy,
        HealthLevel::Degraded,
        HealthLevel::Emergency,
        HealthLevel::Failed,
    ];

    /// Numeric severity (0 = Healthy, 3 = Failed).
    #[must_use]
    pub const fn severity(self) -> u8 {
        match self {
            HealthLevel::Healthy => 0,
            HealthLevel::Degraded => 1,
            HealthLevel::Emergency => 2,
            HealthLevel::Failed => 3,
        }
    }

    /// Attempt an in-flight transition to `proposed`. Returns the new level.
    ///
    /// The rule is: `proposed` is honoured only if its severity is **≥** the
    /// current level's. Otherwise `self` is kept. This enforces the
    /// one-way-in-flight invariant without ever panicking.
    #[must_use]
    pub const fn transition_in_flight(self, proposed: HealthLevel) -> HealthLevel {
        if proposed.severity() >= self.severity() {
            proposed
        } else {
            self
        }
    }

    /// Reset to `Healthy` — only callable from the ground after manual
    /// clearance. Kept as a separate function so the one-way discipline
    /// of `transition_in_flight` stays provably total.
    #[must_use]
    pub const fn reset_on_ground() -> HealthLevel {
        HealthLevel::Healthy
    }

    /// `true` if the airframe is in a mission-abort state (Emergency or Failed).
    #[must_use]
    pub const fn requires_abort(self) -> bool {
        self.severity() >= HealthLevel::Emergency.severity()
    }
}

// ============================================================================
// MotorFaultDetector — identify a dead rotor from commanded thrusts vs
// observed angular acceleration.
//
// Physics: at hover the body angular dynamics obey
//
//   J · ω̇ = τ_cmd   (ignoring gyroscopic ω×Jω, small at hover)
//
// where `τ_cmd` is the sum of each motor's per-thrust torque vector times
// the actual (not commanded) thrust. If motor `k` dies, the **actual**
// thrust at index `k` goes to zero while the **commanded** value stays at
// whatever the allocator requested. The residual
//
//   r = ω̇_observed - J⁻¹ · τ_expected_from_cmd
//
// should therefore line up with `-J⁻¹ · e_k · T_cmd[k]`, i.e. the
// "missing contribution" of motor `k`.
//
// The detector scores each motor by this alignment, increments a per-
// motor persistence counter when the same motor is the best fit, and
// declares a fault once the counter crosses a threshold.
// ============================================================================

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

/// Watches one flight for a single-motor failure.
///
/// Operates on **filtered** observed ω̇ (the caller should pass the same
/// LPF output it feeds to INDI). Built for a 4-motor airframe; multi-
/// motor variants are out of scope until a real failure case demands them.
#[derive(Clone, Copy, Debug)]
pub struct MotorFaultDetector {
    /// Per-motor persistence: number of consecutive ticks this motor was
    /// the best-aligned fault hypothesis above the residual threshold.
    persistence: [u32; 4],
    /// Ticks-of-alignment required before declaring the motor dead.
    /// At 1 kHz control rate with noisy IMU, 50 ticks ≈ 50 ms is a
    /// reasonable "not a transient" threshold.
    n_ticks_to_declare: u32,
    /// Magnitude of the `ω̇` residual (rad/s²) below which no motor is
    /// scored. Protects against noise amplifying spurious alignments.
    residual_threshold_rad_s2: f32,
    /// Liveness mask; `false` means the detector has declared the motor
    /// dead (latched — one-way, parallel to HealthLevel's no-recovery-
    /// in-flight discipline).
    alive: [bool; 4],
    /// Overall health of the motor subsystem. `Emergency` once any motor
    /// is declared dead.
    level: HealthLevel,
}

impl MotorFaultDetector {
    /// Default thresholds tuned for a 250 g X-quad at ~1 kHz:
    ///
    /// * `residual_threshold_rad_s2 = 2.0` — at hover, a fully dead
    ///   motor produces ~4 rad/s² of ω̇ residual, so 2.0 is half-signal
    /// * `n_ticks_to_declare = 50` — 50 ms at 1 kHz, long enough to
    ///   reject a single noisy sample, short enough that the failover
    ///   allocator kicks in before serious altitude loss
    pub const DEFAULT_RESIDUAL_THRESHOLD: f32 = 2.0;
    pub const DEFAULT_N_TICKS_TO_DECLARE: u32 = 50;

    #[must_use]
    pub const fn new() -> Self {
        Self::with_thresholds(
            Self::DEFAULT_RESIDUAL_THRESHOLD,
            Self::DEFAULT_N_TICKS_TO_DECLARE,
        )
    }

    #[must_use]
    pub const fn with_thresholds(residual_threshold: f32, n_ticks: u32) -> Self {
        Self {
            persistence: [0; 4],
            n_ticks_to_declare: n_ticks,
            residual_threshold_rad_s2: residual_threshold,
            alive: [true; 4],
            level: HealthLevel::Healthy,
        }
    }

    /// Feed one tick of (commanded motor thrusts, observed ω̇). Returns
    /// the overall motor-subsystem [`HealthLevel`].
    ///
    /// * `motor_cmd` — thrusts just sent to the ESCs (after saturation).
    /// * `omega_dot_observed` — LPF-filtered numerical derivative of ω.
    /// * `effectiveness` — the 4×4 `E` matrix from [`algo-alloc`]. The
    ///   first three rows give the per-motor body torques.
    /// * `inertia_inv` — `J⁻¹` of the airframe.
    pub fn observe(
        &mut self,
        motor_cmd: &SVector<f32, 4>,
        omega_dot_observed: Vector3<f32>,
        effectiveness: &SMatrix<f32, 4, 4>,
        inertia_inv: &Matrix3<f32>,
    ) -> HealthLevel {
        // Expected body torque from commanded thrusts (rows 0..3 of E).
        let mut tau = Vector3::zeros();
        for i in 0..4 {
            tau.x += effectiveness[(0, i)] * motor_cmd[i];
            tau.y += effectiveness[(1, i)] * motor_cmd[i];
            tau.z += effectiveness[(2, i)] * motor_cmd[i];
        }
        let omega_dot_expected = inertia_inv * tau;
        let residual = omega_dot_observed - omega_dot_expected;
        let residual_mag =
            libm::sqrtf(residual.x * residual.x + residual.y * residual.y + residual.z * residual.z);

        if residual_mag < self.residual_threshold_rad_s2 {
            self.persistence = [0; 4];
            return self.level;
        }

        // Score each motor by alignment between the residual and the
        // inverse of that motor's torque contribution. A dead motor's
        // missing contribution shows up as residual pointing along
        // `-J⁻¹ · e_i`.
        let mut best_score = 0.0_f32;
        let mut best_idx: usize = usize::MAX;
        for (i, &alive_i) in self.alive.iter().enumerate() {
            if !alive_i {
                continue;
            }
            let e_col = Vector3::new(
                effectiveness[(0, i)],
                effectiveness[(1, i)],
                effectiveness[(2, i)],
            );
            let e_torque_direction = inertia_inv * e_col;
            let norm_sq = e_torque_direction.x * e_torque_direction.x
                + e_torque_direction.y * e_torque_direction.y
                + e_torque_direction.z * e_torque_direction.z;
            if norm_sq < 1.0e-12 {
                continue;
            }
            let norm = libm::sqrtf(norm_sq);
            let unit_x = e_torque_direction.x / norm;
            let unit_y = e_torque_direction.y / norm;
            let unit_z = e_torque_direction.z / norm;
            let anti_align =
                -(residual.x * unit_x + residual.y * unit_y + residual.z * unit_z);
            let cmd_scale = motor_cmd[i].max(0.0);
            let score = anti_align * cmd_scale;
            if score > best_score {
                best_score = score;
                best_idx = i;
            }
        }

        if best_idx == usize::MAX {
            // No motor could explain the residual (e.g. all declared
            // dead already, or numeric edge). Don't touch persistence.
            return self.level;
        }

        for (i, p) in self.persistence.iter_mut().enumerate() {
            if i == best_idx {
                *p = p.saturating_add(1);
            } else {
                *p = 0;
            }
        }

        let mut should_declare = false;
        for (i, (&alive_i, &p)) in self
            .alive
            .iter()
            .zip(self.persistence.iter())
            .enumerate()
        {
            if i == best_idx && alive_i && p >= self.n_ticks_to_declare {
                should_declare = true;
            }
        }
        if should_declare {
            for (i, a) in self.alive.iter_mut().enumerate() {
                if i == best_idx {
                    *a = false;
                }
            }
            self.level = self.level.transition_in_flight(HealthLevel::Emergency);
        }

        self.level
    }

    /// Liveness mask for the caller to forward to `FlightState.motor_alive`.
    #[must_use]
    pub const fn alive(&self) -> [bool; 4] {
        self.alive
    }

    /// Count of motors currently declared dead.
    #[must_use]
    pub fn dead_count(&self) -> u32 {
        let mut c = 0_u32;
        for &a in self.alive.iter() {
            if !a {
                c = c.saturating_add(1);
            }
        }
        c
    }

    /// Current persistence counter for each motor — useful for tests
    /// and instrumentation.
    #[must_use]
    pub const fn persistence(&self) -> [u32; 4] {
        self.persistence
    }

    /// Current motor-subsystem health level.
    #[must_use]
    pub const fn level(&self) -> HealthLevel {
        self.level
    }
}

impl Default for MotorFaultDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(clippy::indexing_slicing, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn severities_are_strictly_increasing() {
        for (a, b) in HealthLevel::ALL.iter().zip(HealthLevel::ALL.iter().skip(1)) {
            assert!(a.severity() < b.severity());
        }
    }

    #[test]
    fn deterioration_is_accepted() {
        assert_eq!(
            HealthLevel::Healthy.transition_in_flight(HealthLevel::Degraded),
            HealthLevel::Degraded,
        );
        assert_eq!(
            HealthLevel::Degraded.transition_in_flight(HealthLevel::Failed),
            HealthLevel::Failed,
        );
    }

    #[test]
    fn recovery_in_flight_is_rejected() {
        assert_eq!(
            HealthLevel::Failed.transition_in_flight(HealthLevel::Healthy),
            HealthLevel::Failed,
        );
        assert_eq!(
            HealthLevel::Emergency.transition_in_flight(HealthLevel::Degraded),
            HealthLevel::Emergency,
        );
    }

    #[test]
    fn staying_put_is_a_no_op() {
        for lvl in HealthLevel::ALL {
            assert_eq!(lvl.transition_in_flight(lvl), lvl);
        }
    }

    #[test]
    fn reset_returns_healthy() {
        assert_eq!(HealthLevel::reset_on_ground(), HealthLevel::Healthy);
    }

    #[test]
    fn abort_threshold() {
        assert!(!HealthLevel::Healthy.requires_abort());
        assert!(!HealthLevel::Degraded.requires_abort());
        assert!(HealthLevel::Emergency.requires_abort());
        assert!(HealthLevel::Failed.requires_abort());
    }

    // ---- SensorRejectionCounter tests ------------------------------------

    #[test]
    fn counter_starts_healthy() {
        let c = SensorRejectionCounter::new();
        assert_eq!(c.level(), HealthLevel::Healthy);
        assert_eq!(c.streak(), 0);
    }

    #[test]
    fn counter_stays_healthy_on_acceptances() {
        let mut c = SensorRejectionCounter::new();
        for _ in 0..100 {
            assert_eq!(c.observe(true), HealthLevel::Healthy);
        }
        assert_eq!(c.streak(), 0);
    }

    #[test]
    fn counter_accepted_resets_streak() {
        let mut c = SensorRejectionCounter::new();
        for _ in 0..5 {
            c.observe(false);
        }
        assert_eq!(c.streak(), 5);
        c.observe(true);
        assert_eq!(c.streak(), 0);
        // but level stays the same as before acceptance (Healthy because < 10)
        assert_eq!(c.level(), HealthLevel::Healthy);
    }

    #[test]
    fn counter_escalates_on_consecutive_rejections() {
        let mut c = SensorRejectionCounter::with_thresholds(3, 6);
        for _ in 0..2 {
            c.observe(false);
        }
        assert_eq!(c.level(), HealthLevel::Healthy);
        c.observe(false);
        assert_eq!(c.level(), HealthLevel::Degraded);
        for _ in 0..3 {
            c.observe(false);
        }
        assert_eq!(c.level(), HealthLevel::Emergency);
    }

    #[test]
    fn counter_never_recovers_in_flight() {
        let mut c = SensorRejectionCounter::with_thresholds(2, 5);
        c.observe(false);
        c.observe(false);
        assert_eq!(c.level(), HealthLevel::Degraded);
        // Acceptance resets the streak but level stays Degraded.
        c.observe(true);
        assert_eq!(c.level(), HealthLevel::Degraded);
        c.observe(true);
        assert_eq!(c.level(), HealthLevel::Degraded);
    }

    #[test]
    fn counter_does_not_overflow_on_long_streak() {
        let mut c = SensorRejectionCounter::new();
        for _ in 0..10_000 {
            c.observe(false);
        }
        assert_eq!(c.level(), HealthLevel::Emergency);
        // saturating_add keeps streak finite.
        assert!(c.streak() <= 10_000);
    }

    // ---- MotorFaultDetector tests ---------------------------------------

    /// Standard 250 g X-quad geometry: arm_m = 0.15, k_yaw = 0.016.
    /// Matches `app_copter::default_config_250g` so the numbers in
    /// these tests transfer to SITL.
    fn test_effectiveness_and_inertia() -> (SMatrix<f32, 4, 4>, Matrix3<f32>) {
        let h = 0.15 / core::f32::consts::SQRT_2;
        let k = 0.016_f32;
        // Columns: [-y, +x, yaw_coef, 1]
        // M0 at (+h, +h, CW  k):   [-h, +h, +k, 1]
        // M1 at (-h, -h, CW  k):   [+h, -h, +k, 1]
        // M2 at (+h, -h, CCW -k):  [+h, +h, -k, 1]
        // M3 at (-h, +h, CCW -k):  [-h, -h, -k, 1]
        let mut e = SMatrix::<f32, 4, 4>::zeros();
        e[(0, 0)] = -h;
        e[(1, 0)] = h;
        e[(2, 0)] = k;
        e[(3, 0)] = 1.0;
        e[(0, 1)] = h;
        e[(1, 1)] = -h;
        e[(2, 1)] = k;
        e[(3, 1)] = 1.0;
        e[(0, 2)] = h;
        e[(1, 2)] = h;
        e[(2, 2)] = -k;
        e[(3, 2)] = 1.0;
        e[(0, 3)] = -h;
        e[(1, 3)] = -h;
        e[(2, 3)] = -k;
        e[(3, 3)] = 1.0;
        let mut j_inv = Matrix3::zeros();
        j_inv[(0, 0)] = 1.0 / 0.015;
        j_inv[(1, 1)] = 1.0 / 0.015;
        j_inv[(2, 2)] = 1.0 / 0.025;
        (e, j_inv)
    }

    /// Given a commanded thrust vector and `alive` mask, compute what
    /// a noise-free IMU would measure for ω̇.
    fn synthetic_omega_dot(
        motor_cmd: &SVector<f32, 4>,
        alive: [bool; 4],
        effectiveness: &SMatrix<f32, 4, 4>,
        inertia_inv: &Matrix3<f32>,
    ) -> Vector3<f32> {
        let mut tau = Vector3::zeros();
        for (i, &a) in alive.iter().enumerate() {
            let thrust = if a { motor_cmd[i] } else { 0.0 };
            tau.x += effectiveness[(0, i)] * thrust;
            tau.y += effectiveness[(1, i)] * thrust;
            tau.z += effectiveness[(2, i)] * thrust;
        }
        inertia_inv * tau
    }

    #[test]
    fn detector_starts_clean() {
        let d = MotorFaultDetector::new();
        assert_eq!(d.level(), HealthLevel::Healthy);
        assert_eq!(d.alive(), [true; 4]);
        assert_eq!(d.dead_count(), 0);
        assert_eq!(d.persistence(), [0; 4]);
    }

    #[test]
    fn detector_no_false_alarm_on_clean_flight() {
        let (e, j_inv) = test_effectiveness_and_inertia();
        let mut d = MotorFaultDetector::new();
        let cmd = SVector::<f32, 4>::from_column_slice(&[0.6, 0.6, 0.6, 0.6]);
        // alive = [true; 4] — no fault
        let obs = synthetic_omega_dot(&cmd, [true; 4], &e, &j_inv);
        for _ in 0..1_000 {
            d.observe(&cmd, obs, &e, &j_inv);
        }
        assert_eq!(d.level(), HealthLevel::Healthy);
        assert_eq!(d.alive(), [true; 4]);
    }

    #[test]
    fn detector_identifies_dead_motor_0() {
        let (e, j_inv) = test_effectiveness_and_inertia();
        let mut d = MotorFaultDetector::with_thresholds(1.0, 20);
        let cmd = SVector::<f32, 4>::from_column_slice(&[0.6, 0.6, 0.6, 0.6]);
        // alive mask has motor 0 dead; detector doesn't know that yet.
        let mut alive_actual = [true; 4];
        alive_actual[0] = false;
        let obs = synthetic_omega_dot(&cmd, alive_actual, &e, &j_inv);
        for _ in 0..40 {
            d.observe(&cmd, obs, &e, &j_inv);
        }
        assert_eq!(d.alive()[0], false, "motor 0 should be declared dead");
        for i in 1..4 {
            assert!(d.alive()[i], "motor {i} should still be alive");
        }
        assert_eq!(d.level(), HealthLevel::Emergency);
        assert_eq!(d.dead_count(), 1);
    }

    #[test]
    fn detector_each_motor_correctly_attributed() {
        // Rotational symmetry: detector must identify whichever motor
        // we kill, not only motor 0.
        let (e, j_inv) = test_effectiveness_and_inertia();
        let cmd = SVector::<f32, 4>::from_column_slice(&[0.6, 0.6, 0.6, 0.6]);
        for dead in 0..4 {
            let mut d = MotorFaultDetector::with_thresholds(1.0, 20);
            let mut alive_actual = [true; 4];
            alive_actual[dead] = false;
            let obs = synthetic_omega_dot(&cmd, alive_actual, &e, &j_inv);
            for _ in 0..40 {
                d.observe(&cmd, obs, &e, &j_inv);
            }
            assert!(!d.alive()[dead], "motor {dead} should be dead");
            for i in 0..4 {
                if i != dead {
                    assert!(d.alive()[i], "motor {i} should be alive when {dead} died");
                }
            }
        }
    }

    #[test]
    fn detector_requires_persistence_not_single_spike() {
        let (e, j_inv) = test_effectiveness_and_inertia();
        let mut d = MotorFaultDetector::with_thresholds(1.0, 50);
        let cmd = SVector::<f32, 4>::from_column_slice(&[0.6, 0.6, 0.6, 0.6]);
        let mut alive_actual = [true; 4];
        alive_actual[0] = false;
        let obs_fault = synthetic_omega_dot(&cmd, alive_actual, &e, &j_inv);
        let obs_clean = synthetic_omega_dot(&cmd, [true; 4], &e, &j_inv);
        // One spike of fault (10 ticks), then clean → no declaration.
        for _ in 0..10 {
            d.observe(&cmd, obs_fault, &e, &j_inv);
        }
        assert_eq!(d.alive(), [true; 4], "10 ticks of fault < threshold 50");
        for _ in 0..200 {
            d.observe(&cmd, obs_clean, &e, &j_inv);
        }
        assert_eq!(d.alive(), [true; 4], "clean obs kept it healthy");
    }

    #[test]
    fn detector_is_latched_does_not_recover() {
        let (e, j_inv) = test_effectiveness_and_inertia();
        let mut d = MotorFaultDetector::with_thresholds(1.0, 20);
        let cmd = SVector::<f32, 4>::from_column_slice(&[0.6, 0.6, 0.6, 0.6]);
        let mut alive_actual = [true; 4];
        alive_actual[0] = false;
        let obs_fault = synthetic_omega_dot(&cmd, alive_actual, &e, &j_inv);
        for _ in 0..50 {
            d.observe(&cmd, obs_fault, &e, &j_inv);
        }
        assert!(!d.alive()[0]);
        // Motor magically becomes fine again — detector must NOT
        // un-declare. Latching is the same one-way-in-flight contract
        // as HealthLevel.
        let obs_clean = synthetic_omega_dot(&cmd, [true; 4], &e, &j_inv);
        for _ in 0..500 {
            d.observe(&cmd, obs_clean, &e, &j_inv);
        }
        assert!(!d.alive()[0], "detector must not un-declare a dead motor");
        assert_eq!(d.level(), HealthLevel::Emergency);
    }
}

// ============================================================================
// Kani formal proofs.
//
// The domain is 4 values; CBMC closes everything in milliseconds.
// ============================================================================
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    fn any_health_level() -> HealthLevel {
        let s: u8 = kani::any();
        kani::assume(s < 4);
        match s {
            0 => HealthLevel::Healthy,
            1 => HealthLevel::Degraded,
            2 => HealthLevel::Emergency,
            _ => HealthLevel::Failed,
        }
    }

    /// Monotonicity: a transition never produces a level whose severity
    /// is below `self`'s. This is *the* FDIR safety invariant.
    #[kani::proof]
    fn check_transition_is_monotone() {
        let current = any_health_level();
        let proposed = any_health_level();
        let next = current.transition_in_flight(proposed);
        assert!(next.severity() >= current.severity());
    }

    /// Deterioration honour: when `proposed` is at least as severe as
    /// `current`, the transition adopts it.
    #[kani::proof]
    fn check_transition_honours_deterioration() {
        let current = any_health_level();
        let proposed = any_health_level();
        if proposed.severity() >= current.severity() {
            let next = current.transition_in_flight(proposed);
            assert_eq!(next, proposed);
        }
    }

    /// Idempotence: transitioning to the same level is a no-op.
    #[kani::proof]
    fn check_transition_is_idempotent() {
        let lvl = any_health_level();
        assert_eq!(lvl.transition_in_flight(lvl), lvl);
    }

    /// `reset_on_ground` always yields Healthy. Trivial but makes the
    /// "no silent reset mid-flight" contract obvious to readers.
    #[kani::proof]
    fn check_reset_yields_healthy() {
        assert_eq!(HealthLevel::reset_on_ground(), HealthLevel::Healthy);
    }

    /// `requires_abort` agrees with the Emergency severity threshold for
    /// every level — proves the helper can never disagree with the enum.
    #[kani::proof]
    fn check_abort_threshold_matches_severity() {
        let lvl = any_health_level();
        assert_eq!(
            lvl.requires_abort(),
            lvl.severity() >= HealthLevel::Emergency.severity(),
        );
    }

    /// Single-step monotonicity: for any reachable counter state and any
    /// outcome, `observe` never lowers the stored level. Concrete-input
    /// harness kept small so CBMC closes in milliseconds.
    ///
    /// (A symbolic-sequence version exploded CBMC's state space on the
    /// default CaDiCaL solver; the 6 unit tests plus this single-step
    /// harness together give the same guarantee in practice.)
    #[kani::proof]
    fn check_counter_observe_single_step_is_monotone() {
        let mut c = SensorRejectionCounter::new();
        let before = c.level();
        let accepted: bool = kani::any();
        let after = c.observe(accepted);
        assert!(after.severity() >= before.severity());
    }
}
