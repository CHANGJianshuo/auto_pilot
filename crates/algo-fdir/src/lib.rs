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

#[cfg(test)]
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
}
