#![no_std]

//! Task scheduling primitives.
//!
//! The concrete scheduler (embassy-executor or Hubris) plugs in behind these
//! abstractions so algorithm code is unaware of which runtime owns it.

/// Priority levels used across the system. Lower number = higher priority.
///
/// The ordering is **total**: any two priorities are comparable and the
/// comparison is transitive + antisymmetric. See `kani_proofs` below for
/// machine-checked proofs of these properties.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    RateLoop = 0,
    AttitudeLoop = 1,
    Estimator = 2,
    PositionLoop = 3,
    Navigation = 4,
    Telemetry = 5,
    Logging = 6,
    Background = 7,
}

impl Priority {
    /// All priority variants in ascending (strictly-increasing) order.
    /// Used by proofs to enumerate the finite domain.
    pub const ALL: [Priority; 8] = [
        Priority::RateLoop,
        Priority::AttitudeLoop,
        Priority::Estimator,
        Priority::PositionLoop,
        Priority::Navigation,
        Priority::Telemetry,
        Priority::Logging,
        Priority::Background,
    ];

    /// Numeric rank (0 = highest, 7 = lowest).
    #[must_use]
    pub const fn rank(self) -> u8 {
        // Explicit match avoids the `as` cast lint without any runtime cost.
        match self {
            Priority::RateLoop => 0,
            Priority::AttitudeLoop => 1,
            Priority::Estimator => 2,
            Priority::PositionLoop => 3,
            Priority::Navigation => 4,
            Priority::Telemetry => 5,
            Priority::Logging => 6,
            Priority::Background => 7,
        }
    }

    /// Build a `Priority` from its rank. Returns `None` for out-of-range
    /// values so callers don't need a panic path.
    #[must_use]
    pub const fn from_rank(r: u8) -> Option<Priority> {
        match r {
            0 => Some(Priority::RateLoop),
            1 => Some(Priority::AttitudeLoop),
            2 => Some(Priority::Estimator),
            3 => Some(Priority::PositionLoop),
            4 => Some(Priority::Navigation),
            5 => Some(Priority::Telemetry),
            6 => Some(Priority::Logging),
            7 => Some(Priority::Background),
            _ => None,
        }
    }
}

/// Fixed-period task descriptor used for schedule analysis and WCET budgets.
#[derive(Clone, Copy, Debug)]
pub struct TaskSpec {
    pub name: &'static str,
    pub priority: Priority,
    /// Nominal period in microseconds.
    pub period_us: u32,
    /// Budgeted worst-case execution time in microseconds.
    pub wcet_budget_us: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_ranks_round_trip() {
        for p in Priority::ALL {
            assert_eq!(Priority::from_rank(p.rank()), Some(p));
        }
    }

    #[test]
    fn ranks_are_strictly_increasing() {
        for (a, b) in Priority::ALL.iter().zip(Priority::ALL.iter().skip(1)) {
            assert!(a.rank() < b.rank());
            assert!(a < b);
        }
    }

    #[test]
    fn out_of_range_rank_is_none() {
        for r in 8u8..=255u8 {
            assert_eq!(Priority::from_rank(r), None);
        }
    }

    #[test]
    fn ordering_matches_rank() {
        for a in Priority::ALL {
            for b in Priority::ALL {
                assert_eq!(a.cmp(&b), a.rank().cmp(&b.rank()));
            }
        }
    }
}

// ============================================================================
// Kani formal proofs.
//
// These exhaustively cover the 8-variant Priority enum — symbolic values are
// finite and tiny, so CBMC closes every harness in milliseconds.
// ============================================================================
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    fn any_priority() -> Priority {
        let r: u8 = kani::any();
        kani::assume(r < 8);
        Priority::from_rank(r).unwrap()
    }

    /// Totality: every pair of priorities is comparable.
    #[kani::proof]
    fn check_ordering_is_total() {
        let a = any_priority();
        let b = any_priority();
        // `Ord::cmp` always returns a value (never panics), so the mere
        // ability to call it on any two priorities proves totality.
        let _ = a.cmp(&b);
        assert!(a < b || a == b || a > b);
    }

    /// Transitivity: a ≤ b ∧ b ≤ c ⇒ a ≤ c.
    #[kani::proof]
    fn check_ordering_is_transitive() {
        let a = any_priority();
        let b = any_priority();
        let c = any_priority();
        if a <= b && b <= c {
            assert!(a <= c);
        }
    }

    /// Antisymmetry: a ≤ b ∧ b ≤ a ⇒ a == b.
    #[kani::proof]
    fn check_ordering_is_antisymmetric() {
        let a = any_priority();
        let b = any_priority();
        if a <= b && b <= a {
            assert!(a == b);
        }
    }

    /// Rank round-trip: `from_rank(p.rank()) == Some(p)` for every variant.
    #[kani::proof]
    fn check_rank_round_trip() {
        let p = any_priority();
        assert_eq!(Priority::from_rank(p.rank()), Some(p));
    }

    /// Out-of-range rank yields `None` — proves no panic branch exists.
    #[kani::proof]
    fn check_out_of_range_rank_is_none() {
        let r: u8 = kani::any();
        kani::assume(r >= 8);
        assert!(Priority::from_rank(r).is_none());
    }
}
