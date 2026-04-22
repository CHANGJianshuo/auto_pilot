//! Minimal deterministic RNG for the simulator — no `rand` crate dep,
//! reproducible across runs, good enough for realism noise in SITL.
//!
//! * Xorshift64 for uniform `u32 / f32` draws.
//! * Box–Muller for `f32` Gaussian draws.

use nalgebra::Vector3;

/// Deterministic pseudo-random number generator seeded by a `u64`.
#[derive(Clone, Debug)]
pub struct SimRng {
    state: u64,
    /// Cached second Box–Muller value, `Some(z)` means use this on the
    /// next `gaussian()` call and clear.
    cached_gaussian: Option<f32>,
}

impl SimRng {
    /// Create with a seed. Seed 0 gets bumped to avoid the xorshift fixed
    /// point; otherwise any seed is fine.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        let state = if seed == 0 {
            0xDEAD_BEEF_CAFE_BABE
        } else {
            seed
        };
        Self {
            state,
            cached_gaussian: None,
        }
    }

    /// One step of xorshift64.
    fn step(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Uniform `u32`.
    pub fn next_u32(&mut self) -> u32 {
        #[allow(clippy::as_conversions)]
        let v = self.step() as u32;
        v
    }

    /// Uniform `f32` in `[0, 1)`.
    #[allow(clippy::as_conversions)]
    pub fn next_f32(&mut self) -> f32 {
        // 24-bit mantissa → use top 24 bits of u32 to avoid 1.0 ever.
        let u = self.next_u32() >> 8;
        (u as f32) / ((1u32 << 24) as f32)
    }

    /// Standard-normal `f32` via Box–Muller; caches the second value.
    pub fn gaussian(&mut self) -> f32 {
        if let Some(z) = self.cached_gaussian.take() {
            return z;
        }
        // Guard against `ln(0)` by re-rolling until u1 > 0.
        let mut u1 = self.next_f32();
        while u1 == 0.0 {
            u1 = self.next_f32();
        }
        let u2 = self.next_f32();
        let r = libm::sqrtf(-2.0 * libm::logf(u1));
        let theta = 2.0 * core::f32::consts::PI * u2;
        let z0 = r * libm::cosf(theta);
        let z1 = r * libm::sinf(theta);
        self.cached_gaussian = Some(z1);
        z0
    }

    /// A 3-vector of independent zero-mean Gaussian samples scaled by `sigma`.
    pub fn gaussian_vec3(&mut self, sigma: f32) -> Vector3<f32> {
        Vector3::new(
            self.gaussian() * sigma,
            self.gaussian() * sigma,
            self.gaussian() * sigma,
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_sequence() {
        let mut a = SimRng::new(42);
        let mut b = SimRng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u32(), b.next_u32());
        }
    }

    #[test]
    fn uniform_in_unit_interval() {
        let mut r = SimRng::new(123);
        for _ in 0..1000 {
            let x = r.next_f32();
            assert!((0.0..1.0).contains(&x));
        }
    }

    #[test]
    fn gaussian_has_approx_zero_mean_unit_variance() {
        let mut r = SimRng::new(7);
        let n = 10_000;
        let mut sum = 0.0_f64;
        let mut sum_sq = 0.0_f64;
        for _ in 0..n {
            let z = f64::from(r.gaussian());
            sum += z;
            sum_sq += z * z;
        }
        let mean = sum / f64::from(n);
        let var = sum_sq / f64::from(n) - mean * mean;
        assert!(mean.abs() < 0.05, "mean {mean}");
        assert!((var - 1.0).abs() < 0.1, "var {var}");
    }
}
