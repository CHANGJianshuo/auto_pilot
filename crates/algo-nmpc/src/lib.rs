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

/// Cascade-PI gains.
///
/// `k_i_vel` enables an integral term on the velocity-error loop to cancel
/// steady-state bias introduced by drag, motor lag, and wind. Default is
/// zero (pure P-P behaviour) so pre-M3.2 callers get unchanged output.
#[derive(Clone, Copy, Debug)]
pub struct PositionGains {
    pub k_pos: Vector3<f32>,
    pub k_vel: Vector3<f32>,
    pub k_i_vel: Vector3<f32>,
    /// Saturation on the commanded NED acceleration magnitude (m/s²). Keeps
    /// aggressive setpoints from producing tilt commands that exceed the
    /// feasible thrust envelope.
    pub max_accel: f32,
    /// Per-axis cap on the velocity integrator magnitude (m/s² equivalent,
    /// because the integrator output is added to accel_cmd). Prevents
    /// integrator wind-up during long saturation events.
    pub max_integrator: f32,
}

impl Default for PositionGains {
    fn default() -> Self {
        Self {
            k_pos: Vector3::new(1.0, 1.0, 2.0),
            k_vel: Vector3::new(3.0, 3.0, 5.0),
            k_i_vel: Vector3::zeros(),
            max_accel: 8.0,
            max_integrator: 5.0,
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

/// Pure-P baseline (no integrator). Kept for callers who don't want to
/// carry integrator state. See [`position_to_attitude_thrust_pi`] for
/// the PI-enabled version used in closed-loop SITL with drag/wind.
#[must_use]
pub fn position_to_attitude_thrust(
    setpoint: &Setpoint,
    current_position: Vector3<f32>,
    current_velocity: Vector3<f32>,
    mass_kg: f32,
    gains: &PositionGains,
) -> AttitudeAndThrust {
    let (att, _) = position_to_attitude_thrust_pi(
        setpoint,
        current_position,
        current_velocity,
        mass_kg,
        gains,
        Vector3::zeros(),
        0.0,
    );
    att
}

/// Cascade PI controller: same math as the baseline plus a bounded
/// integrator on the velocity error.
///
/// Returns `(AttitudeAndThrust, new_integrator_vel)`. The integrator is
/// the caller's responsibility to carry between steps — we stay pure so
/// unit tests and Kani remain straightforward.
///
/// Anti-windup: the integrator only accumulates when the commanded
/// acceleration is not saturated against `max_accel`. This is the
/// "conditional integration" scheme, the cheapest correct one.
#[must_use]
pub fn position_to_attitude_thrust_pi(
    setpoint: &Setpoint,
    current_position: Vector3<f32>,
    current_velocity: Vector3<f32>,
    mass_kg: f32,
    gains: &PositionGains,
    integrator_vel: Vector3<f32>,
    dt_s: f32,
) -> (AttitudeAndThrust, Vector3<f32>) {
    // 1. P-PI cascade. P on position error produces velocity command;
    //    PI on velocity error produces acceleration command.
    let pos_err = setpoint.position_ned - current_position;
    let vel_cmd = gains.k_pos.component_mul(&pos_err) + setpoint.velocity_ned;
    let vel_err = vel_cmd - current_velocity;
    let accel_raw = gains.k_vel.component_mul(&vel_err) + integrator_vel + setpoint.accel_ned;

    // 2. Saturate acceleration magnitude.
    let accel_mag = accel_raw.norm();
    let saturated = accel_mag.is_finite() && accel_mag > gains.max_accel;
    let accel_cmd = if saturated {
        accel_raw * (gains.max_accel / accel_mag)
    } else {
        accel_raw
    };

    // 3. Integrator update with conditional integration (anti-windup):
    //    only accumulate when we're not saturated in the direction of the
    //    error (cheapest practical anti-windup scheme).
    let mut new_integrator = integrator_vel;
    if !saturated && dt_s > 0.0 && dt_s.is_finite() {
        new_integrator += gains.k_i_vel.component_mul(&vel_err) * dt_s;
        // Per-axis clamp.
        new_integrator.x = new_integrator
            .x
            .clamp(-gains.max_integrator, gains.max_integrator);
        new_integrator.y = new_integrator
            .y
            .clamp(-gains.max_integrator, gains.max_integrator);
        new_integrator.z = new_integrator
            .z
            .clamp(-gains.max_integrator, gains.max_integrator);
    }

    // 4. Force balance: the vehicle must generate thrust `F = m·(a - g)`.
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

    // 5. Project desired heading onto body xy-plane.
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

    // 6. Assemble rotation matrix (columns are body axes in world NED),
    //    convert to quaternion.
    let r_des = Matrix3::from_columns(&[xb_des, yb_des, zb_des]);
    let q_des = UnitQuaternion::from_matrix(&r_des).into_inner();

    (
        AttitudeAndThrust {
            q_desired: q_des,
            thrust_n: thrust_mag,
        },
        new_integrator,
    )
}

// ----------------------------------------------------------------------------
// M9.0 — LQR position loop (unconstrained-MPC stepping stone)
// ----------------------------------------------------------------------------
//
// Replaces the hand-tuned PI cascade with optimal linear feedback derived
// from a cost function over the double-integrator position model. This is
// the "infinite-horizon, no-constraint" limit of MPC — the same
// formulation plan.md's M4 asks for, minus the QP solver and the box
// bounds on u. Those arrive in M9.1 (inequality constraints) and M9.2
// (finite receding horizon).
//
// Model (per axis, decoupled):
//   state x = [pos; vel] (2-vec)
//   input u = accel      (scalar)
//   x_{k+1} = A x_k + B u_k, with
//     A = [[1, dt], [0, 1]]
//     B = [[0.5 dt²], [dt]]
//
// Cost:
//   J = Σ (x_k^T Q x_k + u_k^T R u_k),  Q = diag(q_pos, q_vel),  R = r
//
// Discrete algebraic Riccati (DARE):
//   P = A^T P A − (A^T P B)(R + B^T P B)^{-1}(B^T P A) + Q
// Converged to a fixed point, the optimal feedback gain is:
//   K = (R + B^T P B)^{-1} B^T P A    (1×2)
// and the control law is:
//   u = −K (x − x_ref)

/// Per-axis cost weights for the LQR design.
///
/// Higher `q_pos` / `q_vel` penalises state deviation more — the
/// controller becomes more aggressive. Higher `r` penalises acceleration
/// use — the controller becomes softer. Ratios matter, not absolutes:
/// scaling all three by the same factor yields the same gains.
#[derive(Clone, Copy, Debug)]
pub struct LqrWeights {
    /// Position-error weight.
    pub q_pos: f32,
    /// Velocity-error weight.
    pub q_vel: f32,
    /// Control-effort weight.
    pub r: f32,
}

impl Default for LqrWeights {
    fn default() -> Self {
        // Hand-tuned to roughly match the PI cascade's defaults on the
        // 250 g test airframe at dt = 0.05 s — aggressive position
        // tracking, modest control effort.
        Self {
            q_pos: 4.0,
            q_vel: 1.0,
            r: 0.5,
        }
    }
}

/// Solved LQR feedback gains for one axis.
///
/// The control law is `u = -(k_p * pos_err + k_v * vel_err) = K · (x_ref − x)`
/// where `K = [k_p, k_v]`. Both gains are positive for stable systems.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqrAxisGains {
    pub k_p: f32,
    pub k_v: f32,
}

/// Iteratively solve the 2×2 DARE for a single-axis double integrator.
///
/// The DARE is a fixed-point problem; for this 2-state system it
/// converges in 20–50 iterations to machine precision. Return early on
/// stagnation to cap worst-case cost — `Icm42688::read_sample` runs on
/// the same CPU, and we can't blow a 1 kHz budget.
///
/// Returns `None` if the iteration diverges or produces non-finite
/// values — caller should fall back to PI gains.
#[must_use]
pub fn compute_lqr_gains(weights: LqrWeights, dt_s: f32) -> Option<LqrAxisGains> {
    // Guard against misuse.
    if !dt_s.is_finite()
        || dt_s <= 0.0
        || weights.q_pos < 0.0
        || weights.q_vel < 0.0
        || weights.r <= 0.0
    {
        return None;
    }

    // State transition and input matrices (2×2 and 2×1).
    let a = nalgebra::Matrix2::new(1.0_f32, dt_s, 0.0, 1.0);
    let b = nalgebra::Vector2::new(0.5 * dt_s * dt_s, dt_s);
    let q_mat = nalgebra::Matrix2::new(weights.q_pos, 0.0, 0.0, weights.q_vel);

    // Start P = Q and iterate P_{k+1} = A'PA - A'PB(R + B'PB)^{-1}B'PA + Q.
    let mut p = q_mat;
    let mut prev_trace = p.m11 + p.m22;
    for _ in 0..200 {
        let atp = a.transpose() * p;
        let btp = b.transpose() * p;
        let bpb = (btp * b).x; // scalar: 1×2 · 2×1 → 1×1
        let denom = weights.r + bpb;
        if denom <= 0.0 || !denom.is_finite() {
            return None;
        }
        let atpb = atp * b; // 2×1
        let btpa = btp * a; // 1×2
        // Outer product for the correction term.
        let correction = (atpb * btpa) * (1.0 / denom);
        let next = atp * a - correction + q_mat;

        if !next.iter().all(|x| x.is_finite()) {
            return None;
        }
        let trace = next.m11 + next.m22;
        p = next;
        // Relative-change stop.
        if (trace - prev_trace).abs() < 1.0e-6 * (1.0 + trace.abs()) {
            break;
        }
        prev_trace = trace;
    }

    // K = (R + B'PB)^{-1} B'P A — row vector, 1×2.
    let btp = b.transpose() * p;
    let bpb = (btp * b).x;
    let denom = weights.r + bpb;
    if denom <= 0.0 || !denom.is_finite() {
        return None;
    }
    let k_row = btp * a * (1.0 / denom);
    let k_p = k_row.x;
    let k_v = k_row.y;
    if !k_p.is_finite() || !k_v.is_finite() || k_p < 0.0 || k_v < 0.0 {
        return None;
    }
    Some(LqrAxisGains { k_p, k_v })
}

/// Fill a `PositionGains` from LQR weights, using the LQR-derived K as
/// the proportional gains on both position and velocity. Integrator is
/// left at zero (LQR tracks bias through the model; bias *rejection*
/// needs either a disturbance observer or an integrator — both are
/// follow-ups).
///
/// Uses the same weights for all three axes. Horizontal / vertical can
/// be tuned differently by calling [`compute_lqr_gains`] twice and
/// merging; this convenience helper is for the common isotropic case.
#[must_use]
pub fn lqr_position_gains(
    weights_xy: LqrWeights,
    weights_z: LqrWeights,
    dt_s: f32,
    max_accel: f32,
) -> Option<PositionGains> {
    let xy = compute_lqr_gains(weights_xy, dt_s)?;
    let z = compute_lqr_gains(weights_z, dt_s)?;
    Some(PositionGains {
        k_pos: Vector3::new(xy.k_p, xy.k_p, z.k_p),
        k_vel: Vector3::new(xy.k_v, xy.k_v, z.k_v),
        k_i_vel: Vector3::zeros(),
        max_accel,
        max_integrator: 0.0,
    })
}

// ----------------------------------------------------------------------------
// M9.1 — Finite-horizon MPC with box constraints (single axis)
// ----------------------------------------------------------------------------
//
// Extends the LQR design to a true QP-based MPC: we keep the same
// double-integrator model but solve a box-constrained quadratic program
// over a finite horizon every tick and apply the first control input
// (receding-horizon principle). Three-axis combination happens in the
// outer loop by running three independent 1-D solvers; proper coupled
// 6-dim MPC is a future expansion if cross-axis constraints matter.
//
// Problem (per axis, error coordinates e = x − x_ref):
//   minimise   Σ_{k=0..H-1} (q_p·e_p_k² + q_v·e_v_k²) + r·u_k²
//              + e_H^T P_f e_H                     (terminal cost)
//   s.t.       e_{k+1} = A e_k + B u_k
//              u_min ≤ u_k ≤ u_max
//
// Reduction to a dense QP in u ∈ R^H:
//   e_k = F_k e_0 + Σ_{j<k} G_{k,j} u_j      (unroll dynamics)
//   J(u) = ½ uᵀ H u + gᵀ u + const
//   H = Gᵀ Q_blk G + R_blk + G_fᵀ P_f G_f
//   g = (Gᵀ Q_blk F + G_fᵀ P_f F_H) e_0     // linear in e_0 ⇒ precompute M
//
// Solved with projected gradient descent (step α = 1/L, L = λ_max(H))
// — for small H the convergence rate is fine and warm-start between
// ticks cuts iteration count to ~5 after the first one.

/// Static configuration for a 1-axis MPC design.
#[derive(Clone, Copy, Debug)]
pub struct Mpc1dConfig {
    /// Stage cost weights (same struct as LQR; the **infinite-horizon**
    /// Riccati P is used as the terminal cost so the truncated horizon
    /// keeps LQR's stability guarantee).
    pub weights: LqrWeights,
    /// Discretisation step of the predictor (s). Must match the rate
    /// at which `solve` is called in closed loop.
    pub dt_s: f32,
    /// Input lower bound u_k ≥ u_min.
    pub u_min: f32,
    /// Input upper bound u_k ≤ u_max.
    pub u_max: f32,
}

/// Precomputed dense-QP data for a `H`-step receding-horizon MPC.
///
/// `new` factors out all work that only depends on the model + weights
/// (computing H, M, the Lipschitz step size, and the terminal cost from
/// the infinite-horizon DARE). The per-tick `solve` only does the e₀-
/// dependent linear term and the projected-gradient loop.
#[derive(Clone, Debug)]
pub struct Mpc1d<const H: usize> {
    /// QP Hessian: H × H, SPD.
    hessian: nalgebra::SMatrix<f32, H, H>,
    /// Linear term map: g = map · e_0.
    lin_from_e0: nalgebra::SMatrix<f32, H, 2>,
    u_min: f32,
    u_max: f32,
    /// 1 / λ_max(hessian) — projected-gradient step size.
    step_size: f32,
}

impl<const H: usize> Mpc1d<H> {
    /// Pre-compute the QP data. Returns `None` on invalid inputs
    /// (negative weights, non-positive `dt`, `u_min > u_max`) or if the
    /// infinite-horizon LQR Riccati fails to converge — the caller is
    /// expected to fall back to PI or plain LQR in that case.
    ///
    /// # Panics (for clippy's benefit)
    ///
    /// The local fixed arrays (`a_pow_b`, `f_pow`) are sized 64 and the
    /// early `H >= a_pow_b.len()` guard rejects anything larger, so the
    /// `[m]` / `[k]` indexing inside the body is bounds-safe by
    /// construction. We locally allow `indexing_slicing` in this
    /// function to keep the math readable.
    #[allow(clippy::indexing_slicing)]
    #[must_use]
    pub fn new(config: Mpc1dConfig) -> Option<Self> {
        if H == 0 || !config.dt_s.is_finite() || config.dt_s <= 0.0 {
            return None;
        }
        if config.u_max < config.u_min || !config.u_min.is_finite() || !config.u_max.is_finite() {
            return None;
        }
        if config.weights.r <= 0.0 || config.weights.q_pos < 0.0 || config.weights.q_vel < 0.0 {
            return None;
        }

        // Infinite-horizon Riccati P to use as terminal cost.
        let p_terminal = dare_2x2(config.weights, config.dt_s)?;

        let dt = config.dt_s;
        let a = nalgebra::Matrix2::new(1.0_f32, dt, 0.0, 1.0);
        let b_vec = nalgebra::Vector2::new(0.5 * dt * dt, dt);
        let q_mat = nalgebra::Matrix2::new(config.weights.q_pos, 0.0, 0.0, config.weights.q_vel);

        // F_k = A^k (row k of the "free response" matrix).
        // G_{k,j} = A^{k-1-j} B for j < k, 0 otherwise (lower triangular).
        // We need F_H and rows F_1..F_H-1 (for stage cost), plus
        // G_H (last row) for the terminal term.

        let mut hessian = nalgebra::SMatrix::<f32, H, H>::zeros();
        let mut lin_from_e0 = nalgebra::SMatrix::<f32, H, 2>::zeros();

        // Cache A^k B_cols to avoid recomputing. a_pow_b[m] = A^m * B.
        let mut a_pow_b = [nalgebra::Vector2::<f32>::zeros(); 64];
        if H >= a_pow_b.len() {
            // Horizon limit: keep it small to avoid heap in no_std.
            return None;
        }
        a_pow_b[0] = b_vec;
        for m in 1..H {
            a_pow_b[m] = a * a_pow_b[m - 1];
        }

        // Free-response F_k e_0. F_k = A^k; store F_1..F_H.
        let mut f_pow = [nalgebra::Matrix2::<f32>::identity(); 64];
        for k in 1..=H {
            f_pow[k] = a * f_pow[k - 1];
        }

        // Stage cost contributions. For each k=1..H-1:
        //   e_k = F_k e_0 + Σ_{j=0}^{k-1} A^{k-1-j} B u_j
        //       = F_k e_0 + Σ_{j=0}^{k-1} a_pow_b[k-1-j] u_j
        // The row of G at row k, col j is a_pow_b[k-1-j].
        for k in 1..H {
            let f_k = f_pow[k];
            for j in 0..k {
                let g_col_k_j = a_pow_b[k - 1 - j]; // 2-vec
                // Accumulate H entries: (a_pow_b[k-1-j'])^T Q (a_pow_b[k-1-j])
                //   appears at (j', j) for each j'.
                for jp in 0..k {
                    let g_col_k_jp = a_pow_b[k - 1 - jp];
                    let contrib = g_col_k_jp.dot(&(q_mat * g_col_k_j));
                    let cur = hessian.get((jp, j)).copied().unwrap_or(0.0);
                    if let Some(slot) = hessian.get_mut((jp, j)) {
                        *slot = cur + contrib;
                    }
                }
                // g ← g + (a_pow_b[k-1-j])^T Q F_k e_0    (linear in e_0)
                //   → lin_from_e0 row j gets (Q F_k)^T (a_pow_b[k-1-j])
                //     which is a 2-vec; accumulate.
                let row_vec = (q_mat * f_k).transpose() * g_col_k_j;
                for col in 0..2 {
                    let cur = lin_from_e0.get((j, col)).copied().unwrap_or(0.0);
                    if let Some(slot) = lin_from_e0.get_mut((j, col)) {
                        *slot = cur + row_vec.get(col).copied().unwrap_or(0.0);
                    }
                }
            }
        }

        // Control-effort term R_blk = r I_H → add r to diagonal.
        let r = config.weights.r;
        for k in 0..H {
            if let Some(slot) = hessian.get_mut((k, k)) {
                *slot += r;
            }
        }

        // Terminal cost e_H^T P_f e_H:
        //   e_H = F_H e_0 + Σ_{j=0}^{H-1} a_pow_b[H-1-j] u_j
        //   Let w_j = a_pow_b[H-1-j].
        let f_h = f_pow[H];
        for j in 0..H {
            let w_j = a_pow_b[H - 1 - j];
            for jp in 0..H {
                let w_jp = a_pow_b[H - 1 - jp];
                let contrib = w_jp.dot(&(p_terminal * w_j));
                let cur = hessian.get((jp, j)).copied().unwrap_or(0.0);
                if let Some(slot) = hessian.get_mut((jp, j)) {
                    *slot = cur + contrib;
                }
            }
            let row_vec = (p_terminal * f_h).transpose() * w_j;
            for col in 0..2 {
                let cur = lin_from_e0.get((j, col)).copied().unwrap_or(0.0);
                if let Some(slot) = lin_from_e0.get_mut((j, col)) {
                    *slot = cur + row_vec.get(col).copied().unwrap_or(0.0);
                }
            }
        }

        // Symmetrise H (float round-off can leave tiny skew).
        for i in 0..H {
            for j in 0..i {
                let avg = 0.5
                    * (hessian.get((i, j)).copied().unwrap_or(0.0)
                        + hessian.get((j, i)).copied().unwrap_or(0.0));
                if let Some(slot) = hessian.get_mut((i, j)) {
                    *slot = avg;
                }
                if let Some(slot) = hessian.get_mut((j, i)) {
                    *slot = avg;
                }
            }
        }

        // Row-sum upper bound on λ_max (Gershgorin). Gives a safe step
        // size α = 1/L; projected gradient converges for any α ≤ 2/L.
        let mut lmax: f32 = 0.0;
        for i in 0..H {
            let mut row_sum = 0.0_f32;
            for j in 0..H {
                row_sum += hessian.get((i, j)).copied().unwrap_or(0.0).abs();
            }
            if row_sum > lmax {
                lmax = row_sum;
            }
        }
        if lmax <= 0.0 || !lmax.is_finite() {
            return None;
        }

        Some(Self {
            hessian,
            lin_from_e0,
            u_min: config.u_min,
            u_max: config.u_max,
            step_size: 1.0 / lmax,
        })
    }

    /// Solve the QP for current error `e_0 = x - x_ref` and return the
    /// first control `u_0` from the optimal sequence.
    ///
    /// `warm_u` is read as an initial guess (previous tick's solution
    /// shifted by one is the canonical choice) and **overwritten** with
    /// the new optimum so the caller can warm-start next call. Pass a
    /// zero-filled vector on the first invocation.
    ///
    /// `max_iter` caps the projected-gradient iterations. 20–50 is
    /// enough once warm-started; first cold solve may need more.
    #[must_use]
    pub fn solve(
        &self,
        e_0: nalgebra::Vector2<f32>,
        warm_u: &mut nalgebra::SVector<f32, H>,
        max_iter: usize,
    ) -> f32 {
        // Linear term g = lin_from_e0 · e_0.
        let g = self.lin_from_e0 * e_0;
        // Project initial guess into the box.
        for k in 0..H {
            if let Some(slot) = warm_u.get_mut(k) {
                *slot = slot.clamp(self.u_min, self.u_max);
            }
        }
        // Projected gradient.
        for _ in 0..max_iter {
            let grad = self.hessian * (*warm_u) + g;
            *warm_u -= grad * self.step_size;
            for k in 0..H {
                if let Some(slot) = warm_u.get_mut(k) {
                    *slot = slot.clamp(self.u_min, self.u_max);
                }
            }
        }
        warm_u.get(0).copied().unwrap_or(0.0)
    }
}

/// Internal: solve the augmented 3×3 DARE for the LQI problem and
/// return P. Used as the MPC-I terminal cost in [`Mpc1dI::new`].
fn dare_3x3(weights: LqiWeights, dt_s: f32) -> Option<nalgebra::Matrix3<f32>> {
    if !dt_s.is_finite()
        || dt_s <= 0.0
        || weights.r <= 0.0
        || weights.q_pos < 0.0
        || weights.q_vel < 0.0
        || weights.q_i < 0.0
    {
        return None;
    }
    let a = nalgebra::Matrix3::new(1.0_f32, dt_s, 0.0, 0.0, 1.0, 0.0, dt_s, 0.0, 1.0);
    let b = nalgebra::Vector3::new(0.5 * dt_s * dt_s, dt_s, 0.0);
    let q_mat = nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(
        weights.q_pos,
        weights.q_vel,
        weights.q_i,
    ));
    let mut p = q_mat;
    let mut prev_trace = p.m11 + p.m22 + p.m33;
    for _ in 0..500 {
        let atp = a.transpose() * p;
        let btp = b.transpose() * p;
        let bpb = (btp * b).x;
        let denom = weights.r + bpb;
        if denom <= 0.0 || !denom.is_finite() {
            return None;
        }
        let correction = (atp * b * btp * a) * (1.0 / denom);
        let next = atp * a - correction + q_mat;
        if !next.iter().all(|x| x.is_finite()) {
            return None;
        }
        let trace = next.m11 + next.m22 + next.m33;
        p = next;
        if (trace - prev_trace).abs() < 1.0e-6 * (1.0 + trace.abs()) {
            break;
        }
        prev_trace = trace;
    }
    Some(p)
}

/// Internal: solve the 2×2 DARE and return P. Same iteration as
/// [`compute_lqr_gains`] but returns the full P rather than the gain.
/// Kept private because callers should stick to the gain API unless
/// they're building an MPC terminal cost like we are here.
fn dare_2x2(weights: LqrWeights, dt_s: f32) -> Option<nalgebra::Matrix2<f32>> {
    if !dt_s.is_finite()
        || dt_s <= 0.0
        || weights.r <= 0.0
        || weights.q_pos < 0.0
        || weights.q_vel < 0.0
    {
        return None;
    }
    let a = nalgebra::Matrix2::new(1.0_f32, dt_s, 0.0, 1.0);
    let b = nalgebra::Vector2::new(0.5 * dt_s * dt_s, dt_s);
    let q_mat = nalgebra::Matrix2::new(weights.q_pos, 0.0, 0.0, weights.q_vel);
    let mut p = q_mat;
    let mut prev_trace = p.m11 + p.m22;
    for _ in 0..200 {
        let atp = a.transpose() * p;
        let btp = b.transpose() * p;
        let bpb = (btp * b).x;
        let denom = weights.r + bpb;
        if denom <= 0.0 || !denom.is_finite() {
            return None;
        }
        let correction = (atp * b * btp * a) * (1.0 / denom);
        let next = atp * a - correction + q_mat;
        if !next.iter().all(|x| x.is_finite()) {
            return None;
        }
        let trace = next.m11 + next.m22;
        p = next;
        if (trace - prev_trace).abs() < 1.0e-6 * (1.0 + trace.abs()) {
            break;
        }
        prev_trace = trace;
    }
    Some(p)
}

// ----------------------------------------------------------------------------
// M9.2 — MPC-based three-axis position controller
// ----------------------------------------------------------------------------
//
// Bundles three independent 1-axis MPCs (one `Mpc1d<H>` per axis, with
// xy sharing one configuration and z getting its own) plus the three
// warm-start buffers. Exposes a `step()` method that mirrors the shape
// of `position_to_attitude_thrust_pi` — caller supplies current state
// and setpoint, receives `(AttitudeAndThrust, updated warm state)`.
//
// Keeping the warm-start buffers inside the struct makes receding-
// horizon bookkeeping painless: the caller doesn't need to remember
// `SVector<f32, H>` × 3 between ticks; they live next to the MPCs that
// use them. The struct is still a pure compute object — no IO, no
// time reads, safe to hand to tests and Kani.

/// Three-axis MPC position controller.
///
/// `MPC_XY` governs both horizontal axes (xy are geometrically
/// interchangeable for a multirotor); `mpc_z` governs altitude and
/// usually carries tighter weights because vertical excursions matter
/// more than horizontal drift.
#[derive(Clone, Debug)]
pub struct Mpc3dPositionController<const H: usize> {
    mpc_xy: Mpc1d<H>,
    mpc_z: Mpc1d<H>,
    warm_x: nalgebra::SVector<f32, H>,
    warm_y: nalgebra::SVector<f32, H>,
    warm_z: nalgebra::SVector<f32, H>,
    /// Projected-gradient iteration cap per axis per call.
    pub max_iter: usize,
    /// Clamp on the commanded acceleration vector magnitude. Applied
    /// after the per-axis MPCs so a pathological weight mix can't
    /// synthesise an ‖accel‖ beyond the flight envelope.
    pub max_accel: f32,
}

impl<const H: usize> Mpc3dPositionController<H> {
    /// Construct from xy and z configs. Both MPCs share the same `dt_s`
    /// — mixing discretisation steps between axes produces a structurally
    /// invalid 3-axis controller.
    #[must_use]
    pub fn new(
        config_xy: Mpc1dConfig,
        config_z: Mpc1dConfig,
        max_iter: usize,
        max_accel: f32,
    ) -> Option<Self> {
        if !(config_xy.dt_s == config_z.dt_s) {
            return None;
        }
        Some(Self {
            mpc_xy: Mpc1d::<H>::new(config_xy)?,
            mpc_z: Mpc1d::<H>::new(config_z)?,
            warm_x: nalgebra::SVector::<f32, H>::zeros(),
            warm_y: nalgebra::SVector::<f32, H>::zeros(),
            warm_z: nalgebra::SVector::<f32, H>::zeros(),
            max_iter,
            max_accel,
        })
    }

    /// Cold-start the warm buffers. Call when a mode switch or large
    /// setpoint jump would leave last tick's trajectory misleading.
    pub fn reset_warm_start(&mut self) {
        self.warm_x.fill(0.0);
        self.warm_y.fill(0.0);
        self.warm_z.fill(0.0);
    }

    /// One controller tick: solve three 1-axis QPs and assemble the
    /// attitude + thrust command. Shares the last-mile force-balance
    /// math with [`position_to_attitude_thrust`] so upstream/downstream
    /// callers see identical units and conventions.
    pub fn step(
        &mut self,
        setpoint: &Setpoint,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        mass_kg: f32,
    ) -> AttitudeAndThrust {
        // Error coords (e = x − x_ref) per axis.
        let e_x = nalgebra::Vector2::new(
            current_position.x - setpoint.position_ned.x,
            current_velocity.x - setpoint.velocity_ned.x,
        );
        let e_y = nalgebra::Vector2::new(
            current_position.y - setpoint.position_ned.y,
            current_velocity.y - setpoint.velocity_ned.y,
        );
        let e_z = nalgebra::Vector2::new(
            current_position.z - setpoint.position_ned.z,
            current_velocity.z - setpoint.velocity_ned.z,
        );

        // Three QPs. xy shares one MPC (same dynamics + weights), z uses
        // its own. Warm buffers are per-axis so xy still have distinct
        // initial guesses.
        let u_x = self.mpc_xy.solve(e_x, &mut self.warm_x, self.max_iter);
        let u_y = self.mpc_xy.solve(e_y, &mut self.warm_y, self.max_iter);
        let u_z = self.mpc_z.solve(e_z, &mut self.warm_z, self.max_iter);

        // Add setpoint's feed-forward acceleration then cap magnitude so
        // the attitude loop gets a feasible request.
        let mut accel_cmd = Vector3::new(u_x, u_y, u_z) + setpoint.accel_ned;
        let mag = accel_cmd.norm();
        if mag.is_finite() && mag > self.max_accel {
            accel_cmd *= self.max_accel / mag;
        }

        // Reuse the PI path's force-balance → (q_desired, thrust). Pack
        // into the existing struct via a dummy gains object whose only
        // purpose is to supply `max_accel` to the last-mile helper.
        let dummy_gains = PositionGains {
            k_pos: Vector3::zeros(),
            k_vel: Vector3::zeros(),
            k_i_vel: Vector3::zeros(),
            max_accel: self.max_accel,
            max_integrator: 0.0,
        };
        accel_to_attitude_thrust(accel_cmd, setpoint.yaw_rad, mass_kg, &dummy_gains)
    }

    /// Read-only access to the xy MPC (for tests that want to inspect
    /// the QP matrices or warm state).
    #[must_use]
    pub fn mpc_xy(&self) -> &Mpc1d<H> {
        &self.mpc_xy
    }
    /// Read-only access to the z MPC.
    #[must_use]
    pub fn mpc_z(&self) -> &Mpc1d<H> {
        &self.mpc_z
    }
}

// ----------------------------------------------------------------------------
// M9.4 — Position controller enum: PI / LQR / MPC / LQI behind one API
// ----------------------------------------------------------------------------

/// Unified dispatch for every position controller the crate exposes.
///
/// Each variant owns whatever state its algorithm needs (PI's integrator,
/// MPC's warm-start buffers, LQI's integrator); LQR is stateless. Use
/// the constructors (`::pi`, `::lqr`, `::mpc`, `::lqi`) rather than
/// building variants directly so the shape of each arm stays intact.
///
/// `H` is the MPC horizon length — a compile-time const because the
/// dense QP's working set is a `SVector<f32, H>`. Non-MPC variants
/// ignore `H`, so a typical application can pick a small value (10 at
/// 1 kHz ≈ 10 ms lookahead) and use the same enum everywhere.
#[derive(Clone, Debug)]
pub enum PositionController<const H: usize> {
    /// Cascade PI — the M2.5 / M3.2 baseline. Stateful: `integrator`
    /// accumulates velocity error with anti-windup inside the step call.
    Pi {
        gains: PositionGains,
        integrator: Vector3<f32>,
    },
    /// Infinite-horizon LQR — optimal feedback for the nominal model.
    /// No integrator; no state between ticks. `gains` should be the
    /// output of [`lqr_position_gains`].
    Lqr { gains: PositionGains },
    /// Finite-horizon MPC with box constraints (M9.1 / M9.2). Owns
    /// the three warm-start buffers internally.
    Mpc(Mpc3dPositionController<H>),
    /// LQR augmented with an integrator state (M9.3). Cheap + cancels
    /// steady-state bias under constant disturbance.
    Lqi(Lqi3dPositionController),
    /// Constraint-aware MPC + integrator state (M9.5). Combines MPC's
    /// look-ahead + box constraints with LQI's bias rejection.
    MpcI(MpcI3dPositionController<H>),
}

impl<const H: usize> PositionController<H> {
    /// Build the cascade-PI variant.
    #[must_use]
    pub fn pi(gains: PositionGains) -> Self {
        Self::Pi {
            gains,
            integrator: Vector3::zeros(),
        }
    }

    /// Pre-compute LQR gains from weights and wrap in an `Lqr` variant.
    /// Returns `None` if the 2×2 DARE fails to converge.
    #[must_use]
    pub fn lqr(
        weights_xy: LqrWeights,
        weights_z: LqrWeights,
        dt_s: f32,
        max_accel: f32,
    ) -> Option<Self> {
        Some(Self::Lqr {
            gains: lqr_position_gains(weights_xy, weights_z, dt_s, max_accel)?,
        })
    }

    /// Build an `Mpc` variant. Same failure modes as
    /// [`Mpc3dPositionController::new`].
    #[must_use]
    pub fn mpc(
        cfg_xy: Mpc1dConfig,
        cfg_z: Mpc1dConfig,
        max_iter: usize,
        max_accel: f32,
    ) -> Option<Self> {
        Mpc3dPositionController::new(cfg_xy, cfg_z, max_iter, max_accel).map(Self::Mpc)
    }

    /// Build an `Lqi` variant.
    #[must_use]
    pub fn lqi(
        weights_xy: LqiWeights,
        weights_z: LqiWeights,
        dt_s: f32,
        max_accel: f32,
        integrator_max: f32,
    ) -> Option<Self> {
        Lqi3dPositionController::new(weights_xy, weights_z, dt_s, max_accel, integrator_max)
            .map(Self::Lqi)
    }

    /// Build an `MpcI` variant (MPC with integrator).
    #[must_use]
    pub fn mpc_i(
        cfg_xy: Mpc1dIConfig,
        cfg_z: Mpc1dIConfig,
        max_iter: usize,
        max_accel: f32,
        integrator_max: f32,
    ) -> Option<Self> {
        MpcI3dPositionController::new(cfg_xy, cfg_z, max_iter, max_accel, integrator_max)
            .map(Self::MpcI)
    }

    /// Single-tick dispatch. `dt_s` is consumed by the PI integrator
    /// update and ignored by variants that baked `dt_s` in at
    /// construction (MPC / LQI).
    pub fn step(
        &mut self,
        setpoint: &Setpoint,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        mass_kg: f32,
        dt_s: f32,
    ) -> AttitudeAndThrust {
        match self {
            Self::Pi { gains, integrator } => {
                let (att, new_integ) = position_to_attitude_thrust_pi(
                    setpoint,
                    current_position,
                    current_velocity,
                    mass_kg,
                    gains,
                    *integrator,
                    dt_s,
                );
                *integrator = new_integ;
                att
            }
            Self::Lqr { gains } => position_to_attitude_thrust(
                setpoint,
                current_position,
                current_velocity,
                mass_kg,
                gains,
            ),
            Self::Mpc(c) => c.step(setpoint, current_position, current_velocity, mass_kg),
            Self::Lqi(c) => c.step(setpoint, current_position, current_velocity, mass_kg),
            Self::MpcI(c) => c.step(setpoint, current_position, current_velocity, mass_kg),
        }
    }

    /// Reset per-controller state on mode switches / big setpoint jumps.
    pub fn reset(&mut self) {
        match self {
            Self::Pi { integrator, .. } => *integrator = Vector3::zeros(),
            Self::Lqr { .. } => {}
            Self::Mpc(c) => c.reset_warm_start(),
            Self::Lqi(c) => c.reset_integrator(),
            Self::MpcI(c) => c.reset(),
        }
    }

    /// Short name of the active variant, for logging / telemetry.
    #[must_use]
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::Pi { .. } => "PI",
            Self::Lqr { .. } => "LQR",
            Self::Mpc(_) => "MPC",
            Self::Lqi(_) => "LQI",
            Self::MpcI(_) => "MPC-I",
        }
    }
}

// ----------------------------------------------------------------------------
// M9.3 — LQI (LQR + integrator): kills steady-state bias
// ----------------------------------------------------------------------------
//
// Plain LQR is optimal for the exact model we designed against. Any
// constant disturbance (aerodynamic drag, wind, mass change after
// takeoff, trim offset) makes the nominal model wrong and LQR shows a
// steady-state tracking error. Classical fix: add an integrator state,
// solve the augmented 3-state Riccati, the resulting `k_i` gain cancels
// constant disturbances exactly.
//
// Augmented model (per axis, error coords e = x − x_ref):
//   z = [e_p; e_v; i],   i_{k+1} = i_k + e_p · dt
//   A_z = [[1, dt, 0], [0, 1, 0], [dt, 0, 1]]
//   B_z = [dt²/2, dt, 0]
//   Q_z = diag(q_pos, q_vel, q_i),  R = r
//
// Control law: u = −K · z = −(k_p · e_p + k_v · e_v + k_i · i).

/// Per-axis cost weights for LQI.
#[derive(Clone, Copy, Debug)]
pub struct LqiWeights {
    pub q_pos: f32,
    pub q_vel: f32,
    /// Integrator weight. Higher → more aggressive bias cancellation +
    /// faster integrator wind-up. `q_i = 0` reduces LQI to LQR exactly
    /// (i will be a nullspace dimension).
    pub q_i: f32,
    pub r: f32,
}

impl Default for LqiWeights {
    fn default() -> Self {
        Self {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 0.5,
            r: 0.5,
        }
    }
}

/// Solved LQI gains for one axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqiAxisGains {
    pub k_p: f32,
    pub k_v: f32,
    pub k_i: f32,
}

/// Iteratively solve the 3×3 DARE for the LQI problem. Same structure
/// as [`compute_lqr_gains`] but with an augmented state.
#[must_use]
pub fn compute_lqi_gains(weights: LqiWeights, dt_s: f32) -> Option<LqiAxisGains> {
    if !dt_s.is_finite()
        || dt_s <= 0.0
        || weights.q_pos < 0.0
        || weights.q_vel < 0.0
        || weights.q_i < 0.0
        || weights.r <= 0.0
    {
        return None;
    }
    let a = nalgebra::Matrix3::new(1.0_f32, dt_s, 0.0, 0.0, 1.0, 0.0, dt_s, 0.0, 1.0);
    let b = nalgebra::Vector3::new(0.5 * dt_s * dt_s, dt_s, 0.0);
    let q_mat = nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(
        weights.q_pos,
        weights.q_vel,
        weights.q_i,
    ));
    let mut p = q_mat;
    let mut prev_trace = p.m11 + p.m22 + p.m33;
    for _ in 0..500 {
        let atp = a.transpose() * p;
        let btp = b.transpose() * p;
        let bpb = (btp * b).x;
        let denom = weights.r + bpb;
        if denom <= 0.0 || !denom.is_finite() {
            return None;
        }
        let atpb = atp * b;
        let btpa = btp * a;
        let correction = (atpb * btpa) * (1.0 / denom);
        let next = atp * a - correction + q_mat;
        if !next.iter().all(|x| x.is_finite()) {
            return None;
        }
        let trace = next.m11 + next.m22 + next.m33;
        p = next;
        if (trace - prev_trace).abs() < 1.0e-6 * (1.0 + trace.abs()) {
            break;
        }
        prev_trace = trace;
    }
    let btp = b.transpose() * p;
    let bpb = (btp * b).x;
    let denom = weights.r + bpb;
    if denom <= 0.0 || !denom.is_finite() {
        return None;
    }
    let k_row = btp * a * (1.0 / denom);
    let k_p = k_row.get(0).copied().unwrap_or(0.0);
    let k_v = k_row.get(1).copied().unwrap_or(0.0);
    let k_i = k_row.get(2).copied().unwrap_or(0.0);
    if !k_p.is_finite() || !k_v.is_finite() || !k_i.is_finite() || k_p < 0.0 || k_v < 0.0 {
        return None;
    }
    Some(LqiAxisGains { k_p, k_v, k_i })
}

/// Three-axis LQI position controller.
///
/// Analytical cousin of `Mpc3dPositionController`: same shape but solves
/// the infinite-horizon Riccati once at construction, then each tick
/// evaluates the fixed-gain law `u = −K · [e_p, e_v, i]` and advances
/// the integrator `i`. Cheap to run (no iterative solver) and has
/// **zero steady-state bias** under constant disturbances — the MPC's
/// advantage under transient constraints is traded for the integrator's
/// advantage under persistent wind / drag.
#[derive(Clone, Debug)]
pub struct Lqi3dPositionController {
    gains_xy: LqiAxisGains,
    gains_z: LqiAxisGains,
    /// Integrator state per axis (NED).
    integrator: Vector3<f32>,
    /// Per-axis anti-windup cap on the integrator magnitude.
    pub integrator_max: f32,
    /// Clamp on commanded acceleration magnitude.
    pub max_accel: f32,
    dt_s: f32,
}

impl Lqi3dPositionController {
    /// Solve per-axis DAREs and return the controller. `None` on bad
    /// weights / dt.
    #[must_use]
    pub fn new(
        weights_xy: LqiWeights,
        weights_z: LqiWeights,
        dt_s: f32,
        max_accel: f32,
        integrator_max: f32,
    ) -> Option<Self> {
        Some(Self {
            gains_xy: compute_lqi_gains(weights_xy, dt_s)?,
            gains_z: compute_lqi_gains(weights_z, dt_s)?,
            integrator: Vector3::zeros(),
            integrator_max,
            max_accel,
            dt_s,
        })
    }

    /// Cold-start the integrators. Call on mode switches or after a
    /// large setpoint jump so prior tracking history doesn't pollute
    /// the new operating point.
    pub fn reset_integrator(&mut self) {
        self.integrator = Vector3::zeros();
    }

    /// Read-only accessor for unit tests + logging.
    #[must_use]
    pub fn integrator(&self) -> Vector3<f32> {
        self.integrator
    }

    /// One controller tick.
    pub fn step(
        &mut self,
        setpoint: &Setpoint,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        mass_kg: f32,
    ) -> AttitudeAndThrust {
        // Error coords.
        let e_p = current_position - setpoint.position_ned;
        let e_v = current_velocity - setpoint.velocity_ned;

        // Compute control u = −K·z per axis. xy share their gains.
        let u_x = -(self.gains_xy.k_p * e_p.x
            + self.gains_xy.k_v * e_v.x
            + self.gains_xy.k_i * self.integrator.x);
        let u_y = -(self.gains_xy.k_p * e_p.y
            + self.gains_xy.k_v * e_v.y
            + self.gains_xy.k_i * self.integrator.y);
        let u_z = -(self.gains_z.k_p * e_p.z
            + self.gains_z.k_v * e_v.z
            + self.gains_z.k_i * self.integrator.z);
        let mut accel_cmd = Vector3::new(u_x, u_y, u_z) + setpoint.accel_ned;
        let mag = accel_cmd.norm();
        let saturated = mag.is_finite() && mag > self.max_accel;
        if saturated {
            accel_cmd *= self.max_accel / mag;
        }

        // Advance integrator with conditional integration (anti-windup):
        // only accumulate when the commanded accel is feasible.
        if !saturated && self.dt_s.is_finite() && self.dt_s > 0.0 {
            self.integrator += e_p * self.dt_s;
            self.integrator.x = self
                .integrator
                .x
                .clamp(-self.integrator_max, self.integrator_max);
            self.integrator.y = self
                .integrator
                .y
                .clamp(-self.integrator_max, self.integrator_max);
            self.integrator.z = self
                .integrator
                .z
                .clamp(-self.integrator_max, self.integrator_max);
        }

        let dummy_gains = PositionGains {
            k_pos: Vector3::zeros(),
            k_vel: Vector3::zeros(),
            k_i_vel: Vector3::zeros(),
            max_accel: self.max_accel,
            max_integrator: 0.0,
        };
        accel_to_attitude_thrust(accel_cmd, setpoint.yaw_rad, mass_kg, &dummy_gains)
    }

    /// Exposed solved gains for inspection / comparison against LQR.
    #[must_use]
    pub fn gains_xy(&self) -> LqiAxisGains {
        self.gains_xy
    }
    #[must_use]
    pub fn gains_z(&self) -> LqiAxisGains {
        self.gains_z
    }
}

/// Shared last-mile: commanded accel (NED) + desired yaw → body
/// attitude + thrust magnitude. Factored out of
/// `position_to_attitude_thrust_pi` so the MPC controller reuses it
/// verbatim (no conceptual drift between the two control paths).
fn accel_to_attitude_thrust(
    accel_cmd: Vector3<f32>,
    yaw_rad: f32,
    mass_kg: f32,
    _gains: &PositionGains,
) -> AttitudeAndThrust {
    let g_vec = Vector3::new(0.0, 0.0, GRAVITY_M_S2);
    let thrust_vec_ned = mass_kg * (accel_cmd - g_vec);
    let thrust_mag = thrust_vec_ned.norm();
    let zb_des = if thrust_mag > 1.0e-3 && thrust_mag.is_finite() {
        -thrust_vec_ned / thrust_mag
    } else {
        Vector3::new(0.0, 0.0, -1.0)
    };
    let cy = libm::cosf(yaw_rad);
    let sy = libm::sinf(yaw_rad);
    let heading = Vector3::new(cy, sy, 0.0);
    let xb_raw = heading - heading.dot(&zb_des) * zb_des;
    let xb_norm = xb_raw.norm();
    let xb_des = if xb_norm > 1.0e-3 {
        xb_raw / xb_norm
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let yb_des = zb_des.cross(&xb_des);
    let r_des = Matrix3::from_columns(&[xb_des, yb_des, zb_des]);
    let q_des = UnitQuaternion::from_matrix(&r_des).into_inner();
    AttitudeAndThrust {
        q_desired: q_des,
        thrust_n: thrust_mag,
    }
}

// ----------------------------------------------------------------------------
// M9.5 — MPC-I: constraint-aware MPC with integrator state
// ----------------------------------------------------------------------------
//
// Combines M9.1 MPC (finite horizon + box constraints) with M9.3 LQI
// (integrator state to kill constant disturbances). The augmented
// state carries the integrator as its third component, so the QP
// terminal cost sees it and the optimiser considers integrator build-up
// as part of the tracking cost.
//
// Augmented model (per axis, error coords z = [e_p, e_v, i]):
//   A_z = [[1, dt, 0], [0, 1, 0], [dt, 0, 1]]
//   B_z = [dt²/2, dt, 0]
//   Q_z = diag(q_pos, q_vel, q_i),  R = r
//   i_{k+1} = i_k + e_p_k · dt  (third row of A_z)
//
// Reduced QP: identical shape to Mpc1d, just with 3-dim state
// propagation — Hessian is still H×H, linear-term map is H×3 (`z_0`
// instead of `e_0`).

/// Static configuration for MPC-I.
#[derive(Clone, Copy, Debug)]
pub struct Mpc1dIConfig {
    pub weights: LqiWeights,
    pub dt_s: f32,
    pub u_min: f32,
    pub u_max: f32,
}

/// MPC-I solver. Sibling of [`Mpc1d`] with augmented-state dynamics.
#[derive(Clone, Debug)]
pub struct Mpc1dI<const H: usize> {
    hessian: nalgebra::SMatrix<f32, H, H>,
    /// `g = lin_from_z0 · z_0`  where z_0 = [e_p, e_v, i].
    lin_from_z0: nalgebra::SMatrix<f32, H, 3>,
    u_min: f32,
    u_max: f32,
    step_size: f32,
}

impl<const H: usize> Mpc1dI<H> {
    /// Pre-compute the QP matrices. See [`Mpc1d::new`] for the general
    /// approach; this variant uses a 3-dim state and [`dare_3x3`] for
    /// the terminal cost.
    #[allow(clippy::indexing_slicing)]
    #[must_use]
    pub fn new(config: Mpc1dIConfig) -> Option<Self> {
        if H == 0 || !config.dt_s.is_finite() || config.dt_s <= 0.0 {
            return None;
        }
        if config.u_max < config.u_min || !config.u_min.is_finite() || !config.u_max.is_finite() {
            return None;
        }
        if config.weights.r <= 0.0
            || config.weights.q_pos < 0.0
            || config.weights.q_vel < 0.0
            || config.weights.q_i < 0.0
        {
            return None;
        }

        let p_terminal = dare_3x3(config.weights, config.dt_s)?;

        let dt = config.dt_s;
        let a = nalgebra::Matrix3::new(1.0_f32, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 1.0);
        let b_vec = nalgebra::Vector3::new(0.5 * dt * dt, dt, 0.0);
        let q_mat = nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(
            config.weights.q_pos,
            config.weights.q_vel,
            config.weights.q_i,
        ));

        let mut hessian = nalgebra::SMatrix::<f32, H, H>::zeros();
        let mut lin_from_z0 = nalgebra::SMatrix::<f32, H, 3>::zeros();

        let mut a_pow_b = [nalgebra::Vector3::<f32>::zeros(); 64];
        if H >= a_pow_b.len() {
            return None;
        }
        a_pow_b[0] = b_vec;
        for m in 1..H {
            a_pow_b[m] = a * a_pow_b[m - 1];
        }

        let mut f_pow = [nalgebra::Matrix3::<f32>::identity(); 64];
        for k in 1..=H {
            f_pow[k] = a * f_pow[k - 1];
        }

        for k in 1..H {
            let f_k = f_pow[k];
            for j in 0..k {
                let g_col_k_j = a_pow_b[k - 1 - j];
                for jp in 0..k {
                    let g_col_k_jp = a_pow_b[k - 1 - jp];
                    let contrib = g_col_k_jp.dot(&(q_mat * g_col_k_j));
                    let cur = hessian.get((jp, j)).copied().unwrap_or(0.0);
                    if let Some(slot) = hessian.get_mut((jp, j)) {
                        *slot = cur + contrib;
                    }
                }
                let row_vec = (q_mat * f_k).transpose() * g_col_k_j;
                for col in 0..3 {
                    let cur = lin_from_z0.get((j, col)).copied().unwrap_or(0.0);
                    if let Some(slot) = lin_from_z0.get_mut((j, col)) {
                        *slot = cur + row_vec.get(col).copied().unwrap_or(0.0);
                    }
                }
            }
        }

        let r = config.weights.r;
        for k in 0..H {
            if let Some(slot) = hessian.get_mut((k, k)) {
                *slot += r;
            }
        }

        let f_h = f_pow[H];
        for j in 0..H {
            let w_j = a_pow_b[H - 1 - j];
            for jp in 0..H {
                let w_jp = a_pow_b[H - 1 - jp];
                let contrib = w_jp.dot(&(p_terminal * w_j));
                let cur = hessian.get((jp, j)).copied().unwrap_or(0.0);
                if let Some(slot) = hessian.get_mut((jp, j)) {
                    *slot = cur + contrib;
                }
            }
            let row_vec = (p_terminal * f_h).transpose() * w_j;
            for col in 0..3 {
                let cur = lin_from_z0.get((j, col)).copied().unwrap_or(0.0);
                if let Some(slot) = lin_from_z0.get_mut((j, col)) {
                    *slot = cur + row_vec.get(col).copied().unwrap_or(0.0);
                }
            }
        }

        // Symmetrise H.
        for i in 0..H {
            for j in 0..i {
                let avg = 0.5
                    * (hessian.get((i, j)).copied().unwrap_or(0.0)
                        + hessian.get((j, i)).copied().unwrap_or(0.0));
                if let Some(slot) = hessian.get_mut((i, j)) {
                    *slot = avg;
                }
                if let Some(slot) = hessian.get_mut((j, i)) {
                    *slot = avg;
                }
            }
        }

        let mut lmax: f32 = 0.0;
        for i in 0..H {
            let mut row_sum = 0.0_f32;
            for j in 0..H {
                row_sum += hessian.get((i, j)).copied().unwrap_or(0.0).abs();
            }
            if row_sum > lmax {
                lmax = row_sum;
            }
        }
        if lmax <= 0.0 || !lmax.is_finite() {
            return None;
        }

        Some(Self {
            hessian,
            lin_from_z0,
            u_min: config.u_min,
            u_max: config.u_max,
            step_size: 1.0 / lmax,
        })
    }

    /// Solve the QP from augmented state `z_0 = [e_p, e_v, i]`. Returns
    /// the first control `u_0`.
    #[must_use]
    pub fn solve(
        &self,
        z_0: nalgebra::Vector3<f32>,
        warm_u: &mut nalgebra::SVector<f32, H>,
        max_iter: usize,
    ) -> f32 {
        let g = self.lin_from_z0 * z_0;
        for k in 0..H {
            if let Some(slot) = warm_u.get_mut(k) {
                *slot = slot.clamp(self.u_min, self.u_max);
            }
        }
        for _ in 0..max_iter {
            let grad = self.hessian * (*warm_u) + g;
            *warm_u -= grad * self.step_size;
            for k in 0..H {
                if let Some(slot) = warm_u.get_mut(k) {
                    *slot = slot.clamp(self.u_min, self.u_max);
                }
            }
        }
        warm_u.get(0).copied().unwrap_or(0.0)
    }
}

/// Three-axis MPC-I position controller.
///
/// Mirrors [`Mpc3dPositionController`] and [`Lqi3dPositionController`]:
/// xy share one solver, z has its own, each axis carries a warm buffer
/// AND an integrator accumulator. The integrator is **measurement-
/// driven** (advanced by actual position error, not the QP's prediction)
/// so it tracks real-world disturbance even when the model is wrong.
#[derive(Clone, Debug)]
pub struct MpcI3dPositionController<const H: usize> {
    mpc_xy: Mpc1dI<H>,
    mpc_z: Mpc1dI<H>,
    warm_x: nalgebra::SVector<f32, H>,
    warm_y: nalgebra::SVector<f32, H>,
    warm_z: nalgebra::SVector<f32, H>,
    integrator: Vector3<f32>,
    pub integrator_max: f32,
    pub max_iter: usize,
    pub max_accel: f32,
    dt_s: f32,
}

impl<const H: usize> MpcI3dPositionController<H> {
    /// Build the three-axis controller. Both axes must share `dt_s` —
    /// the integrator update cadence has to match across the whole xy
    /// / z stack for the LQI-derived terminal cost to be consistent.
    #[must_use]
    pub fn new(
        config_xy: Mpc1dIConfig,
        config_z: Mpc1dIConfig,
        max_iter: usize,
        max_accel: f32,
        integrator_max: f32,
    ) -> Option<Self> {
        if config_xy.dt_s != config_z.dt_s {
            return None;
        }
        Some(Self {
            mpc_xy: Mpc1dI::<H>::new(config_xy)?,
            mpc_z: Mpc1dI::<H>::new(config_z)?,
            warm_x: nalgebra::SVector::<f32, H>::zeros(),
            warm_y: nalgebra::SVector::<f32, H>::zeros(),
            warm_z: nalgebra::SVector::<f32, H>::zeros(),
            integrator: Vector3::zeros(),
            integrator_max,
            max_iter,
            max_accel,
            dt_s: config_xy.dt_s,
        })
    }

    /// Cold-start: zero the warm buffers AND the integrator.
    pub fn reset(&mut self) {
        self.warm_x.fill(0.0);
        self.warm_y.fill(0.0);
        self.warm_z.fill(0.0);
        self.integrator = Vector3::zeros();
    }

    /// Exposed integrator for unit tests / logging.
    #[must_use]
    pub fn integrator(&self) -> Vector3<f32> {
        self.integrator
    }

    /// One controller tick.
    pub fn step(
        &mut self,
        setpoint: &Setpoint,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        mass_kg: f32,
    ) -> AttitudeAndThrust {
        let e_p = current_position - setpoint.position_ned;
        let e_v = current_velocity - setpoint.velocity_ned;

        let z_x = nalgebra::Vector3::new(e_p.x, e_v.x, self.integrator.x);
        let z_y = nalgebra::Vector3::new(e_p.y, e_v.y, self.integrator.y);
        let z_z = nalgebra::Vector3::new(e_p.z, e_v.z, self.integrator.z);

        let u_x = self.mpc_xy.solve(z_x, &mut self.warm_x, self.max_iter);
        let u_y = self.mpc_xy.solve(z_y, &mut self.warm_y, self.max_iter);
        let u_z = self.mpc_z.solve(z_z, &mut self.warm_z, self.max_iter);

        let mut accel_cmd = Vector3::new(u_x, u_y, u_z) + setpoint.accel_ned;
        let mag = accel_cmd.norm();
        let saturated = mag.is_finite() && mag > self.max_accel;
        if saturated {
            accel_cmd *= self.max_accel / mag;
        }

        // Advance integrator with conditional integration (anti-windup).
        if !saturated && self.dt_s.is_finite() && self.dt_s > 0.0 {
            self.integrator += e_p * self.dt_s;
            self.integrator.x = self
                .integrator
                .x
                .clamp(-self.integrator_max, self.integrator_max);
            self.integrator.y = self
                .integrator
                .y
                .clamp(-self.integrator_max, self.integrator_max);
            self.integrator.z = self
                .integrator
                .z
                .clamp(-self.integrator_max, self.integrator_max);
        }

        let dummy_gains = PositionGains {
            k_pos: Vector3::zeros(),
            k_vel: Vector3::zeros(),
            k_i_vel: Vector3::zeros(),
            max_accel: self.max_accel,
            max_integrator: 0.0,
        };
        accel_to_attitude_thrust(accel_cmd, setpoint.yaw_rad, mass_kg, &dummy_gains)
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
            "thrust {} expected {}",
            out.thrust_n,
            expected_thrust
        );
        // q should be approximately identity: (1, 0, 0, 0) or (-1, 0, 0, 0).
        let q = out.q_desired;
        assert!(
            (q.w.abs() - 1.0).abs() < 1.0e-3,
            "q.w = {} (expected ±1)",
            q.w
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
        assert!(
            out.thrust_n > hover,
            "thrust {} should > hover {}",
            out.thrust_n,
            hover
        );
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
        let max_thrust = default_mass() * (gains.max_accel.powi(2) + GRAVITY_M_S2.powi(2)).sqrt();
        assert!(
            out.thrust_n <= max_thrust + 1.0e-3,
            "thrust {} > saturation bound {}",
            out.thrust_n,
            max_thrust
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

    #[test]
    fn pi_integrator_accumulates_against_bias() {
        // With a constant velocity error (e.g. permanent wind),
        // the integrator should grow in the direction that cancels it.
        let gains = PositionGains {
            k_i_vel: Vector3::new(0.5, 0.5, 0.5),
            ..PositionGains::default()
        };
        let sp = Setpoint::default();
        let pos = Vector3::zeros();
        let vel_bias = Vector3::new(0.2, 0.0, 0.0); // vehicle drifting +x
        let mut integ = Vector3::zeros();
        let dt = 0.01;
        for _ in 0..100 {
            let (_att, new_integ) = position_to_attitude_thrust_pi(
                &sp,
                pos,
                vel_bias,
                default_mass(),
                &gains,
                integ,
                dt,
            );
            integ = new_integ;
        }
        // vel_err = -0.2 (since vel_cmd = 0 − 0.2 bias = −0.2). integrator
        // accumulates k_i · vel_err · total_dt = 0.5 · (-0.2) · 1.0 = -0.1,
        // bounded by max_integrator = 5.
        assert!(integ.x < 0.0, "integ.x = {}", integ.x);
        assert!(integ.x > -gains.max_integrator);
    }

    #[test]
    fn pi_integrator_stays_bounded_under_saturation() {
        // Far setpoint → saturated → integrator should not accumulate.
        let gains = PositionGains {
            k_i_vel: Vector3::new(1.0, 1.0, 1.0),
            max_accel: 3.0, // tight cap to trigger saturation
            ..PositionGains::default()
        };
        let sp = Setpoint {
            position_ned: Vector3::new(100.0, 0.0, 0.0),
            ..Setpoint::default()
        };
        let mut integ = Vector3::zeros();
        for _ in 0..1000 {
            let (_, new_integ) = position_to_attitude_thrust_pi(
                &sp,
                Vector3::zeros(),
                Vector3::zeros(),
                default_mass(),
                &gains,
                integ,
                0.01,
            );
            integ = new_integ;
        }
        // Anti-windup: integrator stays near zero despite huge pos error.
        assert!(integ.norm() < 0.1, "integrator ran away: {integ}");
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

    // ------------------------------------------------------------------
    // M9.0 LQR tests
    // ------------------------------------------------------------------

    #[test]
    fn lqr_default_weights_produce_positive_gains() {
        let g = compute_lqr_gains(LqrWeights::default(), 0.05).expect("DARE converges");
        assert!(g.k_p > 0.0 && g.k_v > 0.0);
        assert!(g.k_p.is_finite() && g.k_v.is_finite());
    }

    #[test]
    fn lqr_invalid_inputs_return_none() {
        // Negative r must fail.
        assert!(
            compute_lqr_gains(
                LqrWeights {
                    q_pos: 1.0,
                    q_vel: 1.0,
                    r: -1.0
                },
                0.05,
            )
            .is_none()
        );
        // dt <= 0 must fail.
        assert!(compute_lqr_gains(LqrWeights::default(), 0.0).is_none());
        assert!(compute_lqr_gains(LqrWeights::default(), -0.01).is_none());
        // NaN dt must fail.
        assert!(compute_lqr_gains(LqrWeights::default(), f32::NAN).is_none());
    }

    #[test]
    fn lqr_higher_q_pos_tightens_position_gain() {
        // Doubling q_pos with fixed q_vel / r should raise k_p.
        let soft = compute_lqr_gains(
            LqrWeights {
                q_pos: 1.0,
                q_vel: 1.0,
                r: 1.0,
            },
            0.05,
        )
        .unwrap();
        let stiff = compute_lqr_gains(
            LqrWeights {
                q_pos: 10.0,
                q_vel: 1.0,
                r: 1.0,
            },
            0.05,
        )
        .unwrap();
        assert!(
            stiff.k_p > soft.k_p,
            "expected k_p to grow with q_pos: soft {} stiff {}",
            soft.k_p,
            stiff.k_p
        );
    }

    #[test]
    fn lqr_closed_loop_is_stable() {
        // Simulate a discrete double-integrator under u = -K·x and check
        // the state converges to zero from an initial offset.
        let dt = 0.05_f32;
        let g = compute_lqr_gains(LqrWeights::default(), dt).unwrap();
        let (mut pos, mut vel) = (1.0_f32, 0.0_f32);
        for _ in 0..200 {
            let u = -(g.k_p * pos + g.k_v * vel);
            // Euler integrate the per-axis dynamics.
            pos += vel * dt + 0.5 * u * dt * dt;
            vel += u * dt;
        }
        assert!(
            pos.abs() < 0.01 && vel.abs() < 0.01,
            "converged pos={pos} vel={vel}"
        );
    }

    #[test]
    fn lqr_position_gains_helper_fills_all_three_axes() {
        let weights_xy = LqrWeights {
            q_pos: 2.0,
            q_vel: 1.0,
            r: 0.5,
        };
        let weights_z = LqrWeights {
            q_pos: 6.0,
            q_vel: 2.0,
            r: 0.5,
        };
        let pg = lqr_position_gains(weights_xy, weights_z, 0.05, 8.0).unwrap();
        assert!(pg.k_pos.x > 0.0 && pg.k_pos.y > 0.0 && pg.k_pos.z > 0.0);
        // xy axes share their gains.
        assert_eq!(pg.k_pos.x, pg.k_pos.y);
        assert_eq!(pg.k_vel.x, pg.k_vel.y);
        // z is independently tuned and should differ.
        assert!(pg.k_pos.z != pg.k_pos.x);
        // Integrator disabled in LQR baseline.
        assert_eq!(pg.k_i_vel, Vector3::zeros());
        assert_eq!(pg.max_accel, 8.0);
    }

    // ------------------------------------------------------------------
    // M9.1 MPC tests
    // ------------------------------------------------------------------

    const MPC_H: usize = 10;

    fn default_mpc_config(tight_box: bool) -> Mpc1dConfig {
        Mpc1dConfig {
            weights: LqrWeights::default(),
            dt_s: 0.05,
            u_min: if tight_box { -3.0 } else { -100.0 },
            u_max: if tight_box { 3.0 } else { 100.0 },
        }
    }

    #[test]
    fn mpc_new_rejects_bad_inputs() {
        // dt ≤ 0
        let mut cfg = default_mpc_config(false);
        cfg.dt_s = 0.0;
        assert!(Mpc1d::<MPC_H>::new(cfg).is_none());
        cfg.dt_s = -0.01;
        assert!(Mpc1d::<MPC_H>::new(cfg).is_none());
        cfg.dt_s = f32::NAN;
        assert!(Mpc1d::<MPC_H>::new(cfg).is_none());
        // u_min > u_max
        let mut cfg = default_mpc_config(false);
        cfg.u_min = 1.0;
        cfg.u_max = -1.0;
        assert!(Mpc1d::<MPC_H>::new(cfg).is_none());
        // r ≤ 0
        let mut cfg = default_mpc_config(false);
        cfg.weights.r = 0.0;
        assert!(Mpc1d::<MPC_H>::new(cfg).is_none());
        // H = 0 (zero horizon) — exercise the const path.
        let ok_cfg = default_mpc_config(false);
        assert!(Mpc1d::<0>::new(ok_cfg).is_none());
    }

    #[test]
    fn mpc_unconstrained_solution_matches_lqr_first_action() {
        // With loose box, first-step MPC command should be very close to
        // what the infinite-horizon LQR (same Q/R) prescribes.
        let cfg = default_mpc_config(false);
        let mpc = Mpc1d::<MPC_H>::new(cfg).unwrap();
        let lqr = compute_lqr_gains(cfg.weights, cfg.dt_s).unwrap();
        let e0 = nalgebra::Vector2::new(0.5_f32, 0.0);
        let mut warm = nalgebra::SVector::<f32, MPC_H>::zeros();
        let u = mpc.solve(e0, &mut warm, 500);
        let u_lqr = -(lqr.k_p * e0.x + lqr.k_v * e0.y);
        assert!((u - u_lqr).abs() < 0.05, "MPC u0 {u} vs LQR u {u_lqr}");
    }

    #[test]
    fn mpc_respects_box_constraint() {
        // Tight u_max. From a large position error, unconstrained LQR
        // would command far above u_max; MPC must clamp.
        let cfg = default_mpc_config(true);
        let mpc = Mpc1d::<MPC_H>::new(cfg).unwrap();
        let e0 = nalgebra::Vector2::new(10.0_f32, 0.0);
        let mut warm = nalgebra::SVector::<f32, MPC_H>::zeros();
        let u = mpc.solve(e0, &mut warm, 200);
        assert!(u >= cfg.u_min - 1.0e-3);
        assert!(u <= cfg.u_max + 1.0e-3);
        // And every horizon step must be feasible too.
        for k in 0..MPC_H {
            let w = warm.get(k).copied().unwrap_or(0.0);
            assert!(w >= cfg.u_min - 1.0e-3 && w <= cfg.u_max + 1.0e-3);
        }
    }

    #[test]
    fn mpc_closed_loop_converges_under_tight_box() {
        // Simulate double-integrator closed-loop with MPC driving u.
        // Initial offset (1, 0), u constrained to [-3, 3] — should
        // converge to ~0 in a few seconds even with constraint active.
        let cfg = default_mpc_config(true);
        let mpc = Mpc1d::<MPC_H>::new(cfg).unwrap();
        let mut warm = nalgebra::SVector::<f32, MPC_H>::zeros();
        let (mut pos, mut vel) = (1.0_f32, 0.0_f32);
        for _ in 0..200 {
            let e = nalgebra::Vector2::new(pos, vel);
            let u = mpc.solve(e, &mut warm, 50);
            // Euler-integrate single-axis double integrator at cfg.dt_s.
            pos += vel * cfg.dt_s + 0.5 * u * cfg.dt_s * cfg.dt_s;
            vel += u * cfg.dt_s;
        }
        assert!(
            pos.abs() < 0.05 && vel.abs() < 0.05,
            "converged pos={pos} vel={vel}"
        );
    }

    // ------------------------------------------------------------------
    // M9.5 MPC-I tests
    // ------------------------------------------------------------------

    fn default_mpci_config(tight: bool) -> Mpc1dIConfig {
        Mpc1dIConfig {
            weights: LqiWeights {
                q_pos: 4.0,
                q_vel: 1.0,
                q_i: 1.0,
                r: 0.5,
            },
            dt_s: 0.05,
            u_min: if tight { -3.0 } else { -100.0 },
            u_max: if tight { 3.0 } else { 100.0 },
        }
    }

    #[test]
    fn mpci_new_rejects_bad_inputs() {
        let mut cfg = default_mpci_config(false);
        cfg.dt_s = 0.0;
        assert!(Mpc1dI::<MPC_H>::new(cfg).is_none());
        cfg.dt_s = f32::NAN;
        assert!(Mpc1dI::<MPC_H>::new(cfg).is_none());
        let mut cfg = default_mpci_config(false);
        cfg.weights.q_i = -1.0;
        assert!(Mpc1dI::<MPC_H>::new(cfg).is_none());
        let mut cfg = default_mpci_config(false);
        cfg.u_min = 1.0;
        cfg.u_max = -1.0;
        assert!(Mpc1dI::<MPC_H>::new(cfg).is_none());
    }

    #[test]
    fn mpci_zero_state_zero_command() {
        let cfg = default_mpci_config(false);
        let mpc = Mpc1dI::<MPC_H>::new(cfg).unwrap();
        let mut warm = nalgebra::SVector::<f32, MPC_H>::zeros();
        let u = mpc.solve(nalgebra::Vector3::zeros(), &mut warm, 100);
        assert!(u.abs() < 1.0e-4, "u at rest = {u}, expected ≈ 0");
    }

    #[test]
    fn mpci_respects_box_constraint() {
        // Tight u_max + large initial error → must clamp.
        let cfg = default_mpci_config(true);
        let mpc = Mpc1dI::<MPC_H>::new(cfg).unwrap();
        let z0 = nalgebra::Vector3::new(10.0_f32, 0.0, 0.0);
        let mut warm = nalgebra::SVector::<f32, MPC_H>::zeros();
        let u = mpc.solve(z0, &mut warm, 200);
        assert!(u >= cfg.u_min - 1.0e-3);
        assert!(u <= cfg.u_max + 1.0e-3);
        for k in 0..MPC_H {
            let w = warm.get(k).copied().unwrap_or(0.0);
            assert!(w >= cfg.u_min - 1.0e-3 && w <= cfg.u_max + 1.0e-3);
        }
    }

    #[test]
    fn mpci_integrator_zero_out_reduces_to_plain_mpc() {
        // With q_i = 0 the integrator state costs nothing and MPC-I's
        // optimal u_0 should closely track M9.1's Mpc1d from the same
        // initial (e_p, e_v). Use loose box to isolate the dynamics.
        let dt = 0.05_f32;
        let weights_qi0 = LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 0.0,
            r: 0.5,
        };
        let mpci = Mpc1dI::<MPC_H>::new(Mpc1dIConfig {
            weights: weights_qi0,
            dt_s: dt,
            u_min: -100.0,
            u_max: 100.0,
        })
        .unwrap();
        let plain = Mpc1d::<MPC_H>::new(Mpc1dConfig {
            weights: LqrWeights {
                q_pos: 4.0,
                q_vel: 1.0,
                r: 0.5,
            },
            dt_s: dt,
            u_min: -100.0,
            u_max: 100.0,
        })
        .unwrap();
        let mut warm_i = nalgebra::SVector::<f32, MPC_H>::zeros();
        let mut warm_p = nalgebra::SVector::<f32, MPC_H>::zeros();
        let u_i = mpci.solve(nalgebra::Vector3::new(0.5_f32, 0.0, 0.0), &mut warm_i, 500);
        let u_p = plain.solve(nalgebra::Vector2::new(0.5_f32, 0.0), &mut warm_p, 500);
        assert!((u_i - u_p).abs() < 0.05, "MPC-I {u_i} vs Mpc1d {u_p}");
    }

    #[test]
    fn mpci_controller_hover_at_origin() {
        let cfg = default_mpci_config(false);
        let mut ctl = MpcI3dPositionController::<MPC_H>::new(cfg, cfg, 50, 8.0, 5.0).unwrap();
        let out = ctl.step(
            &Setpoint::default(),
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
        );
        let hover = default_mass() * GRAVITY_M_S2;
        assert!((out.thrust_n - hover).abs() < 1.0e-3);
        assert!(out.q_desired.i.abs() < 1.0e-3 && out.q_desired.j.abs() < 1.0e-3);
    }

    #[test]
    fn mpci_controller_reset_clears_everything() {
        let cfg = default_mpci_config(false);
        let mut ctl = MpcI3dPositionController::<MPC_H>::new(cfg, cfg, 25, 8.0, 5.0).unwrap();
        // Drive off-setpoint so warm + integrator populate.
        let sp = Setpoint {
            position_ned: Vector3::new(0.5, -0.3, -0.8),
            ..Setpoint::default()
        };
        for _ in 0..80 {
            ctl.step(&sp, Vector3::zeros(), Vector3::zeros(), default_mass());
        }
        assert!(ctl.integrator().norm() > 0.0);
        ctl.reset();
        assert_eq!(ctl.integrator(), Vector3::zeros());
        assert_eq!(ctl.warm_x, nalgebra::SVector::<f32, MPC_H>::zeros());
        assert_eq!(ctl.warm_y, nalgebra::SVector::<f32, MPC_H>::zeros());
        assert_eq!(ctl.warm_z, nalgebra::SVector::<f32, MPC_H>::zeros());
    }

    #[test]
    fn mpci_is_a_position_controller_variant() {
        let cfg = default_mpci_config(false);
        let ctrl: PositionController<MPC_H> =
            PositionController::mpc_i(cfg, cfg, 25, 8.0, 5.0).unwrap();
        assert_eq!(ctrl.kind(), "MPC-I");
    }

    // ------------------------------------------------------------------
    // M9.4 PositionController enum dispatch tests
    // ------------------------------------------------------------------

    const SHOOT_H: usize = 10;

    #[test]
    fn controller_enum_constructs_every_variant() {
        let pi: PositionController<SHOOT_H> = PositionController::pi(PositionGains::default());
        assert_eq!(pi.kind(), "PI");

        let lqr = PositionController::<SHOOT_H>::lqr(
            LqrWeights::default(),
            LqrWeights::default(),
            0.001,
            8.0,
        )
        .unwrap();
        assert_eq!(lqr.kind(), "LQR");

        let mpc = PositionController::<SHOOT_H>::mpc(
            Mpc1dConfig {
                weights: LqrWeights::default(),
                dt_s: 0.001,
                u_min: -20.0,
                u_max: 20.0,
            },
            Mpc1dConfig {
                weights: LqrWeights::default(),
                dt_s: 0.001,
                u_min: -20.0,
                u_max: 20.0,
            },
            25,
            8.0,
        )
        .unwrap();
        assert_eq!(mpc.kind(), "MPC");

        let lqi = PositionController::<SHOOT_H>::lqi(
            LqiWeights::default(),
            LqiWeights::default(),
            0.001,
            8.0,
            5.0,
        )
        .unwrap();
        assert_eq!(lqi.kind(), "LQI");
    }

    #[test]
    fn controller_enum_each_variant_holds_hover_at_origin() {
        // At the setpoint with zero velocity, every controller should
        // command exactly hover thrust and level attitude — the cheap
        // cross-variant "do nothing when nothing is needed" check.
        let hover = default_mass() * GRAVITY_M_S2;
        let cases: [PositionController<SHOOT_H>; 4] = [
            PositionController::pi(PositionGains::default()),
            PositionController::lqr(LqrWeights::default(), LqrWeights::default(), 0.001, 8.0)
                .unwrap(),
            PositionController::mpc(
                Mpc1dConfig {
                    weights: LqrWeights::default(),
                    dt_s: 0.001,
                    u_min: -20.0,
                    u_max: 20.0,
                },
                Mpc1dConfig {
                    weights: LqrWeights::default(),
                    dt_s: 0.001,
                    u_min: -20.0,
                    u_max: 20.0,
                },
                20,
                8.0,
            )
            .unwrap(),
            PositionController::lqi(
                LqiWeights::default(),
                LqiWeights::default(),
                0.001,
                8.0,
                5.0,
            )
            .unwrap(),
        ];
        for mut c in cases {
            let kind = c.kind();
            let out = c.step(
                &Setpoint::default(),
                Vector3::zeros(),
                Vector3::zeros(),
                default_mass(),
                0.001,
            );
            assert!(
                (out.thrust_n - hover).abs() < 1.0e-3,
                "{kind} hover err {}",
                (out.thrust_n - hover).abs()
            );
            assert!(
                out.q_desired.i.abs() < 1.0e-3 && out.q_desired.j.abs() < 1.0e-3,
                "{kind} attitude not level"
            );
        }
    }

    #[test]
    fn controller_enum_reset_clears_state() {
        // Drive each stateful variant with a non-zero error so internal
        // buffers populate, then `reset` — verify state is zero by
        // stepping again from the origin and seeing a pure-gain
        // response (which, here, is just hover again).
        let hover = default_mass() * GRAVITY_M_S2;
        let disturbing_sp = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -5.0),
            ..Setpoint::default()
        };
        let variants: [fn() -> PositionController<SHOOT_H>; 3] = [
            || {
                PositionController::pi(PositionGains {
                    k_i_vel: Vector3::new(0.5, 0.5, 0.5),
                    ..PositionGains::default()
                })
            },
            || {
                PositionController::mpc(
                    Mpc1dConfig {
                        weights: LqrWeights::default(),
                        dt_s: 0.001,
                        u_min: -20.0,
                        u_max: 20.0,
                    },
                    Mpc1dConfig {
                        weights: LqrWeights::default(),
                        dt_s: 0.001,
                        u_min: -20.0,
                        u_max: 20.0,
                    },
                    20,
                    8.0,
                )
                .unwrap()
            },
            || {
                PositionController::lqi(
                    LqiWeights::default(),
                    LqiWeights::default(),
                    0.001,
                    8.0,
                    5.0,
                )
                .unwrap()
            },
        ];
        for make in variants {
            let mut c = make();
            let kind = c.kind();
            // Drive a few steps far from setpoint.
            for _ in 0..50 {
                c.step(
                    &disturbing_sp,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    default_mass(),
                    0.001,
                );
            }
            c.reset();
            // With state cleared and zero error, output should be hover.
            let out = c.step(
                &Setpoint::default(),
                Vector3::zeros(),
                Vector3::zeros(),
                default_mass(),
                0.001,
            );
            assert!(
                (out.thrust_n - hover).abs() < 1.0e-3,
                "{kind} after reset thrust {}, expected {}",
                out.thrust_n,
                hover
            );
        }
    }

    // ------------------------------------------------------------------
    // M9.3 LQI tests
    // ------------------------------------------------------------------

    fn default_lqi_weights() -> LqiWeights {
        LqiWeights {
            q_pos: 4.0,
            q_vel: 1.0,
            q_i: 0.5,
            r: 0.5,
        }
    }

    #[test]
    fn lqi_default_weights_produce_positive_gains() {
        let g = compute_lqi_gains(default_lqi_weights(), 0.05).unwrap();
        assert!(g.k_p > 0.0 && g.k_v > 0.0 && g.k_i > 0.0);
    }

    #[test]
    fn lqi_with_zero_qi_matches_lqr() {
        // q_i = 0 ⇒ the integrator state is free (doesn't cost anything)
        // and the LQI (k_p, k_v) should equal the LQR gains for the same
        // (q_pos, q_vel, r). Also k_i ≈ 0.
        let lqr = compute_lqr_gains(
            LqrWeights {
                q_pos: 4.0,
                q_vel: 1.0,
                r: 0.5,
            },
            0.05,
        )
        .unwrap();
        let mut w = default_lqi_weights();
        w.q_i = 0.0;
        let lqi = compute_lqi_gains(w, 0.05).unwrap();
        assert!(
            (lqi.k_p - lqr.k_p).abs() < 0.01,
            "k_p {} vs {}",
            lqi.k_p,
            lqr.k_p
        );
        assert!(
            (lqi.k_v - lqr.k_v).abs() < 0.01,
            "k_v {} vs {}",
            lqi.k_v,
            lqr.k_v
        );
        assert!(lqi.k_i.abs() < 0.01, "k_i should be ≈0, got {}", lqi.k_i);
    }

    #[test]
    fn lqi_invalid_inputs_return_none() {
        assert!(compute_lqi_gains(default_lqi_weights(), 0.0).is_none());
        let mut w = default_lqi_weights();
        w.r = 0.0;
        assert!(compute_lqi_gains(w, 0.05).is_none());
        let mut w = default_lqi_weights();
        w.q_pos = -1.0;
        assert!(compute_lqi_gains(w, 0.05).is_none());
    }

    #[test]
    fn lqi_higher_q_i_produces_larger_k_i() {
        // Monotone response: raising the integrator weight q_i must
        // raise k_i (the DARE solution is unique for positive Q/R).
        let mut w_soft = default_lqi_weights();
        w_soft.q_i = 0.1;
        let mut w_stiff = default_lqi_weights();
        w_stiff.q_i = 2.0;
        let soft = compute_lqi_gains(w_soft, 0.05).unwrap();
        let stiff = compute_lqi_gains(w_stiff, 0.05).unwrap();
        assert!(
            stiff.k_i > soft.k_i,
            "k_i soft {} stiff {}",
            soft.k_i,
            stiff.k_i
        );
    }

    #[test]
    fn lqi_step_at_origin_holds_hover() {
        // Zero position error, zero velocity, zero integrator → command
        // is zero acceleration → thrust exactly cancels gravity.
        let mut ctl = Lqi3dPositionController::new(
            default_lqi_weights(),
            default_lqi_weights(),
            0.01,
            20.0,
            10.0,
        )
        .unwrap();
        let out = ctl.step(
            &Setpoint::default(),
            Vector3::zeros(),
            Vector3::zeros(),
            0.25,
        );
        let hover = 0.25 * GRAVITY_M_S2;
        assert!((out.thrust_n - hover).abs() < 1.0e-3);
        assert!(out.q_desired.i.abs() < 1.0e-3 && out.q_desired.j.abs() < 1.0e-3);
    }

    #[test]
    fn lqi_reset_integrator_zeros_state() {
        let mut ctl = Lqi3dPositionController::new(
            default_lqi_weights(),
            default_lqi_weights(),
            0.01,
            20.0,
            10.0,
        )
        .unwrap();
        // One step at a bias-generating condition populates integrator.
        let sp = Setpoint {
            position_ned: Vector3::new(1.0, -0.5, 0.3),
            ..Setpoint::default()
        };
        for _ in 0..100 {
            ctl.step(&sp, Vector3::zeros(), Vector3::zeros(), 0.25);
        }
        assert!(ctl.integrator().norm() > 0.0);
        ctl.reset_integrator();
        assert_eq!(ctl.integrator(), Vector3::zeros());
    }

    #[test]
    fn mpc3d_new_rejects_mixed_dt() {
        let xy = Mpc1dConfig {
            weights: LqrWeights::default(),
            dt_s: 0.05,
            u_min: -10.0,
            u_max: 10.0,
        };
        let z = Mpc1dConfig {
            weights: LqrWeights::default(),
            dt_s: 0.02,
            u_min: -10.0,
            u_max: 10.0,
        };
        assert!(Mpc3dPositionController::<8>::new(xy, z, 40, 8.0).is_none());
    }

    #[test]
    fn mpc3d_hover_at_origin_is_stationary() {
        let cfg = default_mpc_config(false);
        let mut ctl = Mpc3dPositionController::<MPC_H>::new(cfg, cfg, 50, 8.0).unwrap();
        let out = ctl.step(
            &Setpoint::default(),
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
        );
        // At origin with zero setpoint: thrust must cancel gravity, q is
        // level (identity up to sign).
        let hover = default_mass() * GRAVITY_M_S2;
        assert!((out.thrust_n - hover).abs() < 1.0e-3);
        assert!(out.q_desired.i.abs() < 1.0e-3 && out.q_desired.j.abs() < 1.0e-3);
    }

    #[test]
    fn mpc3d_reset_warm_start_zeros_buffers() {
        let cfg = default_mpc_config(false);
        let mut ctl = Mpc3dPositionController::<MPC_H>::new(cfg, cfg, 20, 8.0).unwrap();
        // One step from a non-trivial state populates warm buffers.
        let sp = Setpoint {
            position_ned: Vector3::new(0.0, 0.0, -1.0),
            ..Setpoint::default()
        };
        ctl.step(&sp, Vector3::zeros(), Vector3::zeros(), default_mass());
        // Reset must clear everything.
        ctl.reset_warm_start();
        assert_eq!(ctl.warm_x, nalgebra::SVector::<f32, MPC_H>::zeros());
        assert_eq!(ctl.warm_y, nalgebra::SVector::<f32, MPC_H>::zeros());
        assert_eq!(ctl.warm_z, nalgebra::SVector::<f32, MPC_H>::zeros());
    }

    #[test]
    fn mpc_zero_error_produces_zero_command() {
        let cfg = default_mpc_config(false);
        let mpc = Mpc1d::<MPC_H>::new(cfg).unwrap();
        let mut warm = nalgebra::SVector::<f32, MPC_H>::zeros();
        let u = mpc.solve(nalgebra::Vector2::zeros(), &mut warm, 100);
        assert!(u.abs() < 1.0e-4, "expected u≈0, got {u}");
    }

    #[test]
    fn lqr_zero_error_produces_zero_feedback() {
        // Plug LQR gains into the existing position_to_attitude_thrust
        // and verify that "at the setpoint" produces hover thrust + no
        // tilt — i.e. the LQR feedback term is exactly zero.
        let pg =
            lqr_position_gains(LqrWeights::default(), LqrWeights::default(), 0.05, 8.0).unwrap();
        let out = position_to_attitude_thrust(
            &Setpoint::default(),
            Vector3::zeros(),
            Vector3::zeros(),
            default_mass(),
            &pg,
        );
        let hover = default_mass() * GRAVITY_M_S2;
        assert!((out.thrust_n - hover).abs() < 1.0e-3);
        assert!(out.q_desired.i.abs() < 1.0e-3 && out.q_desired.j.abs() < 1.0e-3);
    }
}
