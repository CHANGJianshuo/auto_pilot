//! Controller shootout — executable benchmark.
//!
//! Runs every position-loop controller the workspace exposes through the
//! same realistic-sim scenario (wind + drag + motor lag) and prints a
//! markdown table of horizontal / altitude tracking error. Useful when
//! you want to quote numbers in the README without reading six unit
//! tests.
//!
//! Run with:
//!
//! ```bash
//! cargo run -p sim-hil --example controller_shootout --release
//! ```
//!
//! Output shape:
//!
//! ```text
//! Scenario: wind (1.5, 0, 0) m/s + realistic drag + motor lag, 15 s
//! ┌────────────────┬───────────────────┬─────────────────┐
//! │ Controller     │ horizontal err (m) │ altitude err (m) │
//! ├────────────────┼───────────────────┼─────────────────┤
//! │ PI cascade     │               ... │             ... │
//! │ LQR            │               ... │             ... │
//! │ MPC            │               ... │             ... │
//! │ LQI            │               ... │             ... │
//! │ MPC-I          │               ... │             ... │
//! │ MPC + residual │               ... │             ... │
//! └────────────────┴───────────────────┴─────────────────┘
//! ```
//!
//! Not a test — failures print and exit non-zero, but the numeric
//! thresholds live in `controller_shootout_residual_sits_between_*` in
//! `sim-hil/src/lib.rs`.

// Examples are allowed to use unwrap / expect / as-conversions; they
// are illustrative code, not flight-critical firmware.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::as_conversions)]

use algo_nmpc::{
    LqiWeights, LqrWeights, Mpc1dConfig, Mpc1dIConfig, Mpc3dPositionController, PositionController,
    PositionGains,
};
use nalgebra::Vector3;
use nn_runtime::{AffineBackend, ResidualPolicy, SafetyEnvelope};
use sim_hil::{
    SimConfig,
    residual_mpc::MpcResidualController,
    sitl::{SitlResult, SitlScenario, run_with_controller, run_with_mpc_residual},
};

const H: usize = 10;

fn main() {
    let scenario = SitlScenario {
        sim_cfg: SimConfig::realistic_dynamics(Vector3::new(1.5, 0.0, 0.0)),
        seed: 13,
        ticks: 15_000,
        ..Default::default()
    };

    let dt = scenario.dt_s;
    let lqr_weights = LqrWeights {
        q_pos: 4.0,
        q_vel: 1.0,
        r: 0.5,
    };
    let lqi_weights = LqiWeights {
        q_pos: 4.0,
        q_vel: 1.0,
        q_i: 1.5,
        r: 0.5,
    };
    let max_accel = PositionGains::default().max_accel;

    let mut rows: Vec<(&'static str, SitlResult)> = Vec::new();

    // 1. PI cascade — default gains with a modest integrator.
    let mut pi: PositionController<H> = PositionController::pi(PositionGains {
        k_i_vel: Vector3::new(0.5, 0.5, 0.8),
        ..PositionGains::default()
    });
    rows.push(("PI cascade", run_with_controller(&scenario, &mut pi)));

    // 2. LQR — cost-optimal feedback, no integrator.
    let mut lqr =
        PositionController::<H>::lqr(lqr_weights, lqr_weights, dt, max_accel).expect("LQR DARE");
    rows.push(("LQR", run_with_controller(&scenario, &mut lqr)));

    // 3. MPC — constraint-aware, no integrator.
    let cfg_mpc = Mpc1dConfig {
        weights: lqr_weights,
        dt_s: dt,
        u_min: -20.0,
        u_max: 20.0,
    };
    let mut mpc = PositionController::<H>::mpc(cfg_mpc, cfg_mpc, 25, max_accel).expect("MPC");
    rows.push(("MPC", run_with_controller(&scenario, &mut mpc)));

    // 4. LQI — LQR + integrator, no horizon.
    let mut lqi =
        PositionController::<H>::lqi(lqi_weights, lqi_weights, dt, max_accel, 5.0).expect("LQI");
    rows.push(("LQI", run_with_controller(&scenario, &mut lqi)));

    // 5. MPC-I — constraint-aware + integrator.
    let cfg_mpci = Mpc1dIConfig {
        weights: lqi_weights,
        dt_s: dt,
        u_min: -20.0,
        u_max: 20.0,
    };
    let mut mpci =
        PositionController::<H>::mpc_i(cfg_mpci, cfg_mpci, 25, max_accel, 5.0).expect("MPC-I");
    rows.push(("MPC-I", run_with_controller(&scenario, &mut mpci)));

    // 6. MPC + residual — hand-tuned PD affine.
    let inner_mpc = Mpc3dPositionController::<H>::new(cfg_mpc, cfg_mpc, 25, max_accel).unwrap();
    let mut w = [[0.0_f32; nn_runtime::FEATURE_LEN]; nn_runtime::RESIDUAL_LEN];
    w[0][0] = -2.0; // pos_err_x → residual_x
    w[0][3] = -1.0; // vel_x     → residual_x
    let backend = AffineBackend::new(w, [0.0; nn_runtime::RESIDUAL_LEN], 5.0);
    let policy = ResidualPolicy::new(backend, SafetyEnvelope::small_multirotor_default());
    let mut residual = MpcResidualController::new(inner_mpc, policy);
    let residual_result = run_with_mpc_residual(&scenario, &mut residual);
    rows.push(("MPC + residual", residual_result));

    println!(
        "Scenario: wind {:?} m/s + realistic drag + motor lag, {:.1} s",
        scenario.sim_cfg.wind_ned,
        scenario.ticks as f32 * scenario.dt_s
    );
    println!();
    println!("| Controller     | horizontal err (m) | altitude err (m) |");
    println!("|----------------|--------------------|------------------|");
    for (name, r) in &rows {
        println!(
            "| {:<14} | {:>18.3} | {:>16.3} |",
            name, r.horizontal_err_m, r.altitude_err_m
        );
    }
    println!();
    println!("Ordering by horizontal error (smaller is better):");
    let mut ranked = rows.clone();
    ranked.sort_by(|a, b| {
        a.1.horizontal_err_m
            .partial_cmp(&b.1.horizontal_err_m)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (i, (name, r)) in ranked.iter().enumerate() {
        println!("  {}. {:<14} — {:.3} m", i + 1, name, r.horizontal_err_m);
    }
}
