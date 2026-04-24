// Example is host-only and interactive; relaxing the strictest
// workspace lints here keeps the visualisation code readable.
#![allow(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::unwrap_used,
    clippy::too_many_lines
)]

//! 3D visualisation of the SITL flight stack via rerun.io.
//!
//! Run:
//!   cargo run -p sim-hil --example sitl_rerun --features rerun-viz --release
//!
//! This spawns the rerun native viewer in a window. Once it comes up
//! you'll see:
//!
//!   * **3D scene**:
//!       - world axes at the origin
//!       - a vehicle mesh (box + 4 motor arms) at the EKF-estimated pose
//!       - a trailing line-strip showing the actual sim trajectory
//!       - a thrust vector arrow in body -z (what the controller is
//!         asking the motors to produce)
//!       - the setpoint as a small sphere
//!   * **Time series** (scroll via the timeline at the bottom):
//!       - altitude / horizontal error
//!       - per-motor thrust
//!       - body rates (p, q, r)
//!       - EKF wind estimate
//!
//! The scenario is a simple 8-figure trajectory — re-using the M17
//! figure-8 benchmark so the visualisation shows a meaningful
//! aerial dance, not just hover.

use algo_ekf::GRAVITY_M_S2;
use algo_indi::attitude_to_rate;
use algo_nmpc::{LqiWeights, Mpc1dIConfig, PositionController, PositionGains, Setpoint};
use app_copter::{
    ArmState, FlightState, apply_baro_measurement, apply_gps_measurement, apply_mag_measurement,
    default_config_250g, rate_loop_step,
};
use nalgebra::{SVector, Vector3};
use sim_hil::{
    SimConfig, SimRng, SimState, accel_world, sense_baro, sense_gps, sense_imu, sense_mag,
    sitl::figure_eight, step,
};

fn main() -> anyhow::Result<()> {
    // Spawn the native viewer. This blocks on a separate thread; the
    // returned stream is what we log through.
    let rec = rerun::RecordingStreamBuilder::new("auto_pilot_sitl").spawn()?;

    // Static scene geometry — the world frame, logged once and then
    // referenced by every dynamic entity.
    rec.log_static(
        "world",
        &rerun::ViewCoordinates::new(rerun::components::ViewCoordinates::RIGHT_HAND_Z_DOWN),
    )?;
    rec.log_static(
        "world/axes",
        &rerun::Arrows3D::from_vectors([
            [1.0_f32, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        .with_colors([
            rerun::Color::from_rgb(255, 0, 0),
            rerun::Color::from_rgb(0, 255, 0),
            rerun::Color::from_rgb(0, 0, 255),
        ])
        .with_labels(["x (north)", "y (east)", "z (down)"]),
    )?;

    // --- Set up the SITL scenario ------------------------------------
    let sim_cfg = SimConfig::default();
    let mut sim_state = SimState {
        position_ned: Vector3::new(0.0, 0.0, -1.0),
        motor_thrusts_actual_n: SVector::<f32, 4>::repeat(
            sim_cfg.mass_kg * GRAVITY_M_S2 / 4.0,
        ),
        ..SimState::default()
    };
    let mut rng = SimRng::new(7);
    let mut app_cfg = default_config_250g();
    let mut flight = FlightState {
        arm_state: ArmState::Armed,
        ..FlightState::default()
    };
    let _ = apply_baro_measurement(&mut flight, &sense_baro(&sim_cfg, &sim_state, &mut rng));
    let _ = apply_gps_measurement(&mut flight, &sense_gps(&sim_cfg, &sim_state, &mut rng));

    // MPC-I position controller — the Phase-III benchmark winner on
    // moving targets.
    let lqi = LqiWeights {
        q_pos: 4.0,
        q_vel: 1.0,
        q_i: 1.5,
        r: 0.5,
    };
    let cfg = Mpc1dIConfig {
        weights: lqi,
        dt_s: 0.001,
        u_min: -20.0,
        u_max: 20.0,
    };
    let mut ctrl =
        PositionController::<10>::mpc_i(cfg, cfg, 25, PositionGains::default().max_accel, 5.0)
            .unwrap();

    let mut traj = figure_eight(2.0, 10.0, 1.0);
    let dt = 0.001_f32;
    let total_ticks: usize = 20_000; // 20 s = two full 8-figure orbits

    // Trajectory buffer — accumulates world positions for the trail
    // line-strip. Only pushed at 50 Hz so the line doesn't get
    // thousands of points.
    let mut trail: Vec<[f32; 3]> = Vec::with_capacity(total_ticks / 20);

    for i in 0..total_ticks {
        let t_s = (i as f32) * dt;
        rec.set_time_seconds("sim_time", f64::from(t_s));

        let sp = traj(t_s);

        // ---- Flight-stack step ---------------------------------------
        let accel_w = accel_world(&sim_cfg, &sim_state);
        let imu = sense_imu(&sim_cfg, &sim_state, accel_w, &mut rng);
        let att = ctrl.step(
            &sp,
            flight.state.position_ned,
            flight.state.velocity_ned,
            app_cfg.mass_kg,
            dt,
        );
        let rate_cmd =
            attitude_to_rate(flight.state.attitude, att.q_desired, &app_cfg.k_attitude);
        let saved_hover = app_cfg.hover_thrust_n;
        app_cfg.hover_thrust_n = att.thrust_n;
        let out = rate_loop_step(&mut app_cfg, &mut flight, imu, dt, rate_cmd);
        app_cfg.hover_thrust_n = saved_hover;
        step(&sim_cfg, &mut sim_state, &out.motor_thrusts_n, dt);

        // Measurement updates at the rates from docs/topics.md.
        if i.is_multiple_of(200) {
            let _ = apply_gps_measurement(
                &mut flight,
                &sense_gps(&sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(40) {
            let _ = apply_mag_measurement(
                &mut flight,
                &sense_mag(&sim_cfg, &sim_state, &mut rng),
            );
        }
        if i.is_multiple_of(20) {
            let _ = apply_baro_measurement(
                &mut flight,
                &sense_baro(&sim_cfg, &sim_state, &mut rng),
            );
        }

        // ---- Log to rerun every 10 ticks (100 Hz display rate) -------
        if !i.is_multiple_of(10) {
            continue;
        }

        let pos = sim_state.position_ned;
        let q = sim_state.attitude;
        let pos_arr = [pos.x, pos.y, pos.z];

        // Vehicle pose — a single Transform3D ties the whole vehicle
        // subtree (body + motors + thrust arrow) to the world frame.
        rec.log(
            "world/vehicle",
            &rerun::Transform3D::from_translation_rotation(
                pos_arr,
                rerun::Quaternion::from_xyzw([q.i, q.j, q.k, q.w]),
            ),
        )?;

        // Body box — 30×30×6 cm, centred on CG.
        rec.log(
            "world/vehicle/body",
            &rerun::Boxes3D::from_half_sizes([[0.15_f32, 0.15, 0.03]])
                .with_colors([rerun::Color::from_rgb(80, 80, 200)]),
        )?;

        // Motor dots at the four X-quad arm tips. Colour-code by thrust
        // magnitude so the rotor that's working hardest glows red.
        let h = 0.15_f32 / core::f32::consts::SQRT_2;
        let motor_positions: [[f32; 3]; 4] = [
            [h, h, 0.0],   // M0 front-right
            [-h, -h, 0.0], // M1 rear-left
            [h, -h, 0.0],  // M2 front-left
            [-h, h, 0.0],  // M3 rear-right
        ];
        let motor_colors: Vec<rerun::Color> = (0..4)
            .map(|mi| {
                let t = out.motor_thrusts_n[mi];
                let frac = (t / app_cfg.motor_max_n).clamp(0.0, 1.0);
                let r = (frac * 255.0) as u8;
                let g = ((1.0 - frac) * 255.0) as u8;
                rerun::Color::from_rgb(r, g, 30)
            })
            .collect();
        rec.log(
            "world/vehicle/motors",
            &rerun::Points3D::new(motor_positions)
                .with_radii([0.02_f32; 4])
                .with_colors(motor_colors),
        )?;

        // Thrust vector — body +z points downward in NED, so thrust is
        // along body -z. Scale to magnitude so a stronger thrust draws
        // a longer arrow.
        let thrust_body: [f32; 3] = [0.0, 0.0, -att.thrust_n / (app_cfg.mass_kg * GRAVITY_M_S2)];
        rec.log(
            "world/vehicle/thrust",
            &rerun::Arrows3D::from_vectors([thrust_body])
                .with_colors([rerun::Color::from_rgb(255, 200, 50)])
                .with_labels(["thrust"]),
        )?;

        // Setpoint — small sphere at the target position, cyan.
        let sp_pos = sp.position_ned;
        rec.log(
            "world/setpoint",
            &rerun::Points3D::new([[sp_pos.x, sp_pos.y, sp_pos.z]])
                .with_radii([0.04_f32])
                .with_colors([rerun::Color::from_rgb(0, 220, 220)]),
        )?;

        // Trail — downsampled at 50 Hz.
        if i.is_multiple_of(20) {
            trail.push(pos_arr);
            // rerun re-renders the whole line-strip each time; cap
            // trail length so render cost stays bounded.
            if trail.len() > 2000 {
                trail.remove(0);
            }
            rec.log(
                "world/trajectory",
                &rerun::LineStrips3D::new([trail.clone()])
                    .with_colors([rerun::Color::from_rgb(180, 180, 255)]),
            )?;
        }

        // ---- Time-series panels --------------------------------------
        rec.log(
            "ts/altitude_m",
            &rerun::Scalar::new(f64::from(-pos.z)),
        )?;
        rec.log(
            "ts/xy_err_m",
            &rerun::Scalar::new(f64::from(
                ((pos.x - sp_pos.x).powi(2) + (pos.y - sp_pos.y).powi(2)).sqrt(),
            )),
        )?;
        rec.log(
            "ts/body_rate/roll",
            &rerun::Scalar::new(f64::from(sim_state.body_rate_rad_s.x)),
        )?;
        rec.log(
            "ts/body_rate/pitch",
            &rerun::Scalar::new(f64::from(sim_state.body_rate_rad_s.y)),
        )?;
        rec.log(
            "ts/body_rate/yaw",
            &rerun::Scalar::new(f64::from(sim_state.body_rate_rad_s.z)),
        )?;
        rec.log(
            "ts/thrust_n",
            &rerun::Scalar::new(f64::from(att.thrust_n)),
        )?;
        for mi in 0..4 {
            rec.log(
                format!("ts/motor_{mi}_n"),
                &rerun::Scalar::new(f64::from(out.motor_thrusts_n[mi])),
            )?;
        }
        rec.log(
            "ts/wind_n",
            &rerun::Scalar::new(f64::from(flight.state.wind_ne.x)),
        )?;
        rec.log(
            "ts/wind_e",
            &rerun::Scalar::new(f64::from(flight.state.wind_ne.y)),
        )?;
    }

    tracing::info!("sitl_rerun: finished {total_ticks} ticks — leaving viewer open");
    // Keep the viewer alive; user kills with Ctrl-C or closes the window.
    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
