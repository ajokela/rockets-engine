//! winds-engine + ballistics-engine: a trajectory through a 3-D wind field.
//!
//! The force model is ballistics-engine's kernel — Earth gravity, ICAO
//! atmosphere and a drag curve — adapted by `BallisticForces`, which samples
//! a winds-engine `WindField` at every integration substep. Here that field
//! is a terrain-adjusted one: it varies with *altitude* (a sheared profile
//! background) and with *horizontal position* (mass-consistent flow over a
//! ridge, bent by distributed surface wind monitors). The result is a wind
//! that is different at every (x, y, z), wired into an RK4 trajectory.
//!
//! Run with:
//!   cargo run --example wind_field_trajectory --features ballistics
use std::sync::Arc;

use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    ControlInput, FlightState, IntegratorSettings, Motor, Rocket, Simulator, ThrustCurve,
    ThrustPoint, Vec3,
};
use winds_engine::{
    HeightGrid, Monitor, ProfileField, ProfileLevel, TerrainFlowField, WindField,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // ---- A synthetic ridge DEM (metres, x downrange / z cross-range). ----
    let (ncols, nrows) = (41usize, 21usize);
    let mut heights = Vec::with_capacity(ncols * nrows);
    for _i in 0..nrows {
        for j in 0..ncols {
            let x = (j as f64 - 20.0) * 100.0; // -2000 .. 2000 m
            heights.push(300.0 * (-(x * x) / (500.0_f64 * 500.0)).exp());
        }
    }
    let grid = HeightGrid::new(-2000.0, -1000.0, 100.0, 100.0, ncols, nrows, heights)?;

    // ---- Two wind monitors on the range floor (speed, FROM-direction). ----
    let monitors = [
        Monitor::at(-1000.0, 0.0, 3.0, 180.0), // 3 m/s from the east
        Monitor::at(1000.0, 0.0, 4.0, 160.0),  // 4 m/s from the south-east
    ];

    // ---- Background: a boundary-layer profile (shear with altitude). ----
    let background = ProfileField::new(vec![
        ProfileLevel {
            altitude_m: 0.0,
            speed_mps: 2.0,
            direction_from_deg: 180.0,
        },
        ProfileLevel {
            altitude_m: 200.0,
            speed_mps: 8.0,
            direction_from_deg: 180.0,
        },
        ProfileLevel {
            altitude_m: 2000.0,
            speed_mps: 12.0,
            direction_from_deg: 180.0,
        },
    ])?;

    // ---- The 3-D field: terrain-adjusted, monitor-constrained. ----
    let field = TerrainFlowField::adjust(
        grid.clone(),
        &background,
        &monitors,
        &[10.0, 50.0, 150.0, 500.0],
        1500.0, // monitor influence radius, m
        300.0,  // monitor vertical relaxation, m
        400.0,  // terrain forcing depth, m
    )?;

    // ---- Show it is genuinely 3-D. ----
    println!("wind at 50 m AGL, moving downrange over the ridge:");
    for x in [-1200.0, -600.0, 0.0, 600.0, 1200.0] {
        let w = field.wind(0.0, [x, grid.height_at(x, 0.0) + 50.0, 0.0]);
        println!("  x = {x:6} m   {:+.2} m/s east, {:+.2} m/s north   ({} m/s)", w.x_mps, w.z_mps, w.speed_mps());
    }
    println!("wind at the crest (x = 0), climbing:");
    for agl in [10.0, 150.0, 500.0, 2000.0] {
        let w = field.wind(0.0, [0.0, grid.height_at(0.0, 0.0) + agl, 0.0]);
        println!("  {agl:5} m AGL  {:+.2} m/s east, {:+.2} m/s north", w.x_mps, w.z_mps);
    }

    // ---- Fly a rocket through it vs still air. ----
    let curve = ThrustCurve::new(vec![
        ThrustPoint { time_s: 0.0, thrust_n: 0.0 },
        ThrustPoint { time_s: 0.1, thrust_n: 18.0 },
        ThrustPoint { time_s: 0.8, thrust_n: 6.0 },
        ThrustPoint { time_s: 1.0, thrust_n: 0.0 },
    ])?;
    let motor = Motor::new(curve, 0.012, 0.0)?; // 12 g propellant
    let rocket = Rocket::new(0.08, Some(motor))?;

    let trajectory = |wind: Option<Arc<dyn WindField + Send + Sync>>| -> Result<f64, Box<dyn std::error::Error>> {
        let cd = vec![(0.0, 0.5), (0.8, 0.55), (3.0, 0.5)];
        let forces = BallisticForces::new(0.025, cd, Atmosphere::default(), Vec3::zeros())?
            .apply(wind)?;
        let sim = Simulator::new(rocket.clone(), forces, IntegratorSettings::default())?;
        // Launch tilted 80° from vertical so the flight travels downrange
        // through the ridge's speed-up, shelter and monitor shear.
        let axis = Vec3::new(0.17, 0.98, 0.0);
        let mut state = FlightState {
            time_s: 0.0,
            position_m: Vec3::new(-1200.0, grid.height_at(-1200.0, 0.0), 0.0),
            velocity_mps: Vec3::zeros(),
        };
        let mut apogee = state.position_m.y;
        for _ in 0..600 {
            state = sim.step(&state, 0.05, ControlInput::along(axis))?.state;
            apogee = apogee.max(state.position_m.y);
            if state.time_s > 1.0 && state.velocity_mps.y <= 0.0 && state.position_m.y < apogee - 2.0 {
                break;
            }
        }
        Ok(apogee)
    };

    let still = trajectory(None)?;
    let windy = trajectory(Some(Arc::new(field)))?;
    println!("\napogee in still air: {still:.1} m");
    println!("apogee through the 3-D terrain wind: {windy:.1} m");
    println!("wind-shifted by {:.1} m", windy - still);

    Ok(())
}

/// Apply an optional wind field to a force model (None = calm air).
trait WindFieldExt {
    fn apply(self, wind: Option<Arc<dyn WindField + Send + Sync>>) -> Result<Self, rockets_engine::Error>
    where
        Self: Sized;
}
impl WindFieldExt for BallisticForces {
    fn apply(self, wind: Option<Arc<dyn WindField + Send + Sync>>) -> Result<Self, rockets_engine::Error> {
        match wind {
            Some(field) => self.with_wind_field(field),
            None => Ok(self),
        }
    }
}