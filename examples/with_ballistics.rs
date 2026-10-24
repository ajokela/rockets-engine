//! Using rockets-engine and ballistics-engine together.
//!
//! rockets-engine owns the powered flight (motor, mass, integration, guidance);
//! ballistics-engine supplies the atmosphere and drag kernel through the
//! `BallisticForces` adapter. No fork of either crate is needed.
//!
//! Run with: cargo run --example with_ballistics --features ballistics
use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    ControlInput, FlightState, IntegratorSettings, Motor, Rocket, Simulator, ThrustCurve,
    ThrustPoint, Vec3,
};

fn main() -> Result<(), rockets_engine::Error> {
    // rockets-engine: a prescribed motor burn with propellant mass.
    let curve = ThrustCurve::new(vec![
        ThrustPoint {
            time_s: 0.0,
            thrust_n: 0.0,
        },
        ThrustPoint {
            time_s: 0.1,
            thrust_n: 20.0,
        },
        ThrustPoint {
            time_s: 0.9,
            thrust_n: 15.0,
        },
        ThrustPoint {
            time_s: 1.0,
            thrust_n: 0.0,
        },
    ])?;
    let motor = Motor::new(curve, 0.04, 0.2)?; // 40 g propellant, ignition at 0.2 s
    let rocket = Rocket::new(0.18, Some(motor))?;

    // ballistics-engine: gravity, ICAO atmosphere and drag for a 25 mm body,
    // with a 2 m/s tailwind. This is the same kernel the `ballistics` CLI uses.
    let forces = BallisticForces::new(
        0.025,
        vec![(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)], // Mach:Cd pairs
        Atmosphere::default(),
        Vec3::new(2.0, 0.0, 0.0),
    )?;

    let simulator = Simulator::new(rocket, forces, IntegratorSettings::default())?;
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 2.0, 0.0),
        velocity_mps: Vec3::new(2.0, 8.0, 0.0),
    };
    for _ in 0..100 {
        let result = simulator.step(
            &state,
            0.05,
            ControlInput::along(Vec3::new(0.3, 1.0, 0.0)),
        )?;
        state = result.state;
        if state.time_s % 0.5 < 1e-9 {
            println!(
                "t = {:4.1} s | altitude {:7.2} m | speed {:6.2} m/s | mass {:.3} kg",
                state.time_s,
                state.position_m.y,
                state.velocity_mps.norm(),
                result.rocket.mass_kg
            );
        }
    }
    Ok(())
}
