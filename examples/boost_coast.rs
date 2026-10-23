//! Synthetic small rocket; the parameters are illustrative, not a motor specification.
use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    FixedDirection, FlightState, IntegratorSettings, Motor, Rocket, Simulator, ThrustCurve,
    ThrustPoint, Vec3,
};

fn main() -> rockets_engine::Result<()> {
    let curve = ThrustCurve::new(vec![
        ThrustPoint {
            time_s: 0.0,
            thrust_n: 0.0,
        },
        ThrustPoint {
            time_s: 0.1,
            thrust_n: 24.0,
        },
        ThrustPoint {
            time_s: 0.8,
            thrust_n: 18.0,
        },
        ThrustPoint {
            time_s: 1.0,
            thrust_n: 0.0,
        },
    ])?;
    let motor = Motor::new(curve, 0.04, 0.2)?;
    let rocket = Rocket::new(0.18, Some(motor))?;
    let environment = BallisticForces::new(
        0.025,
        vec![(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)],
        Atmosphere::default(),
        Vec3::new(2.0, 0.0, 0.0),
    )?;
    let simulator = Simulator::new(rocket, environment, IntegratorSettings::default())?;
    let mut guidance = FixedDirection(Vec3::new(0.3, 1.0, 0.0));
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 2.0, 0.0),
        velocity_mps: Vec3::new(2.0, 8.0, 0.0),
    };
    println!("time_s,x_m,altitude_m,speed_mps,mass_kg,thrust_n,phase");
    for _ in 0..200 {
        let result = simulator.step_guided(&state, 0.05, &mut guidance)?;
        state = result.state;
        println!(
            "{:.3},{:.4},{:.4},{:.4},{:.5},{:.3},{:?}",
            state.time_s,
            state.position_m.x,
            state.position_m.y,
            state.velocity_mps.norm(),
            result.rocket.mass_kg,
            result.rocket.motor.thrust_n,
            result.rocket.motor.phase
        );
        // Coarse host-side termination for this demonstration. The library itself
        // does not localize terrain collisions or apply launch-rail constraints.
        if state.position_m.y < 0.0 {
            break;
        }
    }
    Ok(())
}
