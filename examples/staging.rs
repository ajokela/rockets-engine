//! Synthetic three-stage sounding rocket; the parameters are illustrative.
//! Serial staging with a short coast between burnout and next ignition, printed
//! as CSV. Watch the mass drop at each separation.
use rockets_engine::{
    ControlInput, FlightState, IntegratorSettings, Motor, Rocket, Simulator, Stage, ThrustCurve,
    ThrustPoint, UniformGravity, Vec3,
};

fn constant_motor(
    thrust_n: f64,
    duration_s: f64,
    propellant_kg: f64,
    ignition_s: f64,
) -> rockets_engine::Result<Motor> {
    let curve = ThrustCurve::new(vec![
        ThrustPoint {
            time_s: 0.0,
            thrust_n,
        },
        ThrustPoint {
            time_s: duration_s,
            thrust_n,
        },
    ])?;
    Motor::new(curve, propellant_kg, ignition_s)
}

fn main() -> rockets_engine::Result<()> {
    let rocket = Rocket::staged(
        0.05, // Payload stays aboard for the whole flight.
        vec![
            Stage::new(0.05, Some(constant_motor(24.0, 0.6, 0.04, 0.0)?), Some(0.7))?,
            Stage::new(0.04, Some(constant_motor(16.0, 0.5, 0.03, 0.8)?), Some(1.4))?,
            Stage::new(0.03, Some(constant_motor(12.0, 0.4, 0.02, 1.5)?), None)?,
        ],
    )?;
    let simulator = Simulator::new(
        rocket,
        UniformGravity::default(),
        IntegratorSettings::default(),
    )?;
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 2.0, 0.0),
        velocity_mps: Vec3::zeros(),
    };
    println!("time_s,x_m,altitude_m,speed_mps,mass_kg,thrust_n,phase");
    for _ in 0..400 {
        let result = simulator.step(&state, 0.05, ControlInput::along(Vec3::new(0.3, 1.0, 0.0)))?;
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
        if state.position_m.y < 0.0 {
            break;
        }
    }
    Ok(())
}
