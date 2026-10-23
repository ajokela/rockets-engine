//! Minimal scripted control input; no targeting or attitude controller is built into the engine.
use rockets_engine::{
    ControlInput, FlightState, GuidanceContext, IntegratorSettings, Rocket, Simulator,
    UniformGravity, Vec3,
};

fn main() -> rockets_engine::Result<()> {
    let simulator = Simulator::new(
        Rocket::new(1.0, None)?,
        UniformGravity::default(),
        IntegratorSettings {
            max_steering_accel_mps2: 0.5,
            ..Default::default()
        },
    )?;
    let mut calls = 0;
    let mut controller = |context: GuidanceContext<'_>| {
        calls += 1;
        // The caller supplies control authority; the engine caps it to 0.5 m/s².
        Ok(ControlInput {
            thrust_direction: Vec3::x(),
            steering_acceleration_mps2: Vec3::new(
                0.0,
                0.0,
                if context.state.time_s < 0.5 {
                    0.2
                } else {
                    -0.2
                },
            ),
        })
    };
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 20.0, 0.0),
        velocity_mps: Vec3::new(5.0, 0.0, 0.0),
    };
    for _ in 0..100 {
        state = simulator.step_guided(&state, 0.01, &mut controller)?.state;
    }
    println!(
        "{calls} controller updates; final position: {:?}",
        state.position_m
    );
    Ok(())
}
