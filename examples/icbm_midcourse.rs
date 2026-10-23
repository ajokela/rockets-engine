//! ICBM-style midcourse coast in an Earth-centered inertial (ECI) frame.
//!
//! A post-boost state at 300 km altitude coasts under central gravity — no
//! atmosphere, no motor — until it falls back to the 120 km reentry interface,
//! where a host would hand the state to a dense-atmosphere model. Prints the
//! apogee, surface range, handoff conditions and how well specific orbital
//! energy was conserved. The parameters are illustrative.
use rockets_engine::{
    CentralGravity, ControlInput, FlightState, IntegratorSettings, Rocket, Simulator, Vec3,
    EARTH_MU_M3_S2,
};

const R_EARTH_M: f64 = 6_371_000.0;
const HANDOFF_ALTITUDE_M: f64 = 120_000.0;

fn main() -> rockets_engine::Result<()> {
    let simulator = Simulator::new(
        Rocket::new(500.0, None)?, // Post-boost vehicle: warhead bus, no motor.
        CentralGravity::earth(Vec3::zeros())?,
        IntegratorSettings {
            max_step_s: 1.0, // Vacuum coast needs no fine motor grid.
            ..Default::default()
        },
    )?;

    // Post-boost state: 300 km altitude, 6.5 km/s at a 25 degree flight-path
    // angle. Position is measured from Earth's center; +Y is the local radial.
    let altitude_m = 300_000.0;
    let speed_mps = 6_500.0;
    let flight_path_rad = 25.0_f64.to_radians();
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, R_EARTH_M + altitude_m, 0.0),
        velocity_mps: Vec3::new(
            speed_mps * flight_path_rad.cos(),
            speed_mps * flight_path_rad.sin(),
            0.0,
        ),
    };
    let energy0 =
        state.velocity_mps.norm_squared() * 0.5 - EARTH_MU_M3_S2 / state.position_m.norm();

    let start_radius = state.position_m.norm();
    let mut apogee_altitude_m: f64 = 0.0;
    loop {
        let result = simulator.step(&state, 5.0, ControlInput::default())?;
        state = result.state;
        let radius = state.position_m.norm();
        apogee_altitude_m = apogee_altitude_m.max(radius - R_EARTH_M);
        // Past apogee (r and v point apart), stop at the reentry interface.
        // The 5 s cadence overshoots by up to one step at ~7 km/s; a host that
        // cares about the exact crossing would shrink the final step.
        if state.position_m.dot(&state.velocity_mps) < 0.0
            && radius <= R_EARTH_M + HANDOFF_ALTITUDE_M
        {
            break;
        }
        if state.time_s > 3_600.0 {
            break;
        }
    }

    let energy = state.velocity_mps.norm_squared() * 0.5 - EARTH_MU_M3_S2 / state.position_m.norm();
    // Angle swept around Earth's center from the launch radius (0, r, 0).
    let central_angle_rad = (state.position_m.y / state.position_m.norm()).acos();
    println!("apogee_km,{:.1}", apogee_altitude_m / 1000.0);
    println!("flight_time_s,{:.1}", state.time_s);
    println!(
        "surface_range_km,{:.1}",
        start_radius * central_angle_rad / 1000.0
    );
    println!(
        "handoff_altitude_km,{:.1}",
        (state.position_m.norm() - R_EARTH_M) / 1000.0
    );
    println!("handoff_speed_mps,{:.1}", state.velocity_mps.norm());
    println!(
        "energy_drift_relative,{:.3e}",
        (energy - energy0) / energy0.abs()
    );
    Ok(())
}
