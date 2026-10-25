//! Every force reads the same air as the drag kernel, atmosphere zones
//! included: recovery drag, the weathercock and body lift.
#![cfg(all(feature = "model", feature = "ballistics"))]

use ballistics_engine::atmosphere::calculate_air_density_cimp;
use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    ControlInput, FlightState, ForceModel, Guidance, GuidanceContext, IntegratorSettings,
    MotorPhase, MotorSample, RecoveryForces, RocketSample, Simulator, UniformGravity, Vec3,
    Weathercock, WeathercockParams,
};

/// Half the sea-level pressure, at the default station (sea level, 15 °C, dry).
const LOW_ZONE: (f64, f64, f64, f64) = (15.0, 506.625, 0.0, 1e6);

fn low_station() -> Atmosphere {
    Atmosphere::Standard {
        station_altitude_m: 0.0,
        temperature_c: 15.0,
        pressure_hpa: 506.625,
        humidity_percent: 0.0,
    }
}

fn descending(y: f64) -> FlightState {
    FlightState {
        time_s: 3.0,
        position_m: Vec3::new(20.0, y, 0.0),
        velocity_mps: Vec3::new(1.0, -6.0, 0.5),
    }
}

fn recovery(atmosphere: Atmosphere) -> RecoveryForces<UniformGravity> {
    RecoveryForces::new(
        UniformGravity::default(),
        0.5,
        0.0,
        atmosphere,
        Vec3::zeros(),
    )
    .unwrap()
}

#[test]
fn recovery_drag_reads_the_zone_air() {
    // A zone carrying a reading is that station reading, bit for bit, and
    // differs from the station air it replaced.
    let zoned = recovery(Atmosphere::default())
        .with_atmosphere_zones(vec![LOW_ZONE])
        .unwrap();
    let station = recovery(low_station());
    let plain = recovery(Atmosphere::default());
    for y in [0.0, 150.0, 1500.0, 4000.0] {
        let state = descending(y);
        let a = zoned.acceleration(&state, 0.3).unwrap();
        assert_eq!(a, station.acceleration(&state, 0.3).unwrap(), "at y {y}");
        assert_ne!(a, plain.acceleration(&state, 0.3).unwrap(), "at y {y}");
    }
}

fn descent_speed(forces: RecoveryForces<UniformGravity>) -> (f64, f64) {
    let rocket = rockets_engine::Rocket::new(0.3, None).unwrap();
    let sim = Simulator::new(rocket, forces, IntegratorSettings::default()).unwrap();
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 300.0, 0.0),
        velocity_mps: Vec3::zeros(),
    };
    for _ in 0..120 {
        state = sim
            .step(&state, 0.05, ControlInput::along(Vec3::y()))
            .unwrap()
            .state;
    }
    (-state.velocity_mps.y, state.position_m.y)
}

#[test]
fn parachute_descent_scales_as_root_density_ratio_in_a_zone() {
    // Terminal velocity sqrt(2mg/(rho CdA)): half the density, sqrt(2) faster.
    let (plain, plain_y) = descent_speed(recovery(Atmosphere::default()));
    let (zoned, zoned_y) = descent_speed(
        recovery(Atmosphere::default())
            .with_atmosphere_zones(vec![LOW_ZONE])
            .unwrap(),
    );
    let rho0 = calculate_air_density_cimp(15.0, 1013.25, 0.0);
    let rho = calculate_air_density_cimp(LOW_ZONE.0, LOW_ZONE.1, LOW_ZONE.2);
    let expected = (rho0 / rho).sqrt();
    let ratio = zoned / plain;
    // The two descents end a few metres apart in altitude (~1e-4 in density).
    assert!(
        (ratio - expected).abs() / expected < 2e-3,
        "speed ratio {ratio} vs sqrt(rho0/rho) {expected} (y {plain_y} vs {zoned_y})"
    );
}

fn params() -> WeathercockParams {
    WeathercockParams::new(
        vec![(0.0, 2.0), (3.0, 2.0)],
        vec![(0.0, 0.6), (3.0, 0.6)],
        0.3,
        0.3,
        0.1,
        0.05,
        0.1,
    )
    .unwrap()
}

fn axis_history(mut weathercock: Weathercock) -> Vec<Vec3> {
    let state = FlightState {
        time_s: 1.0,
        position_m: Vec3::new(10.0, 800.0, 0.0),
        velocity_mps: Vec3::new(100.0, 0.0, 0.0),
    };
    (0..200)
        .map(|_| {
            weathercock
                .command(GuidanceContext {
                    state: &state,
                    rocket: RocketSample {
                        mass_kg: 1.0,
                        motor: MotorSample {
                            phase: MotorPhase::Unpowered,
                            thrust_n: 0.0,
                            propellant_mass_kg: 0.0,
                        },
                    },
                    step_duration_s: 0.01,
                    max_steering_accel_mps2: 0.0,
                })
                .unwrap()
                .thrust_direction
        })
        .collect()
}

#[test]
fn weathercock_reads_the_zone_air() {
    let start = Vec3::new(1.0, 0.1, 0.0);
    let zoned = Weathercock::new(params(), start, Atmosphere::default(), Vec3::zeros())
        .unwrap()
        .with_atmosphere_zones(vec![LOW_ZONE])
        .unwrap();
    let station = Weathercock::new(params(), start, low_station(), Vec3::zeros()).unwrap();
    let plain = Weathercock::new(params(), start, Atmosphere::default(), Vec3::zeros()).unwrap();
    let zoned = axis_history(zoned);
    assert_eq!(zoned, axis_history(station));
    assert_ne!(zoned, axis_history(plain));
}

fn lifting(atmosphere: Atmosphere) -> BallisticForces {
    BallisticForces::new(
        0.05,
        vec![(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)],
        atmosphere,
        Vec3::zeros(),
    )
    .unwrap()
    .with_body_lift(
        vec![(0.0, 2.0), (1.0, 3.0), (3.0, 4.0)],
        vec![(0.0, 0.0), (1.0, 0.5), (3.0, 1.0)],
    )
    .unwrap()
}

fn lift_increment(forces: &BallisticForces, y: f64) -> Vec3 {
    let state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(50.0, y, 0.0),
        velocity_mps: Vec3::new(200.0, 0.0, 0.0),
    };
    let alpha = 5.0_f64.to_radians();
    let axis = Vec3::new(alpha.cos(), alpha.sin(), 0.0);
    forces.acceleration_with_axis(&state, 0.3, axis).unwrap()
        - forces.acceleration(&state, 0.3).unwrap()
}

#[test]
fn body_lift_reads_the_zone_air() {
    let zoned = lifting(Atmosphere::default())
        .with_atmosphere_zones(vec![LOW_ZONE])
        .unwrap();
    let station = lifting(low_station());
    let plain = lifting(Atmosphere::default());
    for y in [0.0, 1500.0, 6000.0] {
        let lift = lift_increment(&zoned, y);
        assert_eq!(lift, lift_increment(&station, y), "at y {y}");
        assert_ne!(lift, lift_increment(&plain, y), "at y {y}");
    }
}

#[test]
fn every_force_validates_its_zones() {
    let uniform = Atmosphere::Uniform {
        density_kg_m3: 1.0,
        speed_of_sound_mps: 340.0,
    };
    assert!(recovery(Atmosphere::default())
        .with_atmosphere_zones(vec![])
        .is_err());
    assert!(recovery(uniform)
        .with_atmosphere_zones(vec![LOW_ZONE])
        .is_err());
    assert!(recovery(Atmosphere::default())
        .with_atmosphere_zones(vec![(15.0, -1.0, 0.0, 100.0)])
        .is_err());
    let weathercock =
        |atmosphere| Weathercock::new(params(), Vec3::x(), atmosphere, Vec3::zeros()).unwrap();
    assert!(weathercock(Atmosphere::default())
        .with_atmosphere_zones(vec![])
        .is_err());
    assert!(weathercock(uniform)
        .with_atmosphere_zones(vec![LOW_ZONE])
        .is_err());
    assert!(weathercock(Atmosphere::default())
        .with_atmosphere_zones(vec![(15.0, 1013.25, 101.0, 100.0)])
        .is_err());
}
