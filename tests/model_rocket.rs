#![cfg(all(feature = "model", feature = "ballistics"))]

use rockets_engine::{
    ballistics::Atmosphere, Airframe, ControlInput, FlightState, ForceModel, IntegratorSettings,
    MotorFile, RecoveryForces, Simulator, UniformGravity, Vec3,
};

const RASP: &str = "; AeroTech C6 comment line\nAeroTech C6 24 70 0 0.0045 0.0264\n   0.000 0.000\n   0.031 5.471\n   0.063 4.102\n   1.000 0.000\n";

#[test]
fn parses_rasp_header_and_curve() {
    let file = MotorFile::parse(RASP).unwrap();
    assert_eq!(file.manufacturer, "AeroTech");
    assert_eq!(file.designation, "C6");
    assert_eq!(file.diameter_mm, 24.0);
    assert_eq!(file.length_mm, 70.0);
    assert_eq!(file.delays, "0");
    assert_eq!(file.propellant_mass_kg, 0.0045);
    assert_eq!(file.total_mass_kg, 0.0264);
    assert_eq!(file.curve.points().len(), 4);
    let motor = file.motor(0.0).unwrap();
    assert_eq!(motor.burnout_time_s(), 1.0);
    assert!(motor.curve().total_impulse_ns() > 0.0);
}

#[test]
fn parser_rejects_bad_input() {
    assert!(MotorFile::parse("").is_err());
    assert!(MotorFile::parse("Estes D12 24 70 0\n").is_err());
    assert!(MotorFile::parse("Estes D12 24 70 0 0.01 0.03\n0.0 0.0\n").is_err());
    assert!(MotorFile::parse("Estes D12 24 70 0 0.01 0.03\n0.0 0.0\n1.0 banana\n").is_err());
}

#[test]
fn recovery_passthrough_while_ascending() {
    let recovery = RecoveryForces::new(
        UniformGravity::default(),
        0.5,
        0.0,
        Atmosphere::default(),
        Vec3::zeros(),
    )
    .unwrap();
    let state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 100.0, 0.0),
        velocity_mps: Vec3::new(0.0, 10.0, 0.0),
    };
    let a = recovery.acceleration(&state, 0.3).unwrap();
    assert_eq!(a, UniformGravity::default().0);
}

#[test]
fn descent_reaches_the_analytic_terminal_velocity() {
    let cd_area = 0.5;
    let mass = 0.3;
    let recovery = RecoveryForces::new(
        UniformGravity::default(),
        cd_area,
        0.0,
        Atmosphere::default(),
        Vec3::zeros(),
    )
    .unwrap();
    let rocket = rockets_engine::Rocket::new(mass, None).unwrap();
    let sim = Simulator::new(rocket, recovery, IntegratorSettings::default()).unwrap();
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 200.0, 0.0),
        velocity_mps: Vec3::zeros(),
    };
    for _ in 0..120 {
        state = sim
            .step(&state, 0.05, ControlInput::along(Vec3::y()))
            .unwrap()
            .state;
    }
    let g = 9.80665;
    let rho = 1.225;
    let terminal = (2.0 * mass * g / (rho * cd_area)).sqrt();
    let measured = -state.velocity_mps.y;
    assert!(
        (measured - terminal).abs() / terminal < 0.02,
        "descent speed {measured} vs terminal {terminal}"
    );
    assert!(state.velocity_mps.y < 0.0);
}

#[test]
fn deployment_waits_for_the_time_gate() {
    let recovery = RecoveryForces::new(
        UniformGravity::default(),
        0.5,
        10.0,
        Atmosphere::default(),
        Vec3::zeros(),
    )
    .unwrap();
    let early = FlightState {
        time_s: 2.0,
        position_m: Vec3::zeros(),
        velocity_mps: Vec3::new(0.0, -5.0, 0.0),
    };
    assert_eq!(
        recovery.acceleration(&early, 0.3).unwrap(),
        UniformGravity::default().0
    );
    let late = FlightState {
        time_s: 12.0,
        ..early
    };
    let a = recovery.acceleration(&late, 0.3).unwrap();
    // Recovery drag points up, opposing the downward motion.
    assert!(a.y > UniformGravity::default().0.y);
}

#[test]
fn airframe_presets_build_forces_and_params() {
    for af in [Airframe::sport_24mm(), Airframe::high_power_54mm()] {
        af.forces(Vec3::zeros()).unwrap();
        af.weathercock_params().unwrap();
    }
    assert!(Airframe::by_name("sport24").is_some());
    assert!(Airframe::by_name("hp54").is_some());
    assert!(Airframe::by_name("saturn-v").is_none());
}
