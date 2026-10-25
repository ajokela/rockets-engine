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
fn flattened_single_line_rasp_still_parses() {
    // Single-line inputs (e.g. browser fields) strip newlines; leftover header
    // tokens must be treated as thrust samples, not silently dropped.
    // (A leading ';' comment would swallow a flattened line, so use a
    // comment-free file, matching the terminal's example button.)
    let flat = "Estes D12 24 70 0 0.01 0.03 0 0 0.1 10 1 0";
    let file = MotorFile::parse(flat).unwrap();
    assert_eq!(file.curve.points().len(), 3);
    assert_eq!(file.propellant_mass_kg, 0.01);
    assert_eq!(file.motor(0.0).unwrap().burnout_time_s(), 1.0);
}

#[test]
fn registry_motors_parse_and_match_their_impulse_class() {
    let cases = [
        ("estes:a8", 1.26, 2.5),
        ("estes:b6", 2.5, 5.0),
        ("estes:c6", 5.0, 10.0),
        ("estes:d12", 10.0, 20.0),
        ("aerotech:g80", 80.0, 160.0),
    ];
    for (name, lo, hi) in cases {
        let file = rockets_engine::model_rocket::motor_by_name(name)
            .unwrap_or_else(|| panic!("{name} should be registered"));
        let impulse = file.curve.total_impulse_ns();
        assert!(
            impulse > lo && impulse <= hi,
            "{name}: impulse {impulse} outside ({lo}, {hi}]"
        );
        assert!(file.propellant_mass_kg > 0.0);
        assert!(file.total_mass_kg > file.propellant_mass_kg);
        assert!(file.curve.points().len() >= 2);
    }
    assert!(rockets_engine::model_rocket::motor_by_name("estes:x9").is_none());
}

#[test]
fn canonical_thrustcurve_layout_parses() {
    // Real RASP files put the designation first and the manufacturer last.
    let file =
        MotorFile::parse("D12 24 70 0-3-5-7 0.0211 0.0426 Estes\n0.049 2.569\n1.65 0").unwrap();
    assert_eq!(file.designation, "D12");
    assert_eq!(file.manufacturer, "Estes");
    assert_eq!(file.delays, "0-3-5-7");
    assert!((file.propellant_mass_kg - 0.0211).abs() < 1e-12);
    // The origin is prepended when the curve starts after t = 0.
    assert_eq!(file.curve.points()[0].time_s, 0.0);
    assert_eq!(file.curve.points().len(), 3);
    // Six tokens: canonical without the manufacturer field.
    let file = MotorFile::parse("A8 18 70 0 0.004 0.014\n0.016 1.08\n0.534 0").unwrap();
    assert_eq!(file.manufacturer, "");
    assert_eq!(file.designation, "A8");
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
fn recovery_does_not_deploy_on_the_pad() {
    // v = 0 at t = 0 is not a descent: the inner model passes through even
    // with a strong wind, which would otherwise kick the "deployed" chute
    // sideways off the pad.
    let recovery = RecoveryForces::new(
        UniformGravity::default(),
        0.5,
        0.0,
        Atmosphere::default(),
        Vec3::new(-5.0, 0.0, 0.0),
    )
    .unwrap();
    let state = FlightState {
        time_s: 0.0,
        position_m: Vec3::zeros(),
        velocity_mps: Vec3::zeros(),
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
fn dual_deployment_swaps_to_the_main_below_its_altitude() {
    // Drogue 0.25 m² above 150 m, main 1.0 m² below: the descent settles at
    // each parachute's analytic terminal velocity in turn.
    let mass = 0.5;
    let rocket = rockets_engine::Rocket::new(mass, None).unwrap();
    let recovery = RecoveryForces::new(
        UniformGravity::default(),
        0.25,
        0.0,
        Atmosphere::default(),
        Vec3::zeros(),
    )
    .unwrap()
    .with_main(1.0, 150.0)
    .unwrap();
    let sim = Simulator::new(rocket, recovery, IntegratorSettings::default()).unwrap();
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 250.0, 0.0),
        velocity_mps: Vec3::zeros(),
    };
    // 15 s: settled on the drogue, still above the swap altitude.
    for _ in 0..300 {
        state = sim
            .step(&state, 0.05, ControlInput::along(Vec3::y()))
            .unwrap()
            .state;
    }
    assert!(
        state.position_m.y > 150.0,
        "altitude {}",
        state.position_m.y
    );
    let drogue_terminal = (2.0 * mass * 9.80665 / (1.225 * 0.25)).sqrt();
    let measured = -state.velocity_mps.y;
    assert!(
        (measured - drogue_terminal).abs() / drogue_terminal < 0.03,
        "drogue descent {measured} vs {drogue_terminal}"
    );
    // 45 more seconds: below 150 m, settled on the main.
    for _ in 0..900 {
        state = sim
            .step(&state, 0.05, ControlInput::along(Vec3::y()))
            .unwrap()
            .state;
    }
    assert!(
        state.position_m.y < 150.0,
        "altitude {}",
        state.position_m.y
    );
    let main_terminal = (2.0 * mass * 9.80665 / (1.225 * 1.0)).sqrt();
    let measured = -state.velocity_mps.y;
    assert!(
        (measured - main_terminal).abs() / main_terminal < 0.03,
        "main descent {measured} vs {main_terminal}"
    );
}

#[test]
fn main_parachute_validates_its_numbers() {
    let recovery = RecoveryForces::new(
        UniformGravity::default(),
        0.25,
        0.0,
        Atmosphere::default(),
        Vec3::zeros(),
    )
    .unwrap();
    assert!(recovery.clone().with_main(-1.0, 150.0).is_err());
    assert!(recovery.with_main(0.5, f64::NAN).is_err());
}

#[test]
fn recovery_turbulence_is_deterministic_per_seed() {
    let build = |seed| {
        RecoveryForces::new(
            UniformGravity::default(),
            0.25,
            0.0,
            Atmosphere::default(),
            Vec3::zeros(),
        )
        .unwrap()
        .with_turbulence(2.0, 2.0, seed)
        .unwrap()
    };
    let state = FlightState {
        time_s: 3.3,
        position_m: Vec3::new(0.0, 200.0, 0.0),
        velocity_mps: Vec3::new(0.0, -5.0, 0.0),
    };
    let a = build(42).acceleration(&state, 0.5).unwrap();
    assert_eq!(a, build(42).acceleration(&state, 0.5).unwrap());
    assert_ne!(a, build(7).acceleration(&state, 0.5).unwrap());
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
