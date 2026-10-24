use rockets_engine::{
    ControlInput, FixedDirection, FlightState, ForceModel, Guidance, GuidanceContext,
    IntegratorSettings, Motor, MotorPhase, MotorSample, ProgradeDirection, RailForces, Rocket,
    RocketSample, Simulator, SteeringLag, ThrustCurve, ThrustMisalignment, ThrustPoint,
    UniformGravity, Vec3,
};

fn context(state: &FlightState) -> GuidanceContext<'_> {
    GuidanceContext {
        state,
        rocket: RocketSample {
            mass_kg: 1.0,
            motor: MotorSample {
                phase: MotorPhase::Unpowered,
                thrust_n: 0.0,
                propellant_mass_kg: 0.0,
            },
        },
        step_duration_s: 0.05,
        max_steering_accel_mps2: 10.0,
    }
}

fn state(v: Vec3) -> FlightState {
    FlightState {
        time_s: 0.0,
        position_m: Vec3::zeros(),
        velocity_mps: v,
    }
}

#[test]
fn prograde_follows_velocity_and_falls_back_at_rest() {
    let mut g = ProgradeDirection::new(Vec3::y());
    let parked = state(Vec3::zeros());
    assert_eq!(
        g.command(context(&parked)).unwrap().thrust_direction,
        Vec3::y()
    );
    let moving = state(Vec3::new(3.0, 4.0, 0.0));
    let axis = g.command(context(&moving)).unwrap().thrust_direction;
    assert!((axis - Vec3::new(0.6, 0.8, 0.0)).norm() < 1e-12);
}

#[test]
fn prograde_produces_a_gravity_turn() {
    let curve = ThrustCurve::new(vec![
        ThrustPoint {
            time_s: 0.0,
            thrust_n: 30.0,
        },
        ThrustPoint {
            time_s: 2.0,
            thrust_n: 30.0,
        },
    ])
    .unwrap();
    let motor = Motor::new(curve, 0.05, 0.0).unwrap();
    let rocket = Rocket::new(0.3, Some(motor)).unwrap();
    let sim = Simulator::new(
        rocket,
        UniformGravity::default(),
        IntegratorSettings::default(),
    )
    .unwrap();
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::zeros(),
        velocity_mps: Vec3::new(5.0, 50.0, 0.0),
    };
    let mut guidance = ProgradeDirection::new(Vec3::y());
    for _ in 0..60 {
        state = sim.step_guided(&state, 0.05, &mut guidance).unwrap().state;
    }
    let path_angle = state.velocity_mps.y.atan2(state.velocity_mps.x);
    let launch_angle = 50.0_f64.atan2(5.0);
    assert!(
        path_angle < launch_angle,
        "gravity should bend the flight path downrange: {path_angle} vs {launch_angle}"
    );
    assert!(state.position_m.x > 0.0);
}

#[test]
fn zero_misalignment_is_bit_identical() {
    let inner = FixedDirection(Vec3::new(0.0, 1.0, 0.0));
    let mut plain = inner;
    let mut aligned = ThrustMisalignment::new(inner, 0.0, 0.7).unwrap();
    let state = state(Vec3::zeros());
    let a = plain.command(context(&state)).unwrap();
    let b = aligned.command(context(&state)).unwrap();
    assert_eq!(a, b);
}

#[test]
fn misalignment_tilts_by_the_requested_angle() {
    let inner = FixedDirection(Vec3::new(1.0, 0.0, 0.0));
    let tilt = 0.02;
    let mut g = ThrustMisalignment::new(inner, tilt, 0.0).unwrap();
    let state = state(Vec3::zeros());
    let axis = g.command(context(&state)).unwrap().thrust_direction;
    let measured = axis.y.atan2(axis.x); // angle from +X in the XY plane
    assert!((measured.abs() - tilt).abs() < 1e-6, "measured {measured}");
    assert!(ThrustMisalignment::new(inner, -0.1, 0.0).is_err());
}

#[test]
fn steering_lag_relaxes_toward_the_command() {
    let target = Vec3::new(4.0, 0.0, 0.0);
    let mut lag = SteeringLag::new(
        move |_ctx: GuidanceContext<'_>| {
            Ok(ControlInput {
                thrust_direction: Vec3::y(),
                steering_acceleration_mps2: target,
            })
        },
        0.5,
    )
    .unwrap();
    let state = state(Vec3::zeros());
    // One time constant of 0.05 s steps: expect 1 - exp(-1) of the target.
    for _ in 0..10 {
        lag.command(context(&state)).unwrap();
    }
    let current = lag.current();
    let expected = 4.0 * (1.0 - (-1.0_f64).exp());
    assert!((current.x - expected).abs() < 1e-9, "current {current}");
    assert_eq!(lag.set_current(Vec3::zeros()), ());
    assert_eq!(lag.current(), Vec3::zeros());
    assert!(SteeringLag::new(FixedDirection(Vec3::y()), 0.0).is_err());
}

#[test]
fn rail_projects_acceleration_until_exit() {
    let direction = Vec3::new(0.2, 1.0, 0.0).normalize();
    let rail = RailForces::new(UniformGravity::default(), Vec3::zeros(), direction, 2.0).unwrap();
    let on_rail = state(Vec3::zeros());
    let a = rail.acceleration(&on_rail, 1.0).unwrap();
    let expected = direction * UniformGravity::default().0.dot(&direction);
    assert!((a - expected).norm() < 1e-12, "on-rail {a}");
    let mut off_rail = on_rail;
    off_rail.position_m = direction * 3.0;
    let a = rail.acceleration(&off_rail, 1.0).unwrap();
    assert_eq!(a, UniformGravity::default().0);
}

#[test]
fn powered_flight_stays_collinear_on_the_rail() {
    let direction = Vec3::new(0.2, 1.0, 0.0).normalize();
    let curve = ThrustCurve::new(vec![
        ThrustPoint {
            time_s: 0.0,
            thrust_n: 50.0,
        },
        ThrustPoint {
            time_s: 0.5,
            thrust_n: 50.0,
        },
    ])
    .unwrap();
    let motor = Motor::new(curve, 0.02, 0.0).unwrap();
    let rocket = Rocket::new(0.3, Some(motor)).unwrap();
    let rail = RailForces::new(UniformGravity::default(), Vec3::zeros(), direction, 2.0).unwrap();
    let sim = Simulator::new(rocket, rail, IntegratorSettings::default()).unwrap();
    let mut state = FlightState {
        time_s: 0.0,
        position_m: Vec3::zeros(),
        velocity_mps: Vec3::zeros(),
    };
    for _ in 0..10 {
        state = sim
            .step(&state, 0.05, ControlInput::along(direction))
            .unwrap()
            .state;
        let along = state.position_m.dot(&direction);
        if along < 2.0 {
            // Cross product with the rail direction must vanish on the rail.
            assert!(state.position_m.cross(&direction).norm() < 1e-9);
            assert!(state.velocity_mps.cross(&direction).norm() < 1e-9);
        }
    }
    assert!(
        state.position_m.dot(&direction) > 2.0,
        "should have exited the rail"
    );
    // After exit, gravity is free to pull the trajectory off the rail axis.
    assert!(state.position_m.cross(&direction).norm() > 0.0);
}
