use rockets_engine::*;

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual {actual:.16e}, expected {expected:.16e}, tolerance {tolerance:.3e}"
    );
}

fn curve(points: &[(f64, f64)]) -> ThrustCurve {
    ThrustCurve::new(
        points
            .iter()
            .map(|&(time_s, thrust_n)| ThrustPoint { time_s, thrust_n })
            .collect(),
    )
    .unwrap()
}

fn constant_motor(ignition: f64) -> Motor {
    Motor::new(curve(&[(0.0, 10.0), (2.0, 10.0)]), 1.0, ignition).unwrap()
}

fn origin() -> FlightState {
    FlightState {
        time_s: 0.0,
        position_m: Vec3::zeros(),
        velocity_mps: Vec3::zeros(),
    }
}

fn vacuum(rocket: Rocket, max_step_s: f64) -> Simulator<UniformGravity> {
    Simulator::new(
        rocket,
        UniformGravity(Vec3::zeros()),
        IntegratorSettings {
            max_step_s,
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn piecewise_curve_integrates_impulse_and_consumes_fuel_proportionally() {
    let curve = curve(&[(0.0, 0.0), (1.0, 10.0), (2.0, 0.0)]);
    close(curve.total_impulse_ns(), 10.0, 0.0);
    close(curve.thrust_at(0.5).unwrap(), 5.0, 0.0);
    close(curve.impulse_at(0.5).unwrap(), 1.25, 0.0);
    close(curve.impulse_at(1.5).unwrap(), 8.75, 0.0);
    assert_eq!(curve.impulse_at(-1.0).unwrap(), 0.0);
    assert_eq!(curve.impulse_at(100.0).unwrap(), 10.0);
    assert_eq!(curve.thrust_at(100.0).unwrap(), 0.0);
    let motor = Motor::new(curve, 0.2, 0.5).unwrap();
    assert_eq!(
        motor.sample(0.0).unwrap().phase,
        MotorPhase::AwaitingIgnition
    );
    close(motor.sample(1.0).unwrap().propellant_mass_kg, 0.175, 1e-15);
    close(motor.sample(1.5).unwrap().propellant_mass_kg, 0.1, 1e-15);
    assert_eq!(motor.sample(2.5).unwrap().phase, MotorPhase::BurnedOut);
    for i in 0..10_000 {
        let sample = motor.sample(i as f64 * 0.001).unwrap();
        assert!((0.0..=0.2).contains(&sample.propellant_mass_kg));
    }
}

#[test]
fn zero_thrust_section_does_not_burn_propellant() {
    let motor = Motor::new(
        curve(&[(0.0, 10.0), (1.0, 0.0), (2.0, 0.0), (3.0, 10.0)]),
        0.1,
        0.0,
    )
    .unwrap();
    let a = motor.sample(1.0).unwrap();
    let b = motor.sample(2.0).unwrap();
    assert_eq!(a.propellant_mass_kg, b.propellant_mass_kg);
    assert_eq!(motor.sample(1.5).unwrap().phase, MotorPhase::Burning);
    assert_eq!(a.thrust_n, 0.0);
}

#[test]
fn vacuum_burn_matches_ideal_rocket_equation_and_position() {
    let sim = vacuum(Rocket::new(1.0, Some(constant_motor(0.0))).unwrap(), 0.005);
    let result = sim
        .step(&origin(), 2.0, ControlInput::along(Vec3::x()))
        .unwrap();
    // F = 10 N, mdot = 0.5 kg/s, ve = 20 m/s, m0 = 2 kg, mf = 1 kg.
    let velocity = 20.0 * 2.0_f64.ln();
    let distance = 40.0 * (1.0 - 2.0_f64.ln());
    close(result.state.velocity_mps.x, velocity, 2e-11);
    close(result.state.position_m.x, distance, 2e-10);
    assert_eq!(result.rocket.mass_kg, 1.0);
    assert_eq!(result.rocket.motor.thrust_n, 0.0);
    assert_eq!(result.rocket.motor.phase, MotorPhase::BurnedOut);
}

#[test]
fn delayed_ignition_and_burnout_preserve_coast_and_impulse() {
    let sim = vacuum(Rocket::new(1.0, Some(constant_motor(0.7))).unwrap(), 0.003);
    let pre = sim.step(&origin(), 0.7, ControlInput::default()).unwrap();
    assert_eq!(pre.state.position_m, Vec3::zeros());
    assert_eq!(pre.state.velocity_mps, Vec3::zeros());
    assert_eq!(pre.rocket.motor.phase, MotorPhase::Burning);
    let burned = sim.step(&pre.state, 2.0, ControlInput::default()).unwrap();
    let coast = sim
        .step(&burned.state, 1.0, ControlInput::default())
        .unwrap();
    close(coast.state.velocity_mps.x, 20.0 * 2.0_f64.ln(), 1e-10);
    close(
        coast.state.position_m.x,
        burned.state.position_m.x + burned.state.velocity_mps.x,
        1e-11,
    );
    let continuous = sim.step(&origin(), 3.7, ControlInput::default()).unwrap();
    close(
        (continuous.state.position_m - coast.state.position_m).norm(),
        0.0,
        1e-10,
    );
    assert_eq!(coast.rocket.motor.propellant_mass_kg, 0.0);
}

#[test]
fn short_burn_is_not_skipped_by_a_large_step_and_knots_are_split() {
    let motor = Motor::new(curve(&[(0.0, 0.0), (0.01, 1.0), (0.02, 0.0)]), 0.001, 0.23).unwrap();
    let rocket = Rocket::new(1.0, Some(motor)).unwrap();
    let coarse = vacuum(rocket.clone(), 2.0)
        .step(&origin(), 1.0, ControlInput::default())
        .unwrap();
    let fine = vacuum(rocket, 0.0005)
        .step(&origin(), 1.0, ControlInput::default())
        .unwrap();
    assert_eq!(coarse.substeps, 4); // Coast, ramp up, ramp down, coast.
    close(
        coarse.state.velocity_mps.x,
        fine.state.velocity_mps.x,
        2e-10,
    );
    close(coarse.state.position_m.x, fine.state.position_m.x, 1e-9);
    assert_eq!(coarse.rocket.motor.phase, MotorPhase::BurnedOut);
}

#[test]
fn rk4_converges_at_fourth_order_for_variable_mass() {
    let expected = 20.0 * 2.0_f64.ln();
    let error = |dt| {
        let sim = vacuum(Rocket::new(1.0, Some(constant_motor(0.0))).unwrap(), dt);
        (sim.step(&origin(), 2.0, ControlInput::default())
            .unwrap()
            .state
            .velocity_mps
            .x
            - expected)
            .abs()
    };
    let coarse = error(0.25);
    let medium = error(0.125);
    let fine = error(0.0625);
    assert!(coarse / medium > 14.0, "{coarse} / {medium}");
    assert!(medium / fine > 14.0, "{medium} / {fine}");
}

#[test]
fn no_motor_matches_analytic_gravity_in_three_dimensions() {
    let gravity = Vec3::new(0.0, -9.80665, 0.0);
    let sim = Simulator::new(
        Rocket::new(3.0, None).unwrap(),
        UniformGravity(gravity),
        Default::default(),
    )
    .unwrap();
    let state = FlightState {
        position_m: Vec3::new(3.0, 100.0, -2.0),
        velocity_mps: Vec3::new(20.0, 4.0, 3.0),
        ..origin()
    };
    let result = sim.step(&state, 2.0, ControlInput::default()).unwrap();
    close(
        (result.state.position_m - (state.position_m + state.velocity_mps * 2.0 + gravity * 2.0))
            .norm(),
        0.0,
        1e-11,
    );
    close(
        (result.state.velocity_mps - (state.velocity_mps + gravity * 2.0)).norm(),
        0.0,
        1e-11,
    );
    assert_eq!(result.rocket.mass_kg, 3.0);
    assert_eq!(result.rocket.motor.phase, MotorPhase::Unpowered);
}

#[test]
fn thrust_direction_is_normalized_and_works_from_rest() {
    let sim = vacuum(Rocket::new(1.0, Some(constant_motor(0.0))).unwrap(), 0.01);
    let result = sim
        .step(
            &origin(),
            2.0,
            ControlInput::along(Vec3::new(0.0, 3.0, 4.0)),
        )
        .unwrap();
    let speed = 20.0 * 2.0_f64.ln();
    close(result.state.velocity_mps.y, speed * 0.6, 1e-9);
    close(result.state.velocity_mps.z, speed * 0.8, 1e-9);
    assert_eq!(result.state.velocity_mps.x, 0.0);
    for scale in [1e-300, 1e300] {
        let scaled = sim
            .step(&origin(), 0.1, ControlInput::along(Vec3::x() * scale))
            .unwrap();
        assert!(scaled.state.velocity_mps.x > 0.0);
        assert_eq!(scaled.state.velocity_mps.y, 0.0);
    }
}

#[test]
fn guidance_runs_once_per_public_step_and_steering_is_bounded() {
    let sim = Simulator::new(
        Rocket::new(1.0, None).unwrap(),
        UniformGravity(Vec3::zeros()),
        IntegratorSettings {
            max_step_s: 0.01,
            max_steering_accel_mps2: 2.0,
            ..Default::default()
        },
    )
    .unwrap();
    let mut calls = 0;
    let mut guidance = |ctx: GuidanceContext<'_>| {
        calls += 1;
        assert_eq!(ctx.state.time_s, 0.0);
        assert_eq!(ctx.step_duration_s, 1.0);
        assert_eq!(ctx.rocket.motor.phase, MotorPhase::Unpowered);
        Ok(ControlInput {
            thrust_direction: Vec3::x(),
            steering_acceleration_mps2: Vec3::new(0.0, 0.0, 1e300),
        })
    };
    let result = sim.step_guided(&origin(), 1.0, &mut guidance).unwrap();
    assert_eq!(calls, 1);
    assert!(result.substeps >= 100);
    close(result.state.velocity_mps.z, 2.0, 1e-13);
    close(result.state.position_m.z, 1.0, 1e-13);
    let disabled = vacuum(Rocket::new(1.0, None).unwrap(), 0.1);
    let result = disabled
        .step(
            &origin(),
            1.0,
            ControlInput {
                thrust_direction: Vec3::x(),
                steering_acceleration_mps2: Vec3::repeat(100.0),
            },
        )
        .unwrap();
    assert_eq!(result.state.velocity_mps, Vec3::zeros());
}

#[test]
fn force_hook_receives_stage_mass_time_and_intermediate_state() {
    use std::cell::RefCell;
    let stages = RefCell::new(Vec::new());
    let forces = |state: &FlightState, mass: f64| {
        stages.borrow_mut().push((*state, mass));
        Ok(Vec3::zeros())
    };
    let sim = Simulator::new(
        Rocket::new(1.0, Some(constant_motor(0.0))).unwrap(),
        forces,
        IntegratorSettings {
            max_step_s: 0.2,
            ..Default::default()
        },
    )
    .unwrap();
    sim.step(&origin(), 0.2, ControlInput::default()).unwrap();
    let values = stages.borrow();
    assert_eq!(values.len(), 4);
    assert_eq!(values[0].0.time_s, 0.0);
    assert_eq!(values[1].0.time_s, 0.1);
    assert_eq!(values[3].0.time_s, 0.2);
    assert_eq!(values[0].1, 2.0);
    assert_eq!(values[3].1, 1.9);
    assert!(values[2].0.position_m.x > 0.0);
}

#[test]
fn rejects_malformed_motor_data_and_nonrepresentable_schedule() {
    for points in [
        vec![],
        vec![(0.0, 1.0)],
        vec![(1.0, 1.0), (2.0, 2.0)],
        vec![(0.0, 1.0), (0.0, 2.0)],
        vec![(0.0, -1.0), (1.0, 1.0)],
        vec![(0.0, 0.0), (1.0, 0.0)],
        vec![(0.0, 1.0), (f64::NAN, 1.0)],
        vec![(0.0, f64::INFINITY), (1.0, 1.0)],
    ] {
        assert!(ThrustCurve::new(
            points
                .into_iter()
                .map(|(time_s, thrust_n)| ThrustPoint { time_s, thrust_n })
                .collect()
        )
        .is_err());
    }
    assert!(Motor::new(curve(&[(0.0, 1.0), (1.0, 1.0)]), 0.0, 0.0).is_err());
    assert!(Motor::new(curve(&[(0.0, 1.0), (1.0, 1.0)]), 1.0, 1e30).is_err());
    assert!(Motor::new(curve(&[(0.0, 1.0), (1.0, 1.0)]), 1.0, -1.0).is_err());
    assert!(Rocket::new(0.0, None).is_err());
    assert!(Rocket::new(f64::INFINITY, None).is_err());
    assert!(constant_motor(0.0).sample(f64::NAN).is_err());
}

#[test]
fn invalid_state_step_and_controls_fail_without_mutating_state() {
    let sim = vacuum(Rocket::new(1.0, None).unwrap(), 0.1);
    let state = origin();
    for duration in [0.0, -0.1, f64::NAN, f64::INFINITY] {
        assert!(sim.step(&state, duration, ControlInput::default()).is_err());
    }
    for direction in [
        Vec3::zeros(),
        Vec3::repeat(f64::NAN),
        Vec3::repeat(f64::INFINITY),
    ] {
        assert!(sim
            .step(&state, 0.1, ControlInput::along(direction))
            .is_err());
    }
    let invalid = FlightState {
        position_m: Vec3::repeat(f64::NAN),
        ..state
    };
    assert!(sim.step(&invalid, 0.1, ControlInput::default()).is_err());
    let invalid = FlightState {
        time_s: 1e30,
        ..state
    };
    assert!(sim.step(&invalid, 0.1, ControlInput::default()).is_err());
    let mut calls = 0;
    let mut guidance = |_: GuidanceContext<'_>| {
        calls += 1;
        Ok(ControlInput::default())
    };
    assert!(sim.step_guided(&state, -1.0, &mut guidance).is_err());
    assert_eq!(calls, 0);
    assert_eq!(state, origin());
}

#[test]
fn work_budget_and_nonfinite_force_output_return_errors() {
    let rocket = Rocket::new(1.0, None).unwrap();
    for settings in [
        IntegratorSettings {
            max_step_s: 0.0,
            ..Default::default()
        },
        IntegratorSettings {
            max_substeps: 0,
            ..Default::default()
        },
        IntegratorSettings {
            max_steering_accel_mps2: f64::NAN,
            ..Default::default()
        },
    ] {
        assert!(Simulator::new(rocket.clone(), UniformGravity::default(), settings).is_err());
    }
    let sim = Simulator::new(
        rocket.clone(),
        UniformGravity::default(),
        IntegratorSettings {
            max_substeps: 2,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(
        sim.step(&origin(), 1.0, ControlInput::default()),
        Err(Error::StepLimitExceeded { limit: 2 })
    );
    let sim = Simulator::new(
        rocket,
        UniformGravity(Vec3::repeat(f64::NAN)),
        Default::default(),
    )
    .unwrap();
    assert!(matches!(
        sim.step(&origin(), 0.1, ControlInput::default()),
        Err(Error::NumericalFailure(_))
    ));
}

#[test]
fn model_and_guidance_errors_propagate() {
    let forces = |_: &FlightState, _: f64| Err(Error::Model("weather unavailable".into()));
    let sim = Simulator::new(Rocket::new(1.0, None).unwrap(), forces, Default::default()).unwrap();
    assert_eq!(
        sim.step(&origin(), 0.1, ControlInput::default()),
        Err(Error::Model("weather unavailable".into()))
    );
    let mut guide = |_: GuidanceContext<'_>| Err(Error::Model("controller unavailable".into()));
    assert_eq!(
        sim.step_guided(&origin(), 0.1, &mut guide),
        Err(Error::Model("controller unavailable".into()))
    );
}

#[test]
fn exact_step_budget_is_not_exceeded_by_floating_point_drift() {
    for (dt, count, duration) in [(0.1, 10, 1.0), (0.01, 7, 0.07), (0.03, 10, 0.3)] {
        let sim = Simulator::new(
            Rocket::new(1.0, None).unwrap(),
            UniformGravity::default(),
            IntegratorSettings {
                max_step_s: dt,
                max_substeps: count,
                ..Default::default()
            },
        )
        .unwrap();
        let result = sim
            .step(&origin(), duration, ControlInput::default())
            .unwrap();
        assert_eq!(result.substeps, count);
        assert_eq!(result.state.time_s, duration);
    }
}

#[cfg(feature = "serde")]
#[test]
fn snapshot_roundtrip_resumes_identical_burn_and_coast() {
    let sim = vacuum(Rocket::new(1.0, Some(constant_motor(0.25))).unwrap(), 0.01);
    let checkpoint = sim
        .step(&origin(), 0.8, ControlInput::default())
        .unwrap()
        .state;
    let json = serde_json::to_string(&checkpoint).unwrap();
    let restored: FlightState = serde_json::from_str(&json).unwrap();
    assert_eq!(restored, checkpoint);
    let a = sim.step(&checkpoint, 3.0, ControlInput::default()).unwrap();
    let b = sim.step(&restored, 3.0, ControlInput::default()).unwrap();
    assert_eq!(a, b);
    assert_eq!(a.rocket.motor.phase, MotorPhase::BurnedOut);
}
