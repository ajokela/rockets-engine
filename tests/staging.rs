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

fn constant_motor(thrust_n: f64, duration_s: f64, propellant_kg: f64, ignition_s: f64) -> Motor {
    Motor::new(
        curve(&[(0.0, thrust_n), (duration_s, thrust_n)]),
        propellant_kg,
        ignition_s,
    )
    .unwrap()
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

/// Base 5 kg; stage 1: 1 kg dry, 0.5 kg propellant, burns [0, 1], separates at 1.5;
/// stage 2: 0.5 kg dry, 0.2 kg propellant, burns [1.5, 2.5], never separates.
fn two_stage() -> Rocket {
    Rocket::staged(
        5.0,
        vec![
            Stage::new(1.0, Some(constant_motor(20.0, 1.0, 0.5, 0.0)), Some(1.5)).unwrap(),
            Stage::new(0.5, Some(constant_motor(10.0, 1.0, 0.2, 1.5)), None).unwrap(),
        ],
    )
    .unwrap()
}

#[test]
fn mass_combines_base_stage_dry_and_remaining_propellant() {
    let rocket = two_stage();
    close(rocket.sample(0.0).unwrap().mass_kg, 7.2, 1e-15);
    // Stage 1 half burned at t = 0.5.
    close(rocket.sample(0.5).unwrap().mass_kg, 7.2 - 0.25, 1e-12);
    // Stage 1 spent but still attached just before separation.
    close(rocket.sample(1.5 - 1e-12).unwrap().mass_kg, 6.7, 1e-9);
}

#[test]
fn separation_drops_exactly_the_stage_dry_mass() {
    let rocket = two_stage();
    let before = rocket.sample(1.5 - 1e-12).unwrap().mass_kg;
    let after = rocket.sample(1.5).unwrap().mass_kg;
    close(before - after, 1.0, 1e-9);
    close(after, 5.7, 1e-12);
    // All propellant spent and stage 1 gone: only base + stage 2 dry remain.
    close(rocket.sample(3.0).unwrap().mass_kg, 5.5, 1e-12);
}

#[test]
fn inter_stage_coast_reports_awaiting_ignition_with_zero_thrust() {
    let rocket = two_stage();
    let sample = rocket.sample(1.2).unwrap();
    assert_eq!(sample.motor.phase, MotorPhase::AwaitingIgnition);
    assert_eq!(sample.motor.thrust_n, 0.0);
    assert_eq!(rocket.sample(0.5).unwrap().motor.phase, MotorPhase::Burning);
    assert_eq!(
        rocket.sample(2.5).unwrap().motor.phase,
        MotorPhase::BurnedOut
    );
}

#[test]
fn hot_staging_sums_thrust_during_overlap() {
    // Stage 2 ignites at 0.9 while stage 1 still burns; stage 1 separates at 1.0.
    let rocket = Rocket::staged(
        2.0,
        vec![
            Stage::new(0.5, Some(constant_motor(30.0, 1.0, 0.3, 0.0)), Some(1.0)).unwrap(),
            Stage::new(0.2, Some(constant_motor(12.0, 1.0, 0.2, 0.9)), None).unwrap(),
        ],
    )
    .unwrap();
    close(rocket.sample(0.95).unwrap().motor.thrust_n, 42.0, 1e-12);
    close(rocket.sample(0.85).unwrap().motor.thrust_n, 30.0, 1e-12);
    close(rocket.sample(1.0).unwrap().motor.thrust_n, 12.0, 1e-12);
}

#[test]
fn drop_tank_without_motor_only_changes_mass() {
    let rocket = Rocket::staged(3.0, vec![Stage::new(0.3, None, Some(2.0)).unwrap()]).unwrap();
    assert_eq!(
        rocket.sample(1.0).unwrap().motor.phase,
        MotorPhase::Unpowered
    );
    close(rocket.sample(1.999).unwrap().mass_kg, 3.3, 1e-12);
    close(rocket.sample(2.0).unwrap().mass_kg, 3.0, 1e-12);
}

#[test]
fn separating_an_unlit_stage_removes_its_unspent_propellant() {
    // The motor would ignite at t = 10, but the stage separates at t = 5.
    let rocket = Rocket::staged(
        2.0,
        vec![Stage::new(0.4, Some(constant_motor(10.0, 1.0, 0.6, 10.0)), Some(5.0)).unwrap()],
    )
    .unwrap();
    close(rocket.sample(4.0).unwrap().mass_kg, 3.0, 1e-12);
    close(rocket.sample(5.0).unwrap().mass_kg, 2.0, 1e-12);
    assert_eq!(
        rocket.sample(11.0).unwrap().motor.phase,
        MotorPhase::Unpowered
    );
    assert_eq!(rocket.sample(11.0).unwrap().motor.thrust_n, 0.0);
}

#[test]
fn separation_splits_integration_and_mass_changes_midstep() {
    let rocket = two_stage();
    let sim = vacuum(rocket, 0.4); // max_step does not land on the 1.5 s separation
    let control = ControlInput::along(Vec3::x());
    let result = sim.step(&origin(), 1.6, control).unwrap();
    close(result.rocket.mass_kg, 5.5 + 0.2 * 0.9, 1e-9); // stage 2 ten percent burned
    assert!(result.substeps >= 5); // 4 grid steps + the forced split at 1.5
}

#[test]
fn two_burn_delta_v_matches_tsiolkovsky() {
    // Identical effective exhaust velocity for both stages: ve = impulse / propellant.
    let ve = 100.0;
    let s1_impulse = 40.0;
    let s2_impulse = 20.0;
    let base = 1.0;
    let rocket = Rocket::staged(
        base,
        vec![
            Stage::new(0.4, Some(constant_motor(40.0, 1.0, 0.4, 0.0)), Some(1.0)).unwrap(),
            Stage::new(0.2, Some(constant_motor(20.0, 1.0, 0.2, 1.0)), None).unwrap(),
        ],
    )
    .unwrap();
    let m0 = base + 0.4 + 0.4 + 0.2 + 0.2;
    let m_after_burn1 = m0 - 0.4;
    let m_after_sep1 = m_after_burn1 - 0.4;
    let m_after_burn2 = m_after_sep1 - 0.2;
    let dv = ve * (m0 / m_after_burn1).ln() + ve * (m_after_sep1 / m_after_burn2).ln();
    let _ = (s1_impulse, s2_impulse);

    let sim = vacuum(rocket, 0.005);
    let control = ControlInput::along(Vec3::x());
    let first = sim.step(&origin(), 1.0, control).unwrap();
    close(first.rocket.mass_kg, m_after_sep1, 1e-12);
    let second = sim.step(&first.state, 1.0, control).unwrap();
    close(second.state.velocity_mps.x, dv, 1e-6);
    close(second.rocket.mass_kg, m_after_burn2, 1e-12);
    assert_eq!(second.rocket.motor.phase, MotorPhase::BurnedOut);
}

#[test]
fn parallel_boosters_and_sustainer_burn_together() {
    // Twin strap-on boosters flank a sustainer; boosters separate at burnout.
    let booster = || Stage::new(0.3, Some(constant_motor(50.0, 1.0, 0.5, 0.0)), Some(1.0)).unwrap();
    let sustainer = Stage::new(0.4, Some(constant_motor(10.0, 4.0, 0.8, 0.0)), None).unwrap();
    let rocket = Rocket::staged(1.0, vec![booster(), booster(), sustainer]).unwrap();
    close(rocket.sample(0.5).unwrap().motor.thrust_n, 110.0, 1e-12);
    close(
        rocket.sample(0.0).unwrap().mass_kg,
        1.0 + 2.0 * 0.8 + 1.2,
        1e-12,
    );
    close(rocket.sample(1.0).unwrap().motor.thrust_n, 10.0, 1e-12);
    close(rocket.sample(1.0).unwrap().mass_kg, 1.0 + 0.4 + 0.6, 1e-12);
}

#[test]
fn single_motor_rocket_keeps_single_motor_shape() {
    let motor = constant_motor(10.0, 2.0, 1.0, 0.0);
    let rocket = Rocket::new(1.0, Some(motor)).unwrap();
    assert!(rocket.motor().is_some());
    assert_eq!(rocket.stages().len(), 1);
    assert_eq!(rocket.sample(0.5).unwrap().motor.phase, MotorPhase::Burning);
    // A staged vehicle has no single motor to report.
    assert!(two_stage().motor().is_none());
}

#[test]
fn invalid_staging_is_rejected() {
    assert!(Stage::new(0.0, None, None).is_err());
    assert!(Stage::new(-1.0, None, Some(1.0)).is_err());
    assert!(Stage::new(f64::NAN, None, Some(1.0)).is_err());
    assert!(Stage::new(0.5, None, Some(-0.5)).is_err());
    assert!(Stage::new(0.5, None, Some(f64::NAN)).is_err());
    assert!(Rocket::staged(0.0, vec![]).is_err());
    assert!(Rocket::staged(f64::INFINITY, vec![]).is_err());
}
