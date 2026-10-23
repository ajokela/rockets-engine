use rockets_engine::*;

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual {actual:.16e}, expected {expected:.16e}, tolerance {tolerance:.3e}"
    );
}

const R_EARTH: f64 = 6_371_000.0;
const MU: f64 = EARTH_MU_M3_S2;

fn state(time_s: f64, position_m: Vec3, velocity_mps: Vec3) -> FlightState {
    FlightState {
        time_s,
        position_m,
        velocity_mps,
    }
}

fn coast(max_step_s: f64) -> Simulator<CentralGravity> {
    Simulator::new(
        Rocket::new(1.0, None).unwrap(),
        CentralGravity::earth(Vec3::zeros()).unwrap(),
        IntegratorSettings {
            max_step_s,
            ..Default::default()
        },
    )
    .unwrap()
}

fn specific_energy(state: &FlightState) -> f64 {
    state.velocity_mps.norm_squared() * 0.5 - MU / state.position_m.norm()
}

fn angular_momentum(state: &FlightState) -> f64 {
    state.position_m.cross(&state.velocity_mps).norm()
}

#[test]
fn central_gravity_rejects_invalid_parameters() {
    assert!(CentralGravity::new(0.0, Vec3::zeros()).is_err());
    assert!(CentralGravity::new(-MU, Vec3::zeros()).is_err());
    assert!(CentralGravity::new(f64::NAN, Vec3::zeros()).is_err());
    assert!(CentralGravity::earth(Vec3::new(f64::INFINITY, 0.0, 0.0)).is_err());
}

#[test]
fn central_gravity_is_singular_at_its_center() {
    let gravity = CentralGravity::earth(Vec3::zeros()).unwrap();
    let at_center = state(0.0, Vec3::zeros(), Vec3::x());
    assert!(gravity.acceleration(&at_center, 1.0).is_err());
}

#[test]
fn central_gravity_matches_inverse_square() {
    let center = Vec3::new(1.0, 2.0, 3.0);
    let gravity = CentralGravity::new(1000.0, center).unwrap();
    let probe = state(0.0, center + Vec3::new(10.0, 0.0, 0.0), Vec3::zeros());
    let a1 = gravity.acceleration(&probe, 1.0).unwrap();
    close(a1.norm(), 1000.0 / 100.0, 1e-12);
    close(a1.x, -10.0, 1e-12);
    assert_eq!(a1.y, 0.0);
    let probe = state(0.0, center + Vec3::new(20.0, 0.0, 0.0), Vec3::zeros());
    close(
        gravity.acceleration(&probe, 1.0).unwrap().norm(),
        1000.0 / 400.0,
        1e-12,
    );
}

#[test]
fn circular_orbit_conserves_energy_angular_momentum_and_closes() {
    let r0 = R_EARTH + 400_000.0;
    let v_circ = (MU / r0).sqrt();
    let mean_motion = (MU / r0.powi(3)).sqrt();
    let quarter_period = std::f64::consts::FRAC_PI_2 / mean_motion;
    let sim = coast(1.0);
    let start = state(0.0, Vec3::new(r0, 0.0, 0.0), Vec3::new(0.0, v_circ, 0.0));
    let result = sim
        .step(&start, quarter_period, ControlInput::default())
        .unwrap();
    // After a quarter period the body has advanced 90 degrees around the center.
    close(result.state.position_m.x, 0.0, 1e-3);
    close(result.state.position_m.y, r0, 1e-3);
    close(result.state.position_m.z, 0.0, 1e-9);
    close(result.state.velocity_mps.x, -v_circ, 1e-3);
    close(result.state.velocity_mps.y, 0.0, 1e-3);
    let energy_drift = (specific_energy(&result.state) - specific_energy(&start)).abs();
    let momentum_drift = (angular_momentum(&result.state) - angular_momentum(&start)).abs();
    assert!(
        energy_drift / specific_energy(&start).abs() < 1e-12,
        "energy drift {energy_drift:.3e}"
    );
    assert!(
        momentum_drift / angular_momentum(&start) < 1e-12,
        "angular momentum drift {momentum_drift:.3e}"
    );
}

#[test]
fn icbm_midcourse_ellipse_hits_apogee_and_returns() {
    // Surface launch, horizontal, targeted at a 1200 km apogee: a suborbital
    // midcourse ellipse with perigee at the launch radius.
    let r_perigee = R_EARTH;
    let r_apogee = R_EARTH + 1_200_000.0;
    let semi_major = (r_perigee + r_apogee) * 0.5;
    let v_perigee = (MU * (2.0 / r_perigee - 1.0 / semi_major)).sqrt();
    let v_apogee = (MU * (2.0 / r_apogee - 1.0 / semi_major)).sqrt();
    let half_period = std::f64::consts::PI * (semi_major.powi(3) / MU).sqrt();

    let sim = coast(2.0);
    let control = ControlInput::default();
    let start = state(
        0.0,
        Vec3::new(r_perigee, 0.0, 0.0),
        Vec3::new(0.0, v_perigee, 0.0),
    );
    let e0 = specific_energy(&start);
    let h0 = angular_momentum(&start);

    let apogee = sim.step(&start, half_period, control).unwrap();
    close(apogee.state.position_m.norm(), r_apogee, 1e-2);
    close(apogee.state.velocity_mps.norm(), v_apogee, 1e-2);
    // Velocity at apogee is purely tangential: r and v are perpendicular.
    close(
        apogee.state.position_m.dot(&apogee.state.velocity_mps),
        0.0,
        1.0,
    );

    let impact = sim.step(&apogee.state, half_period, control).unwrap();
    close(impact.state.position_m.norm(), r_perigee, 1e-2);
    close(impact.state.velocity_mps.norm(), v_perigee, 1e-2);
    close(impact.state.position_m.x, r_perigee, 1e-2);

    for state in [&apogee.state, &impact.state] {
        let energy_drift = (specific_energy(state) - e0).abs();
        let momentum_drift = (angular_momentum(state) - h0).abs();
        assert!(
            energy_drift / e0.abs() < 1e-10,
            "energy drift {energy_drift:.3e}"
        );
        assert!(
            momentum_drift / h0 < 1e-10,
            "angular momentum drift {momentum_drift:.3e}"
        );
    }
}

#[test]
fn altitude_switch_routes_and_validates() {
    assert!(AltitudeSwitch::new(
        UniformGravity::default(),
        UniformGravity(Vec3::zeros()),
        f64::NAN
    )
    .is_err());
    let low = UniformGravity::default();
    let high = UniformGravity(Vec3::zeros());
    let switch = AltitudeSwitch::new(low, high, 100.0).unwrap();
    close(switch.cutoff_altitude_m(), 100.0, 0.0);
    let below = state(0.0, Vec3::new(0.0, 50.0, 0.0), Vec3::zeros());
    close(switch.acceleration(&below, 1.0).unwrap().y, -9.80665, 0.0);
    // The cutoff itself still belongs to the lower model.
    let at = state(0.0, Vec3::new(0.0, 100.0, 0.0), Vec3::zeros());
    close(switch.acceleration(&at, 1.0).unwrap().y, -9.80665, 0.0);
    let above = state(0.0, Vec3::new(0.0, 150.0, 0.0), Vec3::zeros());
    assert_eq!(switch.acceleration(&above, 1.0).unwrap(), Vec3::zeros());
}

#[test]
fn altitude_switch_integrates_across_the_cutoff() {
    // Fall from rest above the cutoff: vacuum until 100 m, then full gravity.
    let switch = AltitudeSwitch::new(
        UniformGravity::default(),
        UniformGravity(Vec3::zeros()),
        100.0,
    )
    .unwrap();
    let sim = Simulator::new(
        Rocket::new(1.0, None).unwrap(),
        switch,
        IntegratorSettings::default(),
    )
    .unwrap();
    let start = state(0.0, Vec3::new(0.0, 150.0, 0.0), Vec3::zeros());
    let result = sim.step(&start, 0.05, ControlInput::default()).unwrap();
    assert_eq!(result.state.position_m.y, 150.0); // no acceleration above the cutoff
}
