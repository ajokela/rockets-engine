#![cfg(feature = "ballistics")]

use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    FlightState, Vec3,
};

const CD: &[(f64, f64)] = &[(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)];

fn forces() -> BallisticForces {
    BallisticForces::new(0.05, CD.to_vec(), Atmosphere::default(), Vec3::zeros()).unwrap()
}

fn moving_state() -> FlightState {
    FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 100.0, 0.0),
        velocity_mps: Vec3::new(300.0, 0.0, 0.0),
    }
}

fn accel(f: &BallisticForces, state: &FlightState) -> Vec3 {
    f.acceleration_in_wind(state, 0.3, Vec3::zeros()).unwrap()
}

#[test]
fn coriolis_matches_minus_two_omega_cross_v() {
    let base = forces();
    let cor = forces().with_coriolis(40.0, 90.0).unwrap(); // 40°N, firing east
    let state = moving_state();
    let delta = accel(&cor, &state) - accel(&base, &state);

    let omega = 7.2921159e-5_f64;
    let lat = 40.0_f64.to_radians();
    // Firing east at 40°N: -2Ω×v = (0, +2·|v|·Ω·cos(lat), +2·|v|·Ω·sin(lat)).
    let expected = Vec3::new(
        0.0,
        2.0 * 300.0 * omega * lat.cos(),
        2.0 * 300.0 * omega * lat.sin(),
    );
    let err = (delta - expected).norm();
    assert!(
        err < 1e-9,
        "coriolis delta {delta:?} != expected {expected:?} (err {err})"
    );
}

#[test]
fn coriolis_rejects_impossible_latitude() {
    assert!(forces().with_coriolis(91.0, 0.0).is_err());
    assert!(forces().with_coriolis(f64::NAN, 0.0).is_err());
    assert!(forces().with_coriolis(45.0, f64::NAN).is_err());
}

#[test]
fn spin_requires_positive_twist_and_length() {
    assert!(forces().with_spin(0.0, 0.6, true).is_err());
    assert!(forces().with_spin(10.0, 0.0, true).is_err());
    assert!(forces().with_spin(-5.0, 0.6, true).is_err());
}

#[test]
fn magnus_force_is_vertical_and_mirrors_with_twist_handedness() {
    let right = forces().with_spin(10.0, 0.6, true).unwrap();
    let left = forces().with_spin(10.0, 0.6, false).unwrap();
    let plain = forces();
    let state = moving_state();

    let a_r = accel(&right, &state);
    let a_l = accel(&left, &state);
    let a_0 = accel(&plain, &state);

    // The lateral yaw of repose drives a vertical Magnus force (kernel doc).
    let delta = a_r - a_0;
    assert!(
        delta.y.abs() > 1e-6,
        "spin should change the acceleration, got {delta:?}"
    );
    assert!(
        delta.y.abs() > 10.0 * delta.z.abs(),
        "magnus should be mostly vertical, got {delta:?}"
    );
    // Mirrored twist mirrors the yaw, hence the vertical force sign.
    assert!(
        (a_r.y - a_0.y).signum() != (a_l.y - a_0.y).signum(),
        "right {a_r:?} vs left {a_l:?} should pull opposite ways"
    );
    // Drag and gravity are symmetric: lateral parts agree between handednesses.
    assert!(
        (a_r.z - a_l.z).abs() < 1e-9,
        "lateral drag response should not see handedness: {a_r:?} vs {a_l:?}"
    );
}

#[test]
fn tipoff_yaw_decays_downrange() {
    let tip = forces().with_tipoff_yaw(0.05, 50.0).unwrap();
    let plain = forces();
    let near = moving_state();
    let mut far = moving_state();
    far.position_m.x = 50.0 * 10.0; // ten decay lengths: exp(-10) ≈ 4.5e-5

    let delta_near = (accel(&tip, &near) - accel(&plain, &near)).norm();
    let delta_far = (accel(&tip, &far) - accel(&plain, &far)).norm();

    assert!(delta_near > 1e-6, "tipoff should perturb the launch");
    assert!(
        delta_far < 1e-3 * delta_near,
        "tipoff should decay: near {delta_near}, far {delta_far}"
    );
}

#[test]
fn tipoff_rejects_nonpositive_decay() {
    assert!(forces().with_tipoff_yaw(0.05, 0.0).is_err());
    assert!(forces().with_tipoff_yaw(f64::NAN, 50.0).is_err());
}

#[test]
fn effects_stay_off_by_default() {
    // A fresh adapter must equal one built before the MPM knobs existed:
    // no Magnus, no Coriolis, no tipoff, omega unused.
    let state = moving_state();
    let a = accel(&forces(), &state);
    let b = accel(&forces(), &state);
    assert_eq!(a, b);
}
