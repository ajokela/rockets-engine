#![cfg(feature = "ballistics")]

use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    FlightState, ForceModel, UniformGravity, Vec3,
};

const CD: &[(f64, f64)] = &[(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)];
const CNA: &[(f64, f64)] = &[(0.0, 2.0), (1.0, 3.0), (3.0, 4.0)];
const LIFT_K: &[(f64, f64)] = &[(0.0, 0.0), (1.0, 0.5), (3.0, 1.0)];

const DIAMETER: f64 = 0.05;
const MASS: f64 = 0.3;
const DENSITY: f64 = 1.0;
const SOUND: f64 = 340.0;

fn forces() -> BallisticForces {
    BallisticForces::new(
        DIAMETER,
        CD.to_vec(),
        Atmosphere::Uniform {
            density_kg_m3: DENSITY,
            speed_of_sound_mps: SOUND,
        },
        Vec3::zeros(),
    )
    .unwrap()
    .with_body_lift(CNA.to_vec(), LIFT_K.to_vec())
    .unwrap()
}

fn state_at(v: Vec3) -> FlightState {
    FlightState {
        time_s: 0.0,
        position_m: Vec3::zeros(),
        velocity_mps: v,
    }
}

/// Analytic increment at a known angle of attack: N = q·A·Cnα·α toward the axis
/// side of the velocity, ΔD = q·A·k·α² along −v̂.
fn expected(alpha: f64, speed: f64) -> (f64, f64) {
    let mach = speed / SOUND;
    // Linear interpolation inside the [0, 1] table segment.
    let cna = 2.0 + mach;
    let k = 0.5 * mach;
    let q = 0.5 * DENSITY * speed * speed;
    let area = std::f64::consts::FRAC_PI_4 * DIAMETER * DIAMETER;
    (
        q * area * cna * alpha / MASS,
        q * area * k * alpha * alpha / MASS,
    )
}

#[test]
fn aligned_axis_gives_no_increment() {
    let f = forces();
    let v = Vec3::new(200.0, 0.0, 0.0);
    let state = state_at(v);
    let base = f.acceleration(&state, MASS).unwrap();
    let with_axis = f
        .acceleration_with_axis(&state, MASS, v.normalize())
        .unwrap();
    assert_eq!(base, with_axis);
}

#[test]
fn lift_matches_the_analytic_normal_force_and_induced_drag() {
    let f = forces();
    let speed = 200.0;
    let alpha = 5.0_f64.to_radians();
    let state = state_at(Vec3::new(speed, 0.0, 0.0));
    let axis = Vec3::new(alpha.cos(), alpha.sin(), 0.0);

    let base = f.acceleration(&state, MASS).unwrap();
    let with_axis = f.acceleration_with_axis(&state, MASS, axis).unwrap();
    let delta = with_axis - base;

    let (n_accel, d_accel) = expected(alpha, speed);
    // Lift points +Y (axis side); induced drag points −X.
    let expected_delta = Vec3::new(-d_accel, n_accel, 0.0);
    let err = (delta - expected_delta).norm();
    assert!(
        err < 1e-9,
        "delta {delta:?} != analytic {expected_delta:?} (err {err})"
    );
}

#[test]
fn lift_grows_linearly_and_drag_quadratically_in_alpha() {
    let f = forces();
    let speed = 200.0;
    let delta_at = |deg: f64| {
        let alpha = deg.to_radians();
        let state = state_at(Vec3::new(speed, 0.0, 0.0));
        let axis = Vec3::new(alpha.cos(), alpha.sin(), 0.0);
        f.acceleration_with_axis(&state, MASS, axis).unwrap()
            - f.acceleration(&state, MASS).unwrap()
    };
    let d5 = delta_at(5.0);
    let d10 = delta_at(10.0);
    let lift_ratio = d10.y / d5.y;
    let drag_ratio = d10.x / d5.x;
    assert!(
        (lift_ratio - 2.0).abs() < 1e-9,
        "lift should be linear in alpha: {lift_ratio}"
    );
    assert!(
        (drag_ratio - 4.0).abs() < 1e-9,
        "induced drag should be quadratic in alpha: {drag_ratio}"
    );
}

#[test]
fn lift_follows_the_axis_side() {
    let f = forces();
    let alpha = 5.0_f64.to_radians();
    let state = state_at(Vec3::new(200.0, 0.0, 0.0));
    // Axis below the velocity must pull down instead.
    let axis = Vec3::new(alpha.cos(), -alpha.sin(), 0.0);
    let delta = f.acceleration_with_axis(&state, MASS, axis).unwrap()
        - f.acceleration(&state, MASS).unwrap();
    assert!(delta.y < 0.0, "axis below velocity: {delta:?}");
    assert!(delta.z.abs() < 1e-12);
}

#[test]
fn still_air_or_zero_axis_gives_no_increment() {
    let f = forces();
    let parked = state_at(Vec3::zeros());
    let axis = Vec3::new(1.0, 0.0, 0.0);
    let delta = f.acceleration_with_axis(&parked, MASS, axis).unwrap()
        - f.acceleration(&parked, MASS).unwrap();
    assert_eq!(delta, Vec3::zeros());
    let moving = state_at(Vec3::new(200.0, 0.0, 0.0));
    let delta = f
        .acceleration_with_axis(&moving, MASS, Vec3::zeros())
        .unwrap()
        - f.acceleration(&moving, MASS).unwrap();
    assert_eq!(delta, Vec3::zeros());
}

/// Regression: at apogee a vertical flight reverses direction, so the
/// relative wind is exactly anti-parallel to the (fixed, up-pointing) body
/// axis. That is alpha = pi, not zero, but there is no perpendicular
/// component, so the lift increment must be zero — not NaN from a 0/0
/// `perp / sin_alpha`. This was a real crash: the geometry/body-lift path
/// failed with "body lift returned non-finite acceleration" the instant a
/// no-recovery flight crossed apogee.
#[test]
fn apogee_turnaround_is_zero_not_nan() {
    let f = forces();
    // Axis points up; velocity points straight down (post-apogee fall).
    let axis = Vec3::new(0.0, 1.0, 0.0);
    let falling = state_at(Vec3::new(0.0, -20.0, 0.0));
    let delta = f.acceleration_with_axis(&falling, MASS, axis).unwrap()
        - f.acceleration(&falling, MASS).unwrap();
    assert_eq!(delta, Vec3::zeros(), "anti-parallel wind must add no lift");
}

#[test]
fn body_lift_validates_its_tables() {
    let base = BallisticForces::new(
        DIAMETER,
        CD.to_vec(),
        Atmosphere::Uniform {
            density_kg_m3: DENSITY,
            speed_of_sound_mps: SOUND,
        },
        Vec3::zeros(),
    )
    .unwrap();
    assert!(base
        .clone()
        .with_body_lift(vec![(0.0, 2.0)], LIFT_K.to_vec())
        .is_err());
    assert!(base
        .clone()
        .with_body_lift(CNA.to_vec(), vec![(1.0, 0.5), (0.5, 0.5)])
        .is_err());
    assert!(base
        .clone()
        .with_body_lift(vec![(0.0, -1.0), (1.0, 2.0)], LIFT_K.to_vec())
        .is_err());
}

#[test]
fn orientation_blind_models_ignore_the_axis() {
    let g = UniformGravity::default();
    let state = state_at(Vec3::new(50.0, 10.0, 0.0));
    let a = g.acceleration(&state, MASS).unwrap();
    let b = g
        .acceleration_with_axis(&state, MASS, Vec3::new(0.0, 0.0, 1.0))
        .unwrap();
    assert_eq!(a, b);
}
