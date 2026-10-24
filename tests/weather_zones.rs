#![cfg(feature = "ballistics")]

use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    FlightState, Vec3,
};

const CD: &[(f64, f64)] = &[(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)];
const MASS: f64 = 0.3;
const G: f64 = 9.80665;

fn forces() -> BallisticForces {
    BallisticForces::new(0.05, CD.to_vec(), Atmosphere::default(), Vec3::zeros()).unwrap()
}

fn state_at(x: f64) -> FlightState {
    FlightState {
        time_s: 0.0,
        position_m: Vec3::new(x, 100.0, 0.0),
        velocity_mps: Vec3::new(300.0, 0.0, 0.0),
    }
}

#[test]
fn zones_require_standard_atmosphere() {
    let uniform = BallisticForces::new(
        0.05,
        CD.to_vec(),
        Atmosphere::Uniform {
            density_kg_m3: 1.0,
            speed_of_sound_mps: 340.0,
        },
        Vec3::zeros(),
    )
    .unwrap();
    assert!(uniform
        .with_atmosphere_zones(vec![(15.0, 1013.25, 0.0, 1000.0)])
        .is_err());
}

#[test]
fn low_pressure_zone_halves_the_drag() {
    // Same temperature and humidity, half the station pressure → half the
    // station density ratio → drag halves; sound speed (dry air) is unchanged.
    let zoned = forces()
        .with_atmosphere_zones(vec![(15.0, 506.625, 0.0, 10_000.0)])
        .unwrap();
    let base = forces();
    let state = state_at(0.0);
    let g = Vec3::new(0.0, -G, 0.0);
    let drag_base = base
        .acceleration_in_wind(&state, MASS, Vec3::zeros())
        .unwrap()
        - g;
    let drag_zoned = zoned
        .acceleration_in_wind(&state, MASS, Vec3::zeros())
        .unwrap()
        - g;
    let ratio = drag_zoned.norm() / drag_base.norm();
    // Not exactly 0.5: the CIPM compressibility factor varies slightly with
    // pressure, so half pressure is ~0.4999 density, not exactly 0.5.
    assert!(
        (ratio - 0.5).abs() < 2e-3,
        "expected half drag, ratio {ratio} ({drag_zoned:?} vs {drag_base:?})"
    );
}

#[test]
fn zones_switch_by_downrange_distance() {
    let zoned = forces()
        .with_atmosphere_zones(vec![
            (15.0, 1013.25, 0.0, 100.0),
            (15.0, 506.625, 0.0, 10_000.0),
        ])
        .unwrap();
    let base = forces();
    let g = Vec3::new(0.0, -G, 0.0);
    let drag_at = |f: &BallisticForces, x: f64| {
        f.acceleration_in_wind(&state_at(x), MASS, Vec3::zeros())
            .unwrap()
            - g
    };
    let near_base = drag_at(&base, 0.0);
    let near_zoned = drag_at(&zoned, 0.0);
    // First zone mirrors the base atmosphere.
    assert!((near_zoned - near_base).norm() < 1e-9 * near_base.norm().max(1.0));
    // Far zone has half the pressure, so half the drag.
    let far_base = drag_at(&base, 500.0);
    let far_zoned = drag_at(&zoned, 500.0);
    let ratio = far_zoned.norm() / far_base.norm();
    assert!((ratio - 0.5).abs() < 2e-3, "far-zone ratio {ratio}");
}

#[test]
fn wind_segments_apply_only_in_their_zone() {
    use rockets_engine::ForceModel;
    // No wind for the first 100 m, then a 5 m/s headwind (0° per the upstream
    // wind-FROM convention) beyond it. The trait method selects the zone wind;
    // acceleration_in_wind takes an explicit vector by design.
    let segmented = forces()
        .with_wind_segments(vec![(0.0, 0.0, 100.0, 0.0), (5.0, 0.0, 10_000.0, 0.0)])
        .unwrap();
    let base = forces();
    let near_base = base.acceleration(&state_at(0.0), MASS).unwrap();
    let near_seg = segmented.acceleration(&state_at(0.0), MASS).unwrap();
    assert!(
        (near_seg - near_base).norm() < 1e-9,
        "zone 1 should be calm"
    );
    let far_base = base.acceleration(&state_at(500.0), MASS).unwrap();
    let far_seg = segmented.acceleration(&state_at(500.0), MASS).unwrap();
    assert!(
        (far_seg - far_base).norm() > 1e-3,
        "zone 2 should feel the headwind"
    );
    // Headwind increases the drag deceleration along -X.
    assert!(far_seg.x < far_base.x);
}

#[test]
fn empty_or_invalid_zone_ladders_are_rejected() {
    assert!(forces().with_atmosphere_zones(vec![]).is_err());
    assert!(forces().with_wind_segments(vec![]).is_err());
    assert!(forces()
        .with_atmosphere_zones(vec![(15.0, -1.0, 0.0, 100.0)])
        .is_err());
}
