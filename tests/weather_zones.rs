#![cfg(feature = "ballistics")]

use ballistics_engine::{
    air_at, drag::DragTable, KernelAtmosphere, LocalAir, RelativeHumidity, StationAir,
};
use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces, DEFAULT_HUMIDITY_PERCENT as RH},
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

/// The drag ratio a zone at half the default station's pressure (same
/// temperature and humidity) gives against the station itself, at
/// `state_at`'s height and speed, derived from the model the kernel flies.
///
/// A zone is that zone's own station air lapsed from the pad, so each side is
/// upstream's `air_at` on a station built at the zone's conditions (no zone
/// ladder, so a zone lookup that silently fell back to the station could not
/// agree with itself here). The kernel's drag on the body's own Cd curve is
/// `rho * v^2 * Cd(v / c)` times constants, so the ratio is
/// `rho_half * Cd(v / c_half) / (rho * Cd(v / c))`.
///
/// It is not 0.5. Dry, it is 0.49990, the CIPM compressibility factor alone.
/// At the default 50 % humidity it is 0.49771: the vapour pressure is fixed by
/// the temperature and humidity, not by the total pressure, so halving the
/// pressure doubles the water-vapour mole fraction (0.0084 to 0.0169), and
/// `air_at` holds each station's own fraction at every height. Vapour is
/// lighter than dry air, so the density falls to 0.49832 of the station's
/// (not 0.49990). It also carries sound faster, 340.81 against 340.36 m/s, so
/// the body flies at Mach 0.8802, not 0.8814, and Cd falls to 0.9988 of the
/// station's on the curve's rising 0.8-1.2 segment.
fn expected_half_pressure_drag_ratio() -> f64 {
    let Atmosphere::Standard {
        station_altitude_m,
        temperature_c,
        pressure_hpa,
        humidity_percent,
    } = Atmosphere::default()
    else {
        panic!("the default atmosphere is a station");
    };
    let state = state_at(0.0);
    let air = |pressure_hpa: f64| -> LocalAir {
        let humidity = RelativeHumidity::try_percent(humidity_percent).unwrap();
        let station =
            StationAir::new(station_altitude_m, temperature_c, pressure_hpa, humidity).unwrap();
        air_at(
            &KernelAtmosphere::Station(station),
            None,
            state.position_m.x,
            state.position_m.y,
        )
    };
    let (mach, cd) = CD.iter().copied().unzip();
    let table = DragTable::try_new(mach, cd).unwrap();
    let speed = state.velocity_mps.norm();
    let drag =
        |air: LocalAir| air.density_kg_m3 * table.interpolate(speed / air.speed_of_sound_mps);
    drag(air(pressure_hpa / 2.0)) / drag(air(pressure_hpa))
}

/// Tolerance on a drag ratio against [`expected_half_pressure_drag_ratio`]:
/// the two are the same arithmetic on the same air, so only round-off differs.
const RATIO_TOL: f64 = 1e-12;

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
    // Same temperature and humidity (the default atmosphere's), half the
    // station pressure → about half the density → about half the drag. Humid
    // air at half the pressure holds twice the vapour fraction, so it is a
    // little thinner than half and carries sound a little faster; see
    // `expected_half_pressure_drag_ratio`.
    let zoned = forces()
        .with_atmosphere_zones(vec![(15.0, 506.625, RH, 10_000.0)])
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
    let expected = expected_half_pressure_drag_ratio();
    assert!(
        (ratio - expected).abs() < RATIO_TOL,
        "expected drag ratio {expected}, got {ratio} ({drag_zoned:?} vs {drag_base:?})"
    );
}

#[test]
fn zones_switch_by_downrange_distance() {
    let zoned = forces()
        .with_atmosphere_zones(vec![
            (15.0, 1013.25, RH, 100.0),
            (15.0, 506.625, RH, 10_000.0),
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
    // Far zone has half the pressure, so about half the drag; the air does not
    // depend on downrange distance within a zone, so it is the same ratio.
    let far_base = drag_at(&base, 500.0);
    let far_zoned = drag_at(&zoned, 500.0);
    let ratio = far_zoned.norm() / far_base.norm();
    let expected = expected_half_pressure_drag_ratio();
    assert!(
        (ratio - expected).abs() < RATIO_TOL,
        "far-zone ratio {ratio}, expected {expected}"
    );
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
