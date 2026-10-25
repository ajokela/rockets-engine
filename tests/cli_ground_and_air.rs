//! The CLI's ground, its weather station and the air every force reads.
#![cfg(all(feature = "cli", feature = "ballistics"))]

use rockets_engine::command::{run, tokenize};

fn sim(line: &str) -> Result<String, String> {
    run(&tokenize(line))
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    y: f64,
    speed: f64,
}

fn csv_rows(out: &str) -> Vec<Sample> {
    out.lines()
        .skip(1)
        .map(|line| {
            let cols: Vec<f64> = line
                .split(',')
                .take(5)
                .map(|c| c.parse().unwrap())
                .collect();
            Sample {
                y: cols[2],
                speed: cols[4],
            }
        })
        .collect()
}

/// Speed on the way down, interpolated where the descent crosses `y`.
fn descent_speed_at(rows: &[Sample], y: f64) -> f64 {
    let apogee = rows
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.y.total_cmp(&b.1.y))
        .unwrap()
        .0;
    for pair in rows[apogee..].windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if a.y > y && b.y <= y {
            let f = (a.y - y) / (a.y - b.y);
            return a.speed + (b.speed - a.speed) * f;
        }
    }
    panic!("descent never crosses y = {y}");
}

const CHUTE: &str = "simulate --curve 0:0,0.5:40,1:0 --dry-mass 0.3 \
                     --propellant-mass 0.05 --recover 0.5 --duration-s 60 --dt-s 0.25 \
                     --format csv";

#[test]
fn atmosphere_zones_reach_the_parachute() {
    // No drag adapter here: only recovery drag reads the air, so a
    // half-pressure zone must speed the descent by sqrt(rho0/rho) ~ 1.414.
    let plain = csv_rows(&sim(CHUTE).unwrap());
    let zoned = csv_rows(&sim(&format!("{CHUTE} --atmo-zone 15:506.625:0:100000")).unwrap());
    let ratio = descent_speed_at(&zoned, 50.0) / descent_speed_at(&plain, 50.0);
    // CIPM density at half the pressure is 0.49995 of sea level's.
    let expected = (1.0_f64 / 0.49995).sqrt();
    assert!(
        (ratio - expected).abs() / expected < 2e-3,
        "descent speed ratio {ratio}, expected {expected}"
    );
}

const SHOT: &str = "simulate --curve 0:0,0.05:50,0.1:0 --dry-mass 1.0 --propellant-mass 0.01 \
                    --speed-mps 120 --elevation-deg 45 --duration-s 60 --dt-s 0.2";

const MODEL_ROCKET: &str =
    "simulate --motor-eng 'Estes A8 18 70 0 0.0033 0.0164\\n0 0\\n1 9\\n2 0' \
     --geometry nose=ogive:0.07:0.024,tube=0.2:0.024,fins=3:0.05:0.03:0.05:0.02:0.0032 \
     --geometry-masses 0.00427,0.00741,0.01306 --weathercock --rail 1 --wind 3,0,1 \
     --recover 0.1 --recover-main 0.4 --duration-s 60 --dt-s 0.25";

fn line_starting<'a>(out: &'a str, prefix: &str) -> &'a str {
    out.lines()
        .find(|l| l.starts_with(prefix))
        .unwrap_or_else(|| panic!("no '{prefix}' line in:\n{out}"))
}

#[test]
fn ground_altitude_zero_is_the_old_default() {
    for cmd in [SHOT, MODEL_ROCKET] {
        for format in ["table --plot", "csv"] {
            let line = format!("{cmd} --format {format}");
            assert_eq!(
                sim(&line).unwrap(),
                sim(&format!("{line} --ground-altitude-m 0")).unwrap()
            );
        }
    }
}

#[test]
fn ground_altitude_raises_the_pad_and_lands_on_it() {
    // No air in SHOT: the flight is the same shape, raised by the ground,
    // and impact is Y reaching the ground, not sea level.
    let low = sim(SHOT).unwrap();
    let high = sim(&format!("{SHOT} --ground-altitude-m 1500")).unwrap();
    let first_row = high.lines().nth(1).unwrap();
    assert_eq!(
        first_row.split_whitespace().nth(2),
        Some("1500.00"),
        "{first_row}"
    );
    assert_eq!(
        line_starting(&low, "impact"),
        line_starting(&high, "impact")
    );
    let apogee = |out: &str| -> f64 {
        line_starting(out, "apogee")
            .split_whitespace()
            .nth(1)
            .unwrap()
            .parse()
            .unwrap()
    };
    assert!((apogee(&high) - apogee(&low) - 1500.0).abs() < 0.011);
}

#[test]
fn solve_range_and_dispersion_land_on_the_ground() {
    let solve = SHOT.replacen("simulate", "solve-range 500", 1);
    let low = sim(&solve).unwrap();
    let high = sim(&format!("{solve} --ground-altitude-m 1500")).unwrap();
    for key in ["elevation_deg:", "range_m:", "impact_speed_mps:"] {
        assert_eq!(line_starting(&low, key), line_starting(&high, key));
    }
    let spread = SHOT.replacen("simulate", "dispersion --runs 6", 1);
    let low = sim(&spread).unwrap();
    let high = sim(&format!("{spread} --ground-altitude-m 1500")).unwrap();
    for key in ["range_m:", "impact CEP_m:"] {
        assert_eq!(line_starting(&low, key), line_starting(&high, key));
    }
}

#[test]
fn the_station_is_at_the_ground_so_a_pad_reading_lapses_once() {
    // The CLI's station is ICAO standard air at the ground; entering that
    // same reading as a zone must fly identically (one lapse, from the
    // ground), not as if the reading were taken at sea level.
    let rockets_engine::ballistics::Atmosphere::Standard {
        temperature_c,
        pressure_hpa,
        ..
    } = rockets_engine::ballistics::Atmosphere::icao_standard(1500.0).unwrap()
    else {
        unreachable!()
    };
    let cmd = format!("{MODEL_ROCKET} --ground-altitude-m 1500 --format csv");
    assert_eq!(
        sim(&cmd).unwrap(),
        sim(&format!(
            "{cmd} --atmo-zone {temperature_c}:{pressure_hpa}:0:1000000"
        ))
        .unwrap()
    );
}

#[test]
fn main_parachute_defaults_to_300_m_above_the_ground() {
    let cmd = format!("{MODEL_ROCKET} --ground-altitude-m 1500 --format csv");
    let default = sim(&cmd).unwrap();
    assert_eq!(
        default,
        sim(&format!("{cmd} --main-altitude-m 1800")).unwrap()
    );
    assert_ne!(
        default,
        sim(&format!("{cmd} --main-altitude-m 300")).unwrap()
    );
}

#[test]
fn a_flat_frame_start_below_the_ground_is_refused() {
    for line in [
        format!("{SHOT} --ground-altitude-m 1500 --position-m 0,1000,0"),
        format!("{SHOT} --position-m 0,-5,0"),
        format!("{SHOT} --gravity vacuum --ground-altitude-m 200 --position-m 0,199,0"),
    ] {
        let err = sim(&line).unwrap_err();
        assert!(err.contains("below the ground"), "{line}: {err}");
    }
    // At or above the ground is fine, and --position-m stays absolute.
    let out = sim(&format!(
        "{SHOT} --ground-altitude-m 1500 --position-m 0,1600,0 --format csv"
    ))
    .unwrap();
    assert!(out.lines().nth(1).unwrap().starts_with("0,0,1600,0,"));
}

#[test]
fn ground_altitude_is_validated() {
    for value in ["nan", "inf", "20000", "-2000"] {
        let err = sim(&format!("{SHOT} --ground-altitude-m {value}")).unwrap_err();
        assert!(
            err.contains("--ground-altitude-m must be"),
            "{value}: {err}"
        );
    }
}

#[test]
fn the_plot_floor_is_the_ground() {
    let out = sim(&format!("{SHOT} --ground-altitude-m 1500 --plot")).unwrap();
    assert!(out.contains("altitude_m (peak "), "{out}");
    assert!(out.contains(", ground 1500.0)"), "{out}");
    // The launch sample sits on the chart's floor row.
    let floor = out
        .lines()
        .skip_while(|l| !l.starts_with("altitude_m"))
        .take_while(|l| l.starts_with('|') || l.starts_with("altitude_m"))
        .last()
        .unwrap();
    assert!(floor.starts_with("|*"), "{floor}");
}

const ARC: &str = "simulate --curve 0:0,0.05:50,0.1:0 --dry-mass 1.0 --propellant-mass 0.01 \
                   --speed-mps 1000 --elevation-deg 45 --duration-s 400 --dt-s 1 --gravity central";

/// The summary's impact numbers: x, z, t and speed.
fn impact_numbers(out: &str) -> Vec<f64> {
    line_starting(out, "impact")
        .split(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-'))
        .filter_map(|t| t.parse().ok())
        .collect()
}

#[test]
fn central_gravity_lands_on_the_sphere_in_either_frame() {
    // A ~100 km arc: the sphere falls ~800 m below the launch tangent plane,
    // so landing on the sphere is late by seconds, and the Earth-centred
    // frame (where Y never nears 0) must land too, on the same spot.
    let local = sim(&format!("{ARC} --center 0,-6371000,0")).unwrap();
    let centred = sim(&format!("{ARC} --position-m 0,6371000,0")).unwrap();
    let (a, b) = (impact_numbers(&local), impact_numbers(&centred));
    assert_eq!(a.len(), 4, "{a:?}");
    for (u, v) in a.iter().zip(&b) {
        assert!(
            (u - v).abs() < 0.02,
            "launch-local {a:?} vs Earth-centred {b:?}"
        );
    }
    let rows: Vec<(f64, f64)> = sim(&format!("{ARC} --center 0,-6371000,0 --format csv"))
        .unwrap()
        .lines()
        .skip(1)
        .map(|l| {
            let c: Vec<f64> = l.split(',').take(3).map(|v| v.parse().unwrap()).collect();
            (c[0], c[2])
        })
        .collect();
    let plane_crossing = rows
        .windows(2)
        .find(|w| w[0].1 > 0.0 && w[1].1 <= 0.0)
        .map(|w| w[0].0 + (w[1].0 - w[0].0) * w[0].1 / (w[0].1 - w[1].1))
        .unwrap();
    // ~800 m of drop at ~700 m/s of descent: about 1.2 s later.
    assert!(
        a[2] > plane_crossing + 0.5,
        "impact t {} should follow the tangent-plane crossing at {plane_crossing}",
        a[2]
    );
}

#[test]
fn central_gravity_ground_is_radius_plus_ground_altitude() {
    // Launch-local from the default pad (0, E, 0), Earth-centred from R + E,
    // and another planet with its own radius.
    let local = sim(&format!(
        "{ARC} --center 0,-6371000,0 --ground-altitude-m 1500"
    ))
    .unwrap();
    let centred = sim(&format!(
        "{ARC} --position-m 0,6372500,0 --ground-altitude-m 1500"
    ))
    .unwrap();
    let (a, b) = (impact_numbers(&local), impact_numbers(&centred));
    for (u, v) in a.iter().zip(&b) {
        assert!(
            (u - v).abs() < 0.02,
            "launch-local {a:?} vs Earth-centred {b:?}"
        );
    }
    let moon = sim(&format!(
        "{ARC} --mu 4.9048695e12 --radius-m 1737400 --center 0,-1737400,0 --duration-s 4000 --dt-s 5"
    ))
    .unwrap();
    assert!(impact_numbers(&moon)[0] > 0.0);
}

#[test]
fn a_start_inside_the_ground_sphere_is_refused() {
    for line in [
        // Default center and pad: the start is at the planet's center.
        ARC.to_string(),
        format!("{ARC} --position-m 0,6370000,0"),
        format!("{ARC} --position-m 0,6371000,0 --ground-altitude-m 1500"),
        format!("{ARC} --center 0,-6371000,0 --radius-m 6371001"),
    ] {
        let err = sim(&line).unwrap_err();
        assert!(err.contains("inside the ground"), "{line}: {err}");
    }
}

#[test]
fn recovery_and_the_space_switch_are_refused_under_central_gravity() {
    let local = format!("{ARC} --center 0,-6371000,0");
    let err = sim(&format!("{local} --recover 0.5")).unwrap_err();
    assert!(err.contains("--recover needs the flat frame"), "{err}");
    let err = sim(&format!("{local} --recover 0.5 --recover-main 1")).unwrap_err();
    assert!(err.contains("--recover needs the flat frame"), "{err}");
    let err = sim(&format!("{local} --space-above-m 80000")).unwrap_err();
    assert!(err.contains("--space-above-m does nothing"), "{err}");
}

#[test]
fn air_reading_forces_are_refused_under_central_gravity() {
    let local = format!("{ARC} --center 0,-6371000,0");
    for extra in [
        "--diameter-m 0.05 --cd 0:0.4,3:0.4",
        "--weathercock",
        "--airframe sport24 --weathercock",
        "--cna 0:2,3:2 --lift-k 0:0,3:0.5",
    ] {
        let err = sim(&format!("{local} {extra}")).unwrap_err();
        assert!(
            err.contains("central gravity and the drag adapter"),
            "{extra}: {err}"
        );
    }
}

#[test]
fn radius_needs_central_gravity() {
    let err = sim(&format!("{SHOT} --radius-m 6371000")).unwrap_err();
    assert!(err.contains("--radius-m needs --gravity central"), "{err}");
}

#[test]
fn the_default_pad_must_sit_on_the_ground_sphere() {
    // The default pad (0, E, 0) is on the ground only when --center is
    // R + E below it. Any other offset would leave it floating above the
    // sphere and fly the drop as extra range: refused, with the fix named.
    for line in [
        // WGS-84's equatorial radius as the offset, Earth's mean as R.
        format!("{ARC} --center 0,-6378137,0"),
        // The center offset by R + E instead of R.
        format!("{ARC} --center 0,-6372500,0 --ground-altitude-m 1500"),
        format!("{ARC} --center 0,-6371000,0 --radius-m 6370000"),
    ] {
        let err = sim(&line).unwrap_err();
        assert!(err.contains("default pad"), "{line}: {err}");
        assert!(err.contains("above the ground"), "{line}: {err}");
    }
    // An explicit start above the sphere is a launch from altitude.
    let out = sim(&format!("{ARC} --position-m 0,6471000,0")).unwrap();
    assert_eq!(impact_numbers(&out).len(), 4, "{out}");
}

#[test]
fn the_planet_radius_must_be_positive() {
    for radius in ["-50", "0", "nan"] {
        let err = sim(&format!(
            "{ARC} --mu 1 --center 0,-50,0 --radius-m {radius} --ground-altitude-m 100 \
             --position-m 0,50,0 --duration-s 2"
        ))
        .unwrap_err();
        assert!(err.contains("--radius-m must be"), "{radius}: {err}");
    }
}
