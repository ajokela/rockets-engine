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
