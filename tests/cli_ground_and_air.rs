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
