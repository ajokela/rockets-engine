//! Parity checks against OpenRocket's own test-suite expected values
//! (BarrowmanCalculatorTest: Estes Alpha III CP/CNa and fin-count variants).
#![cfg(all(feature = "model", feature = "ballistics"))]

use rockets_engine::geometry::{BodyTube, NoseCone, NoseShape, RocketGeometry, TrapezoidalFins};

fn alpha_iii(fin_count: u32) -> RocketGeometry {
    RocketGeometry::new(
        NoseCone {
            shape: NoseShape::Ogive,
            length_m: 0.07,
            diameter_m: 0.024,
        },
        BodyTube {
            length_m: 0.20,
            diameter_m: 0.024,
        },
        TrapezoidalFins {
            count: fin_count,
            root_chord_m: 0.05,
            tip_chord_m: 0.03,
            span_m: 0.05,
            leading_sweep_m: 0.02,
            thickness_m: 0.0032,
        },
    )
}

/// CNa and CP at Mach 0.3, the implicit Mach of OpenRocket's
/// BarrowmanCalculatorTest (its FlightConditions defaults to 0.3). The
/// analysis tables sample 0.0/0.2/0.4, so interpolate linearly.
fn cp_cna(geo: &RocketGeometry) -> (f64, f64) {
    let analysis = geo.analyze();
    let interp = |table: &[(f64, f64)]| {
        let lo = table.iter().find(|(m, _)| *m == 0.2).unwrap().1;
        let hi = table.iter().find(|(m, _)| *m == 0.4).unwrap().1;
        (lo + hi) / 2.0
    };
    (interp(&analysis.cna_vs_mach), interp(&analysis.cp_vs_mach))
}

#[test]
fn alpha_iii_three_fins_matches_openrocket() {
    let (cna, cp) = cp_cna(&alpha_iii(3));
    // BarrowmanCalculatorTest: exp_cna = 26.14693374, exp_cpx = 0.22351541.
    assert!(
        (cna - 26.14693374).abs() / 26.14693374 < 0.005,
        "CNa {cna} vs OpenRocket 26.14693374"
    );
    assert!(
        (cp - 0.22351541).abs() / 0.22351541 < 0.005,
        "CP {cp} vs OpenRocket 0.22351541"
    );
}

#[test]
fn alpha_iii_four_fins_matches_openrocket() {
    let (cna, cp) = cp_cna(&alpha_iii(4));
    // BarrowmanCalculatorTest.testFinCountEffect: 34.19591165 / 0.22724216.
    assert!(
        (cna - 34.19591165).abs() / 34.19591165 < 0.005,
        "CNa {cna} vs OpenRocket 34.19591165"
    );
    assert!(
        (cp - 0.22724216).abs() / 0.22724216 < 0.005,
        "CP {cp} vs OpenRocket 0.22724216"
    );
}

#[test]
fn alpha_iii_fin_cna_matches_hand_value() {
    // The test file's hand-computed 3-fin value is 24.146933.
    let (cna, _) = cp_cna(&alpha_iii(3));
    let fin_only = cna - 2.0; // subtract the nose's 2
    assert!(
        (fin_only - 24.146933).abs() / 24.146933 < 0.005,
        "fin CNa {fin_only} vs hand value 24.146933"
    );
}
