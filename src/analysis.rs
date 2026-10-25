//! Analysis commands built on the shared trajectory runner: Monte Carlo
//! dispersion and elevation solving. No new physics — repeated integration.

use crate::command::{apogee, parse, run_trajectory, Ground, Row};
#[cfg(feature = "ballistics")]
use crate::Vec3;

/// First ground crossing between sampled rows, linearly interpolated on the
/// height above the ground. Skips degenerate all-on-the-pad runs by requiring
/// a sample above the ground first.
pub(crate) fn impact_row(rows: &[Row], ground: &Ground) -> Option<Row> {
    let height = |r: &Row| ground.height_m(crate::Vec3::new(r.x, r.y, r.z));
    rows.windows(2)
        .find(|pair| height(&pair[0]) > 0.0 && height(&pair[1]) <= 0.0)
        .map(|pair| ground.impact(&pair[0], &pair[1]))
}

/// SplitMix64: tiny deterministic PRNG so dispersion needs no dependencies.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal via Box-Muller.
    fn normal(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-300);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

pub(crate) fn dispersion(args: &[String]) -> Result<String, String> {
    let parsed = parse(args)?;
    if parsed.runs < 2 {
        return Err("error: --runs needs at least 2 samples".to_string());
    }
    if parsed.cd_sigma < 0.0 || parsed.wind_sigma < 0.0 || parsed.speed_sigma < 0.0 {
        return Err("error: sigmas must be nonnegative".to_string());
    }
    let mut rng = SplitMix64(parsed.seed);
    let mut apogees = Vec::with_capacity(parsed.runs);
    let mut impacts: Vec<(f64, f64)> = Vec::with_capacity(parsed.runs);
    for _ in 0..parsed.runs {
        let mut opts = parsed.opts.clone();
        #[cfg(feature = "ballistics")]
        {
            opts.cd_scale *= (parsed.cd_sigma * rng.normal()).exp();
            let wind = Vec3::new(
                parsed.wind_sigma * rng.normal(),
                parsed.wind_sigma * rng.normal(),
                parsed.wind_sigma * rng.normal(),
            );
            opts.wind_mps += wind;
            // Each dispersion run gets its own gust realization.
            opts.turbulence_seed = rng.next_u64();
        }
        #[cfg(not(feature = "ballistics"))]
        let _ = &mut rng;
        opts.speed_mps *= (1.0 + parsed.speed_sigma * rng.normal()).max(0.0);
        let (rows, _) = run_trajectory(&opts, true)?;
        let (peak, _) = apogee(&rows).ok_or("error: empty trajectory")?;
        apogees.push(peak);
        if let Some(impact) = impact_row(&rows, &opts.ground) {
            impacts.push((impact.x, impact.z));
        } else if let Some(last) = rows.last() {
            impacts.push((last.x, last.z));
        }
    }
    let (mean_apogee, sd_apogee) = mean_sd(&apogees);
    let ranges: Vec<f64> = impacts
        .iter()
        .map(|(x, z)| (x * x + z * z).sqrt())
        .collect();
    let (mean_range, sd_range) = mean_sd(&ranges);
    let cep = circular_error_probable(&impacts);
    Ok(format!(
        "runs: {} (seed {})\napogee_m: mean {mean_apogee:.2}, sd {sd_apogee:.2}\nrange_m: mean {mean_range:.2}, sd {sd_range:.2}\nimpact CEP_m: {cep:.2}\n",
        parsed.runs, parsed.seed
    ))
}

fn mean_sd(values: &[f64]) -> (f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0);
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    if values.len() < 2 {
        return (mean, 0.0);
    }
    let variance =
        values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (values.len() - 1) as f64;
    (mean, variance.sqrt())
}

/// Radius containing half the impacts, measured from the mean impact point.
fn circular_error_probable(impacts: &[(f64, f64)]) -> f64 {
    if impacts.is_empty() {
        return 0.0;
    }
    let mean_x = impacts.iter().map(|p| p.0).sum::<f64>() / impacts.len() as f64;
    let mean_z = impacts.iter().map(|p| p.1).sum::<f64>() / impacts.len() as f64;
    let mut distances: Vec<f64> = impacts
        .iter()
        .map(|(x, z)| ((x - mean_x).powi(2) + (z - mean_z).powi(2)).sqrt())
        .collect();
    distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
    distances[distances.len() / 2]
}

pub(crate) fn solve_range(args: &[String]) -> Result<String, String> {
    let parsed = parse(args)?;
    let target = parsed
        .target_m
        .ok_or("error: solve-range needs a target distance in metres")?;
    if target <= 0.0 {
        return Err("error: target range must be positive".to_string());
    }
    let range_at = |elevation: f64| -> Result<f64, String> {
        let mut opts = parsed.opts.clone();
        opts.elevation_deg = elevation;
        let (rows, _) = run_trajectory(&opts, true)?;
        impact_row(&rows, &opts.ground).map(|r| r.x).ok_or_else(|| {
            "error: no ground impact; increase --duration-s or the launch speed".to_string()
        })
    };
    let (lo, hi) = if parsed.high_angle {
        (45.0, 89.5)
    } else {
        (0.5, 45.0)
    };
    let range_lo = range_at(lo)?;
    let range_hi = range_at(hi)?;
    // Low branch: range climbs toward 45°. High branch: it falls from 45°.
    let bracketed = if parsed.high_angle {
        range_lo >= target && range_hi <= target
    } else {
        range_lo <= target && range_hi >= target
    };
    if !bracketed {
        return Err(format!(
            "error: target {target} m is not bracketed on this branch: {lo}° → {range_lo:.1} m, {hi}° → {range_hi:.1} m"
        ));
    }
    let (mut lo, mut hi) = (lo, hi);
    let mut mid = 0.0;
    for _ in 0..48 {
        mid = 0.5 * (lo + hi);
        let range = range_at(mid)?;
        let overshoots = if parsed.high_angle {
            range < target
        } else {
            range > target
        };
        if overshoots {
            hi = mid;
        } else {
            lo = mid;
        }
        if hi - lo < 1e-6 {
            break;
        }
    }
    let mut opts = parsed.opts.clone();
    opts.elevation_deg = mid;
    let (rows, _) = run_trajectory(&opts, false)?;
    let (peak, t) = apogee(&rows).ok_or("error: empty trajectory")?;
    let impact = impact_row(&rows, &opts.ground).ok_or("error: no ground impact")?;
    Ok(format!(
        "target_range_m: {target:.2}\nelevation_deg: {mid:.4}\nrange_m: {:.2}\napogee_m: {peak:.2} at t = {t:.3} s\nimpact_speed_mps: {:.2}\n",
        impact.x, impact.speed
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(s: &str) -> Vec<String> {
        crate::command::tokenize(s)
    }

    #[test]
    fn dispersion_is_deterministic_per_seed() {
        let args = argv(
            "dispersion --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --elevation-deg 60 --speed-mps 80 --duration-s 20 --dt-s 0.2 --runs 8 --seed 7",
        );
        let a = dispersion(&args[1..]).unwrap();
        let b = dispersion(&args[1..]).unwrap();
        assert_eq!(a, b);
        assert!(a.contains("runs: 8 (seed 7)"));
        assert!(a.contains("impact CEP_m:"));
    }

    #[test]
    fn dispersion_needs_two_runs() {
        let err = dispersion(
            &argv(
                "dispersion --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 --runs 1",
            )[1..],
        )
        .unwrap_err();
        assert!(err.contains("at least 2"));
    }

    #[test]
    fn solve_range_recovers_the_elevation_used_to_make_the_target() {
        // Build the target from a known 30° flight, then solve for it back.
        let base = "--curve 0:0,0.05:50,0.1:0 --dry-mass 1.0 --propellant-mass 0.01 \
                    --speed-mps 120 --duration-s 60 --dt-s 0.1";
        let mut opts = parse(&argv(&format!("simulate {base} --elevation-deg 30"))[1..])
            .unwrap()
            .opts;
        opts.elevation_deg = 30.0;
        let (rows, _) = run_trajectory(&opts, true).unwrap();
        let target = impact_row(&rows, &opts.ground).unwrap().x;
        let out =
            solve_range(&argv(&format!("solve-range {target} {base} --elevation-deg 30"))[1..])
                .unwrap();
        assert!(out.contains("elevation_deg: 30.0"), "got {out}");
        assert!(out.contains("range_m:"));
    }

    #[test]
    fn solve_range_rejects_unreachable_targets() {
        let err = solve_range(
            &argv(
                "solve-range 1000000 --curve 0:0,0.05:50,0.1:0 --dry-mass 1.0 \
             --propellant-mass 0.01 --speed-mps 120 --duration-s 60 --dt-s 0.1",
            )[1..],
        )
        .unwrap_err();
        assert!(err.contains("not bracketed"));
    }
}
