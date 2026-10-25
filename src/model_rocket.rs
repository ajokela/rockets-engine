//! Model-rocket conveniences: RASP `.eng` motor files, apogee recovery drag,
//! and illustrative airframe presets. No new integrator code — these are data
//! parsers and a force-model wrapper on the existing machinery.

use crate::{Error, Motor, Result, ThrustCurve, ThrustPoint};

/// A parsed RASP motor file (the `.eng` format used by thrustcurve.org and
/// OpenRocket): a header line of manufacturer, designation, diameter, length,
/// delays and masses, then time/thrust samples.
#[derive(Debug, Clone)]
pub struct MotorFile {
    /// Manufacturer code, e.g. "AeroTech" or "Estes".
    pub manufacturer: String,
    /// Motor designation, e.g. "C6" or "D12".
    pub designation: String,
    /// Motor diameter in millimetres.
    pub diameter_mm: f64,
    /// Motor length in millimetres.
    pub length_mm: f64,
    /// Delay token from the file ("0", "4", "P" for plugged, ...).
    pub delays: String,
    /// Propellant mass in kilograms.
    pub propellant_mass_kg: f64,
    /// Loaded motor mass (casing included) in kilograms.
    pub total_mass_kg: f64,
    /// The parsed thrust curve.
    pub curve: ThrustCurve,
}

impl MotorFile {
    /// Parse RASP text. `;` starts a comment, blank lines are ignored, and the
    /// first non-comment line must be the header.
    pub fn parse(text: &str) -> Result<Self> {
        let mut lines = text
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.starts_with(';'));
        let header = lines
            .next()
            .ok_or_else(|| Error::invalid("motor file", "no header line"))?;
        let tokens: Vec<&str> = header.split_whitespace().collect();
        if tokens.len() < 6 {
            return Err(Error::invalid(
                "motor file header",
                "expected: Designation Diameter Length Delays PropellantMass TotalMass [Mfr]",
            ));
        }
        let number = |token: &str, field: &'static str| -> Result<f64> {
            token
                .parse()
                .map_err(|_| Error::invalid(field, "must be numeric"))
        };
        let numeric = |token: &str| token.parse::<f64>().is_ok();
        // Two layouts: canonical RASP (thrustcurve.org) puts the designation
        // first and an optional manufacturer last; our legacy inline form
        // puts the manufacturer first.
        let (
            manufacturer,
            designation,
            diameter_mm,
            length_mm,
            delays,
            propellant_mass_kg,
            total_mass_kg,
            header_len,
        ) = if numeric(tokens[1]) {
            // Designation Diameter Length Delays PropellantMass TotalMass [Mfr]
            (
                tokens.get(6).map(|s| s.to_string()),
                tokens[0].to_string(),
                number(tokens[1], "diameter_mm")?,
                number(tokens[2], "length_mm")?,
                tokens[3].to_string(),
                number(tokens[4], "propellant_mass")?,
                number(tokens[5], "total_mass")?,
                6 + usize::from(tokens.len() > 6),
            )
        } else {
            // Mfr Designation Diameter Length [Delays] PropellantMass TotalMass
            if tokens.len() >= 7 {
                (
                    Some(tokens[0].to_string()),
                    tokens[1].to_string(),
                    number(tokens[2], "diameter_mm")?,
                    number(tokens[3], "length_mm")?,
                    tokens[4].to_string(),
                    number(tokens[5], "propellant_mass")?,
                    number(tokens[6], "total_mass")?,
                    7,
                )
            } else {
                (
                    Some(tokens[0].to_string()),
                    tokens[1].to_string(),
                    number(tokens[2], "diameter_mm")?,
                    number(tokens[3], "length_mm")?,
                    String::new(),
                    number(tokens[4], "propellant_mass")?,
                    number(tokens[5], "total_mass")?,
                    6,
                )
            }
        };
        // The header may share its line with thrust samples (e.g. when a paste
        // flattens newlines). Treat every token after the header as a flat
        // stream of time/thrust pairs, independent of original line breaks.
        let mut stream = tokens[header_len..]
            .iter()
            .copied()
            .chain(lines.flat_map(|line| line.split_whitespace()));
        let mut points = Vec::new();
        while let Some(time_tok) = stream.next() {
            let thrust_tok = stream
                .next()
                .ok_or_else(|| Error::invalid("motor file", "row needs time and thrust"))?;
            let time_s: f64 = time_tok
                .parse()
                .map_err(|_| Error::invalid("motor file", "non-numeric time"))?;
            let thrust_n: f64 = thrust_tok
                .parse()
                .map_err(|_| Error::invalid("motor file", "non-numeric thrust"))?;
            points.push(ThrustPoint { time_s, thrust_n });
        }
        if points.len() < 2 {
            return Err(Error::invalid(
                "motor file",
                "needs at least two thrust samples",
            ));
        }
        // RASP curves start at their first sample; thrust is zero at t = 0,
        // so prepend the origin when the file starts later (real files do).
        if points[0].time_s > 0.0 {
            points.insert(
                0,
                ThrustPoint {
                    time_s: 0.0,
                    thrust_n: 0.0,
                },
            );
        }
        Ok(Self {
            manufacturer: manufacturer.unwrap_or_default(),
            designation,
            diameter_mm,
            length_mm,
            delays,
            propellant_mass_kg,
            total_mass_kg,
            curve: ThrustCurve::new(points)?,
        })
    }

    /// Read and parse a `.eng` file from disk (native builds).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn from_path(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| Error::Model(format!("could not read motor file: {e}")))?;
        Self::parse(&text)
    }

    /// Build a scheduled motor from the parsed curve and propellant mass.
    pub fn motor(&self, ignition_time_s: f64) -> Result<Motor> {
        Motor::new(self.curve.clone(), self.propellant_mass_kg, ignition_time_s)
    }
}

/// The built-in motor names accepted by [`motor_by_name`].
pub const MOTOR_NAMES: &[&str] = &[
    "estes:a8",
    "estes:b6",
    "estes:c6",
    "estes:d12",
    "aerotech:g80",
];

/// A built-in motor curve by lowercase `mfr:designation` name.
///
/// Curve data: RASP files from thrustcurve.org, made from NAR published
/// static-test data (provenance in the embedded headers).
pub fn motor_by_name(name: &str) -> Option<MotorFile> {
    let text = match name {
        "estes:a8" => ESTES_A8,
        "estes:b6" => ESTES_B6,
        "estes:c6" => ESTES_C6,
        "estes:d12" => ESTES_D12,
        "aerotech:g80" => AEROTECH_G80,
        _ => return None,
    };
    MotorFile::parse(text).ok()
}

// Curve data: thrustcurve.org simfiles 5f4294d20002e900000004ea (D12, made
// from NAR published data, 2000-10-03).
const ESTES_D12: &str = "\
D12 24 70 0-3-5-7 0.0211 0.0426 Estes
0.049 2.569
0.116 9.369
0.184 17.275
0.237 24.258
0.282 29.73
0.297 27.01
0.311 22.589
0.322 17.99
0.348 14.126
0.386 12.099
0.442 10.808
0.546 9.876
0.718 9.306
0.879 9.105
1.066 8.901
1.257 8.698
1.436 8.31
1.59 8.294
1.612 4.613
1.65 0";

// Curve data: thrustcurve.org simfiles 69eb2379c7d63000021d8c29 (C6).
const ESTES_C6: &str = "\
C6 18 70 0-3-5-7 0.0108 0.0231 E
0.031 0.946
0.092 4.826
0.139 9.936
0.192 14.09
0.209 11.446
0.231 7.381
0.248 6.151
0.292 5.489
0.37 4.921
0.475 4.448
0.671 4.258
0.702 4.542
0.723 4.164
0.85 4.448
1.063 4.353
1.211 4.353
1.242 4.069
1.303 4.258
1.468 4.353
1.656 4.448
1.821 4.448
1.834 2.933
1.847 1.325
1.86 0";

// Curve data: thrustcurve.org simfiles (A8-0).
const ESTES_A8: &str = "\
A8 18 70 0 0.00384 0.01389 E
0.016 1.08
0.036 0.293
0.05 0.259
0.06 0.357
0.07 0.419
0.08 0.355
0.102 0.781
0.124 1.443
0.152 2.966
0.17 4.426
0.202 7.16
0.224 9.051
0.246 9.555
0.256 9.317
0.276 7.958
0.294 6.108
0.326 4.235
0.35 3.813
0.376 3.997
0.41 3.926
0.452 3.724
0.474 4.046
0.502 4.092
0.532 4.146
0.534 0";

// Curve data: thrustcurve.org simfiles (B6-0).
const ESTES_B6: &str = "\
B6-0 18 70 0 0.0056 0.0156 E
0.036 1.364
0.064 2.727
0.082 4.215
0.111 6.694
0.135 9.05
0.146 9.545
0.172 11.901
0.181 12.149
0.191 11.901
0.211 9.174
0.239 7.314
0.264 6.074
0.275 5.95
0.333 5.207
0.394 4.835
0.445 4.835
0.556 4.339
0.667 4.587
0.723 4.339
0.78 4.339
0.793 4.091
0.812 2.603
0.833 1.24
0.857 0";

// Curve data: thrustcurve.org simfiles 5f4294d20002e900000007dd (G80NBT).
const AEROTECH_G80: &str = "\
G80NBT 29 123.825 4-7-10 0.069 0.129 AT
0.013 89.054
0.018 101.584
0.029 105.388
0.047 102.927
0.104 100.018
0.19 102.255
0.268 104.94
0.306 104.269
0.352 105.835
0.41 104.94
0.432 106.73
0.512 105.612
0.563 106.954
0.591 104.493
0.62 106.283
0.715 103.15
0.76 103.374
0.806 101.36
0.848 102.032
1.2 88.383
1.243 77.866
1.269 57.952
1.302 44.079
1.329 34.011
1.367 26.403
1.398 22.823
1.433 20.585
1.48 21.704
1.508 20.809
1.584 13.201
1.604 11.635
1.652 11.188
1.683 5.37
1.701 0";

#[cfg(feature = "ballistics")]
pub use recovery::{Airframe, RecoveryForces};

#[cfg(feature = "ballistics")]
mod recovery {
    use crate::{
        ballistics::{Air, Atmosphere, BallisticForces},
        FlightState, ForceModel, Result, Vec3, WeathercockParams,
    };

    /// Apogee recovery: a stateless force-model wrapper that swaps the inner
    /// model for recovery drag once the vehicle is descending (vy <= 0) and
    /// the simulation time is at or past `deploy_after_s`. Recovery drag
    /// replaces the inner acceleration entirely except for gravity:
    /// `a = g − (q·CdA/m)·v̂` evaluated against local air. Terminal velocity
    /// `sqrt(2·m·g/(ρ·CdA))` emerges naturally.
    ///
    /// Because deployment is a pure function of the current state, the wrapper
    /// is deterministic and side-effect free, as `ForceModel` requires.
    #[derive(Debug, Clone)]
    pub struct RecoveryForces<F> {
        inner: F,
        air: Air,
        wind_mps: Vec3,
        wind_sock: Option<ballistics_engine::wind::WindSock>,
        wind_field: Option<crate::ballistics::SharedWindField>,
        turbulence: Option<crate::ballistics::Turbulence>,
        recovery_cd_area_m2: f64,
        deploy_after_s: f64,
        main: Option<(f64, f64)>,
    }

    impl<F> RecoveryForces<F> {
        /// `recovery_cd_area_m2` is the parachute's Cd·A product in m².
        /// `deploy_after_s` is an absolute simulation-time gate: deployment
        /// happens on the first descending state at or after it (0 = at
        /// apogee). `atmosphere` and `wind_mps` should match the inner model's
        /// air; pass its atmosphere zones too, through
        /// [`Self::with_atmosphere_zones`].
        pub fn new(
            inner: F,
            recovery_cd_area_m2: f64,
            deploy_after_s: f64,
            atmosphere: Atmosphere,
            wind_mps: Vec3,
        ) -> Result<Self> {
            if !recovery_cd_area_m2.is_finite() || recovery_cd_area_m2 <= 0.0 {
                return Err(crate::Error::invalid(
                    "recovery_cd_area_m2",
                    "must be finite and positive",
                ));
            }
            if !deploy_after_s.is_finite() || deploy_after_s < 0.0 {
                return Err(crate::Error::invalid(
                    "deploy_after_s",
                    "must be finite and nonnegative",
                ));
            }
            Ok(Self {
                inner,
                air: Air::new(atmosphere)?,
                wind_mps,
                wind_sock: None,
                wind_field: None,
                turbulence: None,
                recovery_cd_area_m2,
                deploy_after_s,
                main: None,
            })
        }

        /// Dual deployment: once the vehicle is recovering and at or below
        /// `altitude_m`, recovery drag uses the main parachute's Cd·A
        /// instead of the drogue's. Pure function of state, like the
        /// deployment gate itself. `altitude_m` is compared with position Y
        /// in the world frame, which with `BallisticForces` is altitude above
        /// sea level, not height above the launch pad.
        pub fn with_main(mut self, cd_area_m2: f64, altitude_m: f64) -> Result<Self> {
            if !cd_area_m2.is_finite() || cd_area_m2 <= 0.0 {
                return Err(crate::Error::invalid(
                    "main_cd_area_m2",
                    "must be finite and positive",
                ));
            }
            if !altitude_m.is_finite() {
                return Err(crate::Error::invalid("main_altitude_m", "must be finite"));
            }
            self.main = Some((cd_area_m2, altitude_m));
            Ok(self)
        }

        /// Read the same station-referenced atmosphere zones as the inner
        /// model (see [`BallisticForces::with_atmosphere_zones`]): with the
        /// same ladder, recovery drag uses the drag kernel's air at every
        /// position. The atmosphere passed to [`Self::new`] must be
        /// `Standard`.
        pub fn with_atmosphere_zones(mut self, zones: Vec<(f64, f64, f64, f64)>) -> Result<Self> {
            self.air = self.air.with_zones(zones)?;
            Ok(self)
        }

        /// Add the same deterministic frozen-field turbulence as the force
        /// model: equal `sigma`/`tau`/`seed` gives identical gusts, so the
        /// descent drifts through the same air the ascent climbed through.
        pub fn with_turbulence(mut self, sigma_mps: f64, tau_s: f64, seed: u64) -> Result<Self> {
            self.turbulence = Some(crate::ballistics::Turbulence::new(sigma_mps, tau_s, seed)?);
            Ok(self)
        }

        fn cd_area(&self, state: &FlightState) -> f64 {
            match self.main {
                Some((cd_area, altitude_m)) if state.position_m.y <= altitude_m => cd_area,
                _ => self.recovery_cd_area_m2,
            }
        }

        /// Sample downrange-segmented wind like the force model does.
        pub fn with_wind_segments(mut self, zones: Vec<(f64, f64, f64, f64)>) -> Result<Self> {
            let segments: Vec<ballistics_engine::wind::WindSegment> = zones
                .into_iter()
                .map(|(speed_mps, angle_deg, until_m, vertical_mps)| {
                    ballistics_engine::wind::WindSegment {
                        speed_kmh: speed_mps * 3.6,
                        angle_deg,
                        until_m,
                        vertical_mps,
                    }
                })
                .collect();
            self.wind_sock = Some(
                ballistics_engine::wind::WindSock::try_new(segments)
                    .map_err(|e| crate::Error::Model(format!("invalid wind segments: {e:?}")))?,
            );
            Ok(self)
        }

        /// Sample the same winds-engine field as the force model.
        pub fn with_wind_field(
            mut self,
            field: std::sync::Arc<dyn winds_engine::WindField + Send + Sync>,
        ) -> Result<Self> {
            self.wind_field = Some(crate::ballistics::SharedWindField(field));
            Ok(self)
        }

        fn effective_wind(&self, state: &FlightState) -> Vec3 {
            let base = match &self.wind_field {
                Some(field) => {
                    let w = field.0.wind(
                        state.time_s,
                        [state.position_m.x, state.position_m.y, state.position_m.z],
                    );
                    Vec3::new(w.x_mps, w.y_mps, w.z_mps)
                }
                None => match &self.wind_sock {
                    Some(sock) => sock.vector_for_range_stateless(state.position_m.x),
                    None => self.wind_mps,
                },
            };
            match &self.turbulence {
                Some(t) => base + t.gust_vec(state.time_s),
                None => base,
            }
        }

        fn recovery_accel(&self, state: &FlightState, mass_kg: f64) -> Vec3 {
            let v_air = state.velocity_mps - self.effective_wind(state);
            let speed = v_air.norm();
            let g = Vec3::new(0.0, -ballistics_engine::constants::G_ACCEL_MPS2, 0.0);
            if speed < 1e-12 {
                return g;
            }
            let (rho, _) = self.air.at(state.position_m);
            let q = 0.5 * rho * speed * speed;
            g - v_air * (q * self.cd_area(state) / (speed * mass_kg))
        }

        fn recovering(&self, state: &FlightState) -> bool {
            // Strictly descending: vy == 0 (pad, exact apogee) is not yet a
            // descent, so a standing wind at t = 0 cannot trip the chute.
            state.velocity_mps.y < 0.0 && state.time_s >= self.deploy_after_s
        }
    }

    impl<F: ForceModel> ForceModel for RecoveryForces<F> {
        fn acceleration(&self, state: &FlightState, mass_kg: f64) -> Result<Vec3> {
            if !self.recovering(state) {
                return self.inner.acceleration(state, mass_kg);
            }
            Ok(self.recovery_accel(state, mass_kg))
        }

        fn acceleration_with_axis(
            &self,
            state: &FlightState,
            mass_kg: f64,
            body_axis: Vec3,
        ) -> Result<Vec3> {
            if !self.recovering(state) {
                return self.inner.acceleration_with_axis(state, mass_kg, body_axis);
            }
            Ok(self.recovery_accel(state, mass_kg))
        }
    }

    /// Illustrative airframe parameters for common model-rocket shapes.
    /// These are generic, not measured data for any particular kit.
    #[derive(Debug, Clone)]
    pub struct Airframe {
        /// Body diameter in metres.
        pub diameter_m: f64,
        /// Axial drag coefficient vs Mach.
        pub cd_vs_mach: Vec<(f64, f64)>,
        /// Normal-force slope per radian vs Mach.
        pub cna_vs_mach: Vec<(f64, f64)>,
        /// Centre of pressure, metres aft of nose, vs Mach.
        pub cp_vs_mach: Vec<(f64, f64)>,
        /// Dry-structure CG, metres aft of nose.
        pub cg_dry_m: f64,
        /// Propellant CG, metres aft of nose.
        pub cg_prop_m: f64,
        /// Scalar pitch inertia about the CG, kg·m².
        pub pitch_inertia_kg_m2: f64,
        /// Induced-drag factor per radian² vs Mach.
        pub induced_k_vs_mach: Vec<(f64, f64)>,
        /// Recovery parachute Cd·A in m².
        pub recovery_cd_area_m2: f64,
    }

    impl Airframe {
        /// A generic 24 mm sport rocket (Estes-alpha-sized), illustrative only.
        pub fn sport_24mm() -> Self {
            Self {
                diameter_m: 0.024,
                cd_vs_mach: vec![(0.0, 0.4), (0.8, 0.45), (1.2, 0.65), (3.0, 0.5)],
                cna_vs_mach: vec![(0.0, 12.0), (1.0, 14.0), (3.0, 10.0)],
                cp_vs_mach: vec![(0.0, 0.19), (1.0, 0.2), (3.0, 0.18)],
                cg_dry_m: 0.12,
                cg_prop_m: 0.16,
                pitch_inertia_kg_m2: 0.004,
                // Tilt model: induced-drag factor equals the normal-force slope.
                induced_k_vs_mach: vec![(0.0, 12.0), (1.0, 14.0), (3.0, 10.0)],
                recovery_cd_area_m2: 0.25,
            }
        }

        /// A generic 54 mm high-power rocket, illustrative only.
        pub fn high_power_54mm() -> Self {
            Self {
                diameter_m: 0.054,
                cd_vs_mach: vec![(0.0, 0.35), (0.8, 0.4), (1.2, 0.6), (3.0, 0.45)],
                cna_vs_mach: vec![(0.0, 16.0), (1.0, 18.0), (3.0, 14.0)],
                cp_vs_mach: vec![(0.0, 0.42), (1.0, 0.45), (3.0, 0.4)],
                cg_dry_m: 0.28,
                cg_prop_m: 0.36,
                pitch_inertia_kg_m2: 0.03,
                // Tilt model: induced-drag factor equals the normal-force slope.
                induced_k_vs_mach: vec![(0.0, 16.0), (1.0, 18.0), (3.0, 14.0)],
                recovery_cd_area_m2: 0.8,
            }
        }

        /// Drag + atmosphere forces for this airframe in the given wind.
        pub fn forces(&self, wind_mps: Vec3) -> Result<BallisticForces> {
            BallisticForces::new(
                self.diameter_m,
                self.cd_vs_mach.clone(),
                Atmosphere::default(),
                wind_mps,
            )?
            .with_body_lift(self.cna_vs_mach.clone(), self.induced_k_vs_mach.clone())
        }

        /// Weathercock parameters for this airframe (CP/CG included).
        pub fn weathercock_params(&self) -> Result<WeathercockParams> {
            WeathercockParams::new(
                self.cna_vs_mach.clone(),
                self.cp_vs_mach.clone(),
                self.cg_dry_m,
                self.cg_prop_m,
                self.diameter_m,
                self.pitch_inertia_kg_m2,
                0.2,
            )
        }

        /// Look up a preset by CLI name (`sport24` or `hp54`).
        pub fn by_name(name: &str) -> Option<Self> {
            match name {
                "sport24" => Some(Self::sport_24mm()),
                "hp54" => Some(Self::high_power_54mm()),
                _ => None,
            }
        }
    }
}
