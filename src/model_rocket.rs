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
                "expected: Mfr Designation Diameter Length Delays PropellantMass TotalMass",
            ));
        }
        let number = |token: &str, field: &'static str| -> Result<f64> {
            token
                .parse()
                .map_err(|_| Error::invalid(field, "must be numeric"))
        };
        let (delays, propellant_mass_kg, total_mass_kg) = if tokens.len() >= 7 {
            (
                tokens[4].to_string(),
                number(tokens[5], "propellant_mass")?,
                number(tokens[6], "total_mass")?,
            )
        } else {
            (
                String::new(),
                number(tokens[4], "propellant_mass")?,
                number(tokens[5], "total_mass")?,
            )
        };
        let diameter_mm = number(tokens[2], "diameter_mm")?;
        let length_mm = number(tokens[3], "length_mm")?;
        // The header may share its line with thrust samples (e.g. when a paste
        // flattens newlines). Treat every token after the header as a flat
        // stream of time/thrust pairs, independent of original line breaks.
        let header_len = if tokens.len() >= 7 { 7 } else { 6 };
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
        Ok(Self {
            manufacturer: tokens[0].to_string(),
            designation: tokens[1].to_string(),
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

#[cfg(feature = "ballistics")]
pub use recovery::{Airframe, RecoveryForces};

#[cfg(feature = "ballistics")]
mod recovery {
    use crate::{
        ballistics::{local_air, Atmosphere, BallisticForces},
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
        atmosphere: Atmosphere,
        atmos_params: (f64, f64, f64, f64),
        wind_mps: Vec3,
        recovery_cd_area_m2: f64,
        deploy_after_s: f64,
    }

    impl<F> RecoveryForces<F> {
        /// `recovery_cd_area_m2` is the parachute's Cd·A product in m².
        /// `deploy_after_s` is an absolute simulation-time gate: deployment
        /// happens on the first descending state at or after it (0 = at
        /// apogee). `wind_mps` should match the inner model's uniform wind.
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
                atmos_params: atmosphere.parameters()?,
                atmosphere,
                wind_mps,
                recovery_cd_area_m2,
                deploy_after_s,
            })
        }

        fn recovery_accel(&self, state: &FlightState, mass_kg: f64) -> Vec3 {
            let v_air = state.velocity_mps - self.wind_mps;
            let speed = v_air.norm();
            let g = Vec3::new(0.0, -ballistics_engine::constants::G_ACCEL_MPS2, 0.0);
            if speed < 1e-12 {
                return g;
            }
            let (rho, _) = local_air(self.atmosphere, self.atmos_params, state.position_m.y);
            let q = 0.5 * rho * speed * speed;
            g - v_air * (q * self.recovery_cd_area_m2 / (speed * mass_kg))
        }

        fn recovering(&self, state: &FlightState) -> bool {
            state.velocity_mps.y <= 0.0 && state.time_s >= self.deploy_after_s
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
