//! Geometry-to-aerodynamics: a subsonic Barrowman subset for slender model
//! rockets (nose cone + body tube + trapezoidal fins), implemented against the
//! OpenRocket technical documentation and its `FinSetCalc`/`BarrowmanCalculator`
//! sources. CP comes from the component moment sum; drag is friction + base +
//! conical nose pressure. Supersonic values are clamped to the Mach-1 results.
//!
//! Induced drag from angle of attack uses the normal-force tilt model
//! (Cd += Cnα·α²). Not covered (deliberately): supersonic wave drag,
//! boat-tails/transitions, and elliptical or free-form fins.

use crate::{Error, Result};

/// Nose-cone families supported by the subset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoseShape {
    /// Straight conical nose.
    Conical,
    /// Tangent ogive nose.
    Ogive,
    /// Half-ellipsoid nose.
    Elliptical,
    /// Power-series parabolic nose (exponent 0.5).
    Parabolic,
}

impl std::str::FromStr for NoseShape {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        match s {
            "conical" | "cone" => Ok(Self::Conical),
            "ogive" => Ok(Self::Ogive),
            "elliptical" | "ellipse" => Ok(Self::Elliptical),
            "parabolic" => Ok(Self::Parabolic),
            _ => Err(Error::invalid(
                "nose shape",
                "conical, ogive, elliptical or parabolic",
            )),
        }
    }
}

impl NoseShape {
    /// CP location as a fraction of nose length, from the volume/area relation
    /// XB = L − V/A_base. Ogive uses the standard tangent-ogive value.
    fn cp_fraction(self) -> f64 {
        match self {
            Self::Conical => 2.0 / 3.0,
            Self::Ogive => 0.466,
            Self::Elliptical => 1.0 / 3.0,
            Self::Parabolic => 0.5,
        }
    }

    /// Pressure-drag coefficient at subsonic speeds. Conical noses use
    /// 0.8·sin²(φ) with the half-angle φ = atan(r/L); streamlined shapes are
    /// taken as zero, the usual Barrowman approximation.
    fn pressure_cd(self, length_m: f64, diameter_m: f64) -> f64 {
        match self {
            Self::Conical => {
                let r = diameter_m / 2.0;
                let sin_phi = r / (r * r + length_m * length_m).sqrt();
                0.8 * sin_phi * sin_phi
            }
            _ => 0.0,
        }
    }

    /// Lateral (wetted) area. Conical uses the exact slant area; the others
    /// approximate the cylinder at the base diameter.
    fn lateral_area(self, length_m: f64, diameter_m: f64) -> f64 {
        let r = diameter_m / 2.0;
        match self {
            Self::Conical => std::f64::consts::PI * r * (r * r + length_m * length_m).sqrt(),
            _ => std::f64::consts::PI * diameter_m * length_m,
        }
    }
}

/// A nose cone with length and base diameter, in metres.
#[derive(Debug, Clone, Copy)]
pub struct NoseCone {
    /// Shape family.
    pub shape: NoseShape,
    /// Nose length in metres.
    pub length_m: f64,
    /// Base diameter in metres.
    pub diameter_m: f64,
}

/// The cylindrical body tube behind the nose.
#[derive(Debug, Clone, Copy)]
pub struct BodyTube {
    /// Tube length in metres.
    pub length_m: f64,
    /// Tube diameter in metres.
    pub diameter_m: f64,
}

/// Identical trapezoidal fins at the aft end of the tube. `leading_sweep_m` is
/// the horizontal offset of the tip leading edge from the root leading edge.
#[derive(Debug, Clone, Copy)]
pub struct TrapezoidalFins {
    /// Number of identical fins.
    pub count: u32,
    /// Root chord length in metres.
    pub root_chord_m: f64,
    /// Tip chord length in metres.
    pub tip_chord_m: f64,
    /// Fin semispan in metres (body surface to tip).
    pub span_m: f64,
    /// Horizontal offset of the tip leading edge from the root leading edge.
    pub leading_sweep_m: f64,
    /// Fin thickness in metres.
    pub thickness_m: f64,
}

/// The complete slender geometry.
#[derive(Debug, Clone, Copy)]
pub struct RocketGeometry {
    /// Nose cone.
    pub nose: NoseCone,
    /// Cylindrical body tube.
    pub tube: BodyTube,
    /// Aft trapezoidal fin set.
    pub fins: TrapezoidalFins,
    /// Roughness height in metres (default 20 µm, painted surface).
    pub roughness_m: f64,
}

impl RocketGeometry {
    /// Build with the default painted-surface roughness (20 µm).
    pub fn new(nose: NoseCone, tube: BodyTube, fins: TrapezoidalFins) -> Self {
        Self {
            nose,
            tube,
            fins,
            roughness_m: 20e-6,
        }
    }

    /// Set the roughness height in metres.
    pub fn with_roughness(mut self, roughness_m: f64) -> Self {
        self.roughness_m = roughness_m;
        self
    }

    /// Total aerodynamic length (nose + tube).
    pub fn total_length_m(&self) -> f64 {
        self.nose.length_m + self.tube.length_m
    }

    /// Largest body diameter (nose base or tube).
    pub fn max_diameter_m(&self) -> f64 {
        self.nose.diameter_m.max(self.tube.diameter_m)
    }

    /// Reference area, π/4·d² of the largest diameter.
    pub fn ref_area_m2(&self) -> f64 {
        std::f64::consts::FRAC_PI_4 * self.max_diameter_m().powi(2)
    }

    fn fin_area(&self) -> f64 {
        self.fins.span_m * (self.fins.root_chord_m + self.fins.tip_chord_m) / 2.0
    }

    /// Mean aerodynamic chord of one fin (trapezoid formula).
    fn mac(&self) -> f64 {
        let (cr, ct) = (self.fins.root_chord_m, self.fins.tip_chord_m);
        (2.0 / 3.0) * (cr + ct - cr * ct / (cr + ct))
    }

    /// Fin CP measured from the fin's root leading edge: xMAC_LE + MAC/4.
    fn fin_cp(&self) -> f64 {
        let (cr, ct, s, m) = (
            self.fins.root_chord_m,
            self.fins.tip_chord_m,
            self.fins.span_m,
            self.fins.leading_sweep_m,
        );
        let y_mac = (s / 3.0) * (cr + 2.0 * ct) / (cr + ct);
        let x_mac_le = m * y_mac / s;
        x_mac_le + 0.25 * self.mac()
    }

    fn midchord_cos(&self) -> f64 {
        let (cr, ct, s, m) = (
            self.fins.root_chord_m,
            self.fins.tip_chord_m,
            self.fins.span_m,
            self.fins.leading_sweep_m,
        );
        let gamma = ((m + ct / 2.0 - cr / 2.0) / s).atan();
        gamma.cos()
    }

    /// Normal-force slope of one fin, divided by the reference area.
    /// OpenRocket's subsonic Diederich simplification.
    fn fin_cna1(&self, mach: f64) -> f64 {
        let span = self.fins.span_m;
        let area = self.fin_area();
        let cos = self.midchord_cos();
        if area <= 0.0 || span <= 0.0 || cos <= 0.0 {
            return 0.0;
        }
        let ratio = (span * span) / (area * cos);
        2.0 * std::f64::consts::PI * span * span
            / (1.0 + (1.0 + (1.0 - mach * mach) * ratio * ratio).sqrt())
            / self.ref_area_m2()
    }

    /// Body-interference factor (1 + tau) with tau = r/(span + r).
    fn interference_tau(&self) -> f64 {
        let r = self.max_diameter_m() / 2.0;
        r / (self.fins.span_m + r)
    }

    fn fin_count_factor(&self) -> f64 {
        match self.fins.count {
            5 => 0.948,
            6 => 0.913,
            7 => 0.854,
            8 => 0.810,
            n if n >= 9 => 0.75,
            _ => 1.0,
        }
    }

    /// Total fin-set normal-force slope for this Mach number.
    ///
    /// The Diederich `fin_cna1` is the slope of a fin *pair* (a wing of
    /// semispan `span`), so a set of `n` fins contributes `n/2` pairs,
    /// with fin-body carryover (1 + tau) and a many-fin interference
    /// reduction. This matches OpenRocket's `FinSetCalc` convention,
    /// verified against its BarrowmanCalculatorTest Alpha III values.
    fn fin_set_cna(&self, mach: f64) -> f64 {
        self.fin_cna1(mach)
            * (1.0 + self.interference_tau())
            * self.fin_count_factor()
            * (self.fins.count as f64 / 2.0)
    }

    /// Fin CP measured from the nose tip (fin trailing edge at body end).
    fn fin_cp_abs(&self) -> f64 {
        self.total_length_m() - self.fins.root_chord_m + self.fin_cp()
    }

    /// Nose radius profile r(x) with x measured from the tip.
    fn nose_radius(&self, x: f64) -> f64 {
        let l = self.nose.length_m;
        let r = self.nose.diameter_m / 2.0;
        let t = (x / l).clamp(0.0, 1.0);
        match self.nose.shape {
            NoseShape::Conical => r * t,
            NoseShape::Elliptical => r * (1.0 - (1.0 - t).powi(2)).sqrt(),
            NoseShape::Parabolic => r * t * (2.0 - t),
            NoseShape::Ogive => {
                let rho = (r * r + l * l) / (2.0 * r);
                (rho * rho - (l - x).powi(2)).max(0.0).sqrt() - (rho - r)
            }
        }
    }

    /// Solid-nose CG (from the tip) and pitch inertia per unit mass about
    /// that CG, from numerical integration of the radius profile.
    fn nose_volume_properties(&self) -> (f64, f64) {
        const N: usize = 1024;
        let l = self.nose.length_m;
        let dx = l / N as f64;
        let mut volume = 0.0; // ∫ r² dx (up to the density·π factor)
        let mut moment = 0.0; // ∫ x r² dx
        for i in 0..N {
            let x = (i as f64 + 0.5) * dx;
            let r2 = self.nose_radius(x).powi(2);
            volume += r2 * dx;
            moment += x * r2 * dx;
        }
        let cg = moment / volume;
        let mut inertia = 0.0; // ∫ r² (r²/4 + (x−cg)²) dx
        for i in 0..N {
            let x = (i as f64 + 0.5) * dx;
            let r2 = self.nose_radius(x).powi(2);
            inertia += r2 * (r2 / 4.0 + (x - cg).powi(2)) * dx;
        }
        (cg, inertia / volume)
    }

    /// Fin planform centroid, measured from the root leading edge.
    fn fin_planform_cg(&self) -> f64 {
        let (cr, ct, m) = (
            self.fins.root_chord_m,
            self.fins.tip_chord_m,
            self.fins.leading_sweep_m,
        );
        (cr * cr + cr * ct + ct * ct + m * (cr + 2.0 * ct)) / (3.0 * (cr + ct))
    }

    /// Fin centroid height above the body surface.
    fn fin_radial_cg(&self) -> f64 {
        let (cr, ct, s) = (
            self.fins.root_chord_m,
            self.fins.tip_chord_m,
            self.fins.span_m,
        );
        s * (cr + 2.0 * ct) / (3.0 * (cr + ct))
    }

    /// Combine component masses with the geometry's component CGs into a
    /// dry CG and a pitch-inertia estimate about that CG.
    ///
    /// The nose is a solid of revolution (numerically integrated), the
    /// tube a thin-walled cylinder, and the fins and optional motor casing
    /// point masses; evenly spaced fins add n/2 · m · (r_body + y_cg)² for
    /// their radial offset. `casing` is (mass_kg, CG from the nose tip).
    pub fn mass_properties(
        &self,
        nose_kg: f64,
        tube_kg: f64,
        fins_kg: f64,
        casing: Option<(f64, f64)>,
    ) -> MassProperties {
        assert!(
            nose_kg >= 0.0 && tube_kg >= 0.0 && fins_kg >= 0.0,
            "component masses must be non-negative"
        );
        let (nose_cg, nose_inertia) = self.nose_volume_properties();
        let tube_cg = self.nose.length_m + self.tube.length_m / 2.0;
        let fin_cg = self.total_length_m() - self.fins.root_chord_m + self.fin_planform_cg();
        let mut parts: Vec<(f64, f64, f64)> = vec![
            (nose_kg, nose_cg, nose_kg * nose_inertia),
            (
                tube_kg,
                tube_cg,
                tube_kg * (self.tube.length_m.powi(2) / 12.0 + self.tube.diameter_m.powi(2) / 8.0),
            ),
            (fins_kg, fin_cg, 0.0),
        ];
        if let Some((kg, cg)) = casing {
            parts.push((kg, cg, 0.0));
        }
        let total: f64 = parts.iter().map(|p| p.0).sum();
        assert!(total > 0.0, "component masses must sum to a positive mass");
        let cg: f64 = parts.iter().map(|p| p.0 * p.1).sum::<f64>() / total;
        let mut inertia: f64 = parts.iter().map(|p| p.2 + p.0 * (p.1 - cg).powi(2)).sum();
        inertia += fins_kg * (self.tube.diameter_m / 2.0 + self.fin_radial_cg()).powi(2) / 2.0;
        MassProperties {
            dry_mass_kg: total,
            dry_cg_m: cg,
            pitch_inertia_kg_m2: inertia,
        }
    }

    /// Total CP from the component moment sum (nose + fin set).
    fn total_cp(&self, mach: f64) -> f64 {
        let nose_cna = 2.0;
        let fin_cna = self.fin_set_cna(mach);
        (nose_cna * self.nose.length_m * self.nose.shape.cp_fraction()
            + fin_cna * self.fin_cp_abs())
            / (nose_cna + fin_cna)
    }

    #[cfg(test)]
    fn nose_cp(&self) -> f64 {
        self.nose.length_m * self.nose.shape.cp_fraction()
    }

    fn base_cd(mach: f64) -> f64 {
        if mach <= 1.0 {
            0.12 + 0.13 * mach * mach
        } else {
            0.25 / mach
        }
    }

    fn friction_cd(&self, mach: f64, sound_mps: f64) -> f64 {
        let length = self.total_length_m();
        let velocity = mach * sound_mps;
        let reynolds = velocity * length / 1.5e-5;
        let rough_ratio = self.roughness_m / length;
        let cf_turbulent = if reynolds < 1.0e4 {
            1.48e-2
        } else {
            1.0 / (1.5 * reynolds.ln() - 5.6).powi(2)
        };
        let cf_rough = 0.032 * rough_ratio.powf(0.2);
        // Fully turbulent: take the larger of the smooth and roughness-limited
        // coefficients, then the subsonic compressibility correction.
        let cf = cf_turbulent.max(cf_rough) * (1.0 - 0.1 * mach * mach);
        let r = self.max_diameter_m() / 2.0;
        let fineness = length / r;
        let correction = 1.0 + 1.0 / (2.0 * fineness);
        let body_wet = self
            .nose
            .shape
            .lateral_area(self.nose.length_m, self.nose.diameter_m)
            + std::f64::consts::PI * self.tube.diameter_m * self.tube.length_m;
        let fin_wet = 2.0 * self.fins.count as f64 * self.fin_area();
        let thickness_factor = 1.0 + 2.0 * self.fins.thickness_m / self.mac();
        cf * (correction * body_wet + thickness_factor * fin_wet) / self.ref_area_m2()
    }

    /// Full analysis. Mach samples at 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, and 3.0;
    /// the last point simply repeats the Mach-1 values (subsonic clamp).
    pub fn analyze(&self) -> AeroAnalysis {
        self.validate();
        let machs: [f64; 7] = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 3.0];
        let mut cd_vs_mach = Vec::new();
        let mut cna_vs_mach = Vec::new();
        let mut cp_vs_mach = Vec::new();
        let mut induced_k_vs_mach = Vec::new();
        for &m in &machs {
            let effective = m.min(1.0);
            cd_vs_mach.push((
                m,
                Self::base_cd(effective)
                    + self
                        .nose
                        .shape
                        .pressure_cd(self.nose.length_m, self.nose.diameter_m)
                    + self.friction_cd(effective, 340.0),
            ));
            let fin_cna = self.fin_set_cna(effective);
            cna_vs_mach.push((m, 2.0 + fin_cna));
            cp_vs_mach.push((m, self.total_cp(effective)));
            // Induced drag from the normal force's tilt onto the flight
            // path: Cd_induced = Cnα·α², so k = Cnα (OpenRocket's model).
            induced_k_vs_mach.push((m, 2.0 + fin_cna));
        }
        AeroAnalysis {
            cd_vs_mach,
            cna_vs_mach,
            cp_vs_mach,
            induced_k_vs_mach,
            ref_area_m2: self.ref_area_m2(),
            total_length_m: self.total_length_m(),
            max_diameter_m: self.max_diameter_m(),
        }
    }

    fn validate(&self) {
        assert!(self.nose.length_m > 0.0 && self.tube.length_m > 0.0);
        assert!(self.nose.diameter_m > 0.0 && self.tube.diameter_m > 0.0);
        assert!(self.fins.count > 0);
        assert!(self.fins.span_m > 0.0);
        assert!(self.fins.root_chord_m > 0.0 && self.fins.tip_chord_m > 0.0);
        assert!(self.fins.thickness_m > 0.0);
    }
}

/// Parse the compact geometry spec used by the CLI:
/// `nose=shape:length:diameter,tube=length:diameter,fins=count:root:tip:span:sweep:thickness`.
/// The fin thickness is optional (default 2 mm). Lengths are in metres.
pub fn parse_geometry(spec: &str) -> Result<RocketGeometry> {
    let mut nose = None;
    let mut tube = None;
    let mut fins = None;
    for item in spec.split(',') {
        let (key, val) = item
            .split_once('=')
            .ok_or_else(|| Error::invalid("geometry", "expected key=value items"))?;
        let parts: Vec<&str> = val.split(':').collect();
        let number = |i: usize| -> Result<f64> {
            parts
                .get(i)
                .ok_or_else(|| Error::invalid("geometry", "missing numeric field"))?
                .parse()
                .map_err(|_| Error::invalid("geometry", "non-numeric field"))
        };
        match key {
            "nose" => {
                if parts.len() != 3 {
                    return Err(Error::invalid("nose", "expected shape:length:diameter"));
                }
                let shape = parts[0].parse::<NoseShape>()?;
                nose = Some(NoseCone {
                    shape,
                    length_m: number(1)?,
                    diameter_m: number(2)?,
                });
            }
            "tube" => {
                if parts.len() != 2 {
                    return Err(Error::invalid("tube", "expected length:diameter"));
                }
                tube = Some(BodyTube {
                    length_m: number(0)?,
                    diameter_m: number(1)?,
                });
            }
            "fins" => {
                if !(5..=6).contains(&parts.len()) {
                    return Err(Error::invalid(
                        "fins",
                        "expected count:root:tip:span:sweep[:thickness]",
                    ));
                }
                let count: u32 = parts[0]
                    .parse()
                    .map_err(|_| Error::invalid("fin count", "must be an integer"))?;
                fins = Some(TrapezoidalFins {
                    count,
                    root_chord_m: number(1)?,
                    tip_chord_m: number(2)?,
                    span_m: number(3)?,
                    leading_sweep_m: number(4)?,
                    thickness_m: if parts.len() == 6 { number(5)? } else { 0.002 },
                });
            }
            other => {
                return Err(Error::invalid(
                    "geometry key",
                    match other {
                        "nose" | "tube" | "fins" => unreachable!(),
                        _ => "must be nose, tube or fins",
                    },
                ))
            }
        }
    }
    Ok(RocketGeometry::new(
        nose.ok_or_else(|| Error::invalid("geometry", "nose is required"))?,
        tube.ok_or_else(|| Error::invalid("geometry", "tube is required"))?,
        fins.ok_or_else(|| Error::invalid("geometry", "fins are required"))?,
    ))
}

/// Dry-mass breakdown estimated from geometry and component masses by
/// [`RocketGeometry::mass_properties`].
#[derive(Debug, Clone, Copy)]
pub struct MassProperties {
    /// Sum of the component masses (structure plus motor casing).
    pub dry_mass_kg: f64,
    /// Dry centre of gravity, metres aft of the nose tip.
    pub dry_cg_m: f64,
    /// Pitch inertia about the dry CG, kg·m².
    pub pitch_inertia_kg_m2: f64,
}

/// The curves and reference quantities produced by [`RocketGeometry::analyze`].
#[derive(Debug, Clone)]
pub struct AeroAnalysis {
    /// Axial drag coefficient vs Mach (subsonic, clamped above Mach 1).
    pub cd_vs_mach: Vec<(f64, f64)>,
    /// Normal-force coefficient slope per radian vs Mach (nose + fin set).
    pub cna_vs_mach: Vec<(f64, f64)>,
    /// Centre of pressure, metres aft of the nose tip, vs Mach.
    pub cp_vs_mach: Vec<(f64, f64)>,
    /// Induced-drag factor per radian² vs Mach (normal-force tilt: k = Cnα).
    pub induced_k_vs_mach: Vec<(f64, f64)>,
    /// Reference area, π/4·d² of the largest diameter.
    pub ref_area_m2: f64,
    /// Total aerodynamic length (nose + tube).
    pub total_length_m: f64,
    /// Largest body diameter.
    pub max_diameter_m: f64,
}

#[cfg(feature = "ballistics")]
impl AeroAnalysis {
    /// Bundle the analysis with the remaining airframe parameters.
    pub fn airframe(
        &self,
        cg_dry_m: f64,
        cg_prop_m: f64,
        pitch_inertia_kg_m2: f64,
        recovery_cd_area_m2: f64,
    ) -> crate::Airframe {
        crate::Airframe {
            diameter_m: self.max_diameter_m,
            cd_vs_mach: self.cd_vs_mach.clone(),
            cna_vs_mach: self.cna_vs_mach.clone(),
            cp_vs_mach: self.cp_vs_mach.clone(),
            cg_dry_m,
            cg_prop_m,
            pitch_inertia_kg_m2,
            induced_k_vs_mach: self.induced_k_vs_mach.clone(),
            recovery_cd_area_m2,
        }
    }

    /// Drag + atmosphere + body lift for this geometry in the given wind.
    pub fn forces(&self, wind_mps: crate::Vec3) -> Result<crate::ballistics::BallisticForces> {
        crate::ballistics::BallisticForces::new(
            self.max_diameter_m,
            self.cd_vs_mach.clone(),
            crate::ballistics::Atmosphere::default(),
            wind_mps,
        )?
        .with_body_lift(self.cna_vs_mach.clone(), self.induced_k_vs_mach.clone())
    }

    /// Weathercock parameters for this geometry.
    pub fn weathercock_params(
        &self,
        cg_dry_m: f64,
        cg_prop_m: f64,
        pitch_inertia_kg_m2: f64,
    ) -> Result<crate::WeathercockParams> {
        crate::WeathercockParams::new(
            self.cna_vs_mach.clone(),
            self.cp_vs_mach.clone(),
            cg_dry_m,
            cg_prop_m,
            self.max_diameter_m,
            pitch_inertia_kg_m2,
            0.2,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example() -> RocketGeometry {
        RocketGeometry::new(
            NoseCone {
                shape: NoseShape::Conical,
                length_m: 0.1,
                diameter_m: 0.024,
            },
            BodyTube {
                length_m: 0.4,
                diameter_m: 0.024,
            },
            TrapezoidalFins {
                count: 3,
                root_chord_m: 0.05,
                tip_chord_m: 0.03,
                span_m: 0.04,
                leading_sweep_m: 0.02,
                thickness_m: 0.002,
            },
        )
    }

    #[test]
    fn nose_cp_fractions_match_barrowman() {
        assert_eq!(NoseShape::Conical.cp_fraction(), 2.0 / 3.0);
        assert_eq!(NoseShape::Ogive.cp_fraction(), 0.466);
        assert_eq!(NoseShape::Elliptical.cp_fraction(), 1.0 / 3.0);
        assert_eq!(NoseShape::Parabolic.cp_fraction(), 0.5);
    }

    #[test]
    fn conical_pressure_drag_uses_half_angle() {
        let cd = NoseShape::Conical.pressure_cd(0.1, 0.024);
        let sin_phi = 0.012 / (0.012_f64.powi(2) + 0.1_f64.powi(2)).sqrt();
        assert!((cd - 0.8 * sin_phi * sin_phi).abs() < 1e-12);
        assert_eq!(NoseShape::Ogive.pressure_cd(0.1, 0.024), 0.0);
    }

    #[test]
    fn fin_cp_matches_the_trapezoid_formula() {
        let geo = example();
        // Independently: MAC = (2/3)(cr+ct - cr*ct/(cr+ct)), yMAC = s(cr+2ct)/(3(cr+ct)).
        let mac = (2.0 / 3.0) * (0.05 + 0.03 - 0.05 * 0.03 / 0.08);
        let y_mac = (0.04 / 3.0) * (0.05 + 2.0 * 0.03) / 0.08;
        let expected = 0.02 * y_mac / 0.04 + 0.25 * mac;
        assert!((geo.fin_cp() - expected).abs() < 1e-12, "{}", geo.fin_cp());
    }

    #[test]
    fn conical_nose_volume_properties_match_the_solid_cone() {
        let geo = example(); // conical nose, L = 0.1, R = 0.012
        let (cg, inertia) = geo.nose_volume_properties();
        assert!((cg - 0.75 * 0.1).abs() < 1e-6, "cg {cg}");
        let expected = 3.0 * 0.012_f64.powi(2) / 20.0 + 3.0 * 0.1_f64.powi(2) / 80.0;
        assert!(
            (inertia - expected).abs() / expected < 1e-4,
            "inertia {inertia} expected {expected}"
        );
    }

    #[test]
    fn fin_centroid_matches_polygon_limits() {
        let geo = example();
        // Rectangle limit (m = 0, ct = cr): centroid at cr/2, height s/2.
        let rectangular = RocketGeometry::new(
            geo.nose,
            geo.tube,
            TrapezoidalFins {
                count: 3,
                root_chord_m: 0.05,
                tip_chord_m: 0.05,
                span_m: 0.04,
                leading_sweep_m: 0.0,
                thickness_m: 0.002,
            },
        );
        assert!((rectangular.fin_planform_cg() - 0.025).abs() < 1e-12);
        assert!((rectangular.fin_radial_cg() - 0.02).abs() < 1e-12);
        // Triangle limit (ct = 0): (cr + m)/3 from the leading edge, s/3 up.
        let triangular = RocketGeometry::new(
            geo.nose,
            geo.tube,
            TrapezoidalFins {
                count: 3,
                root_chord_m: 0.06,
                tip_chord_m: 0.0,
                span_m: 0.03,
                leading_sweep_m: 0.03,
                thickness_m: 0.002,
            },
        );
        assert!((triangular.fin_planform_cg() - (0.06 + 0.03) / 3.0).abs() < 1e-12);
        assert!((triangular.fin_radial_cg() - 0.01).abs() < 1e-12);
    }

    #[test]
    fn mass_properties_are_consistent() {
        let geo = example();
        let props = geo.mass_properties(0.012, 0.020, 0.006, None);
        assert!((props.dry_mass_kg - 0.038).abs() < 1e-12);
        // CG is the mass-weighted component mean, inside the airframe.
        let (nose_cg, _) = geo.nose_volume_properties();
        let tube_cg = geo.nose.length_m + geo.tube.length_m / 2.0;
        let fin_cg = geo.total_length_m() - geo.fins.root_chord_m + geo.fin_planform_cg();
        let expected_cg = (0.012 * nose_cg + 0.020 * tube_cg + 0.006 * fin_cg) / 0.038;
        assert!((props.dry_cg_m - expected_cg).abs() < 1e-12);
        assert!(props.dry_cg_m > 0.0 && props.dry_cg_m < geo.total_length_m());
        assert!(props.pitch_inertia_kg_m2 > 0.0);
        // A casing at the aft end moves the CG aft and adds inertia.
        let with_casing = geo.mass_properties(0.012, 0.020, 0.006, Some((0.02, 0.32)));
        assert!(with_casing.dry_cg_m > props.dry_cg_m);
        assert!(with_casing.pitch_inertia_kg_m2 > props.pitch_inertia_kg_m2);
        assert!((with_casing.dry_mass_kg - 0.058).abs() < 1e-12);
        // A heavier nose moves the CG forward.
        let heavy_nose = geo.mass_properties(0.024, 0.020, 0.006, None);
        assert!(heavy_nose.dry_cg_m < props.dry_cg_m);
    }

    #[test]
    fn induced_k_tracks_cna() {
        // Tilt model: the induced-drag factor equals the normal-force slope.
        let geo = example();
        let analysis = geo.analyze();
        for ((m, k), (_, cna)) in analysis.induced_k_vs_mach.iter().zip(&analysis.cna_vs_mach) {
            assert_eq!(k, cna, "mach {m}");
        }
    }

    #[test]
    fn mass_properties_single_component_is_exact() {
        let geo = example();
        let props = geo.mass_properties(0.0, 0.05, 0.0, None);
        // Only the tube: CG at its midpoint, inertia its thin-cylinder value.
        let tube_cg = geo.nose.length_m + geo.tube.length_m / 2.0;
        assert!((props.dry_cg_m - tube_cg).abs() < 1e-12);
        let expected =
            0.05 * (geo.tube.length_m.powi(2) / 12.0 + geo.tube.diameter_m.powi(2) / 8.0);
        assert!(
            (props.pitch_inertia_kg_m2 - expected).abs() / expected < 1e-12,
            "{} vs {expected}",
            props.pitch_inertia_kg_m2
        );
    }

    #[test]
    fn fin_set_cna_matches_diederich() {
        let geo = example();
        // Fin pair: 2π·span²/(1+sqrt(1+(span²/(A·cosΓ))²)) / Aref, then
        // (N/2)·(1+tau) — the Diederich value is a wing (two fins).
        let area = geo.fin_area();
        let cos = geo.midchord_cos();
        let ratio = geo.fins.span_m * geo.fins.span_m / (area * cos);
        let one = 2.0 * std::f64::consts::PI * geo.fins.span_m * geo.fins.span_m
            / (1.0 + (1.0 + ratio * ratio).sqrt())
            / geo.ref_area_m2();
        let expected = 1.5 * one * (1.0 + geo.interference_tau());
        let actual = geo.fin_set_cna(0.0);
        assert!(
            (actual - expected).abs() / expected < 1e-9,
            "actual {actual} expected {expected}"
        );
        // And the classic closed form 4N(s/d)²/(1+sqrt(1+(2l/(cr+ct))²))·K.
        let d = geo.max_diameter_m();
        let classic = 4.0 * 3.0 * (geo.fins.span_m / d).powi(2)
            / (1.0 + (1.0_f64 + 1.0_f64).sqrt())
            * (1.0 + geo.interference_tau());
        assert!(
            (actual - classic).abs() / classic < 0.02,
            "actual {actual} classic {classic}"
        );
    }

    #[test]
    fn total_cp_is_the_moment_sum() {
        let geo = example();
        let fin_cna = geo.fin_set_cna(0.0);
        let expected = (2.0 * geo.nose_cp() + fin_cna * geo.fin_cp_abs()) / (2.0 + fin_cna);
        assert!((geo.total_cp(0.0) - expected).abs() < 1e-12);
        // CP must sit aft of the nose CP (fins pull it back).
        assert!(geo.total_cp(0.0) > geo.nose_cp());
    }

    #[test]
    fn base_cd_matches_openrocket() {
        assert_eq!(RocketGeometry::base_cd(0.0), 0.12);
        assert_eq!(RocketGeometry::base_cd(1.0), 0.25);
        assert_eq!(RocketGeometry::base_cd(2.0), 0.125);
    }

    #[test]
    fn analysis_produces_finite_curves_and_roughness_costs_drag() {
        let geo = example();
        let analysis = geo.analyze();
        for &(m, cd) in &analysis.cd_vs_mach {
            assert!(cd.is_finite() && cd > 0.0, "mach {m} cd {cd}");
        }
        for &(m, cp) in &analysis.cp_vs_mach {
            assert!(cp.is_finite() && cp > 0.0, "mach {m} cp {cp}");
        }
        let smooth = geo.with_roughness(1e-6).analyze();
        let rough = geo.with_roughness(500e-6).analyze();
        assert!(rough.cd_vs_mach[2].1 > smooth.cd_vs_mach[2].1);
    }
}
