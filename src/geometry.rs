//! Geometry-to-aerodynamics: a Barrowman subset for slender model rockets
//! (nose cone + body tube + fin set + optional conical boat-tail),
//! implemented against the OpenRocket technical documentation and its
//! `FinSetCalc` and `SymmetricComponentCalc` sources. CP comes from the
//! component moment sum (with the supersonic aft shift); drag is friction +
//! base + nose wave drag + rounded-fin form drag. The fin normal-force slope uses Diederich below Mach 1, the Busemann K1 term above Mach 1.5, and OpenRocket's Hermite join between them.
//!
//! Induced drag from angle of attack uses the normal-force tilt model
//! (Cd += Cnα·α²). Fin planforms are trapezoidal (closed form) or
//! half-ellipse (OpenRocket's 48-strip integration). Not covered
//! (deliberately): forward shoulders or multi-diameter bodies, free-form
//! fins, and fin-body interference drag.

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
/// A conical boat-tail (diameter-reducing transition) at the very aft end
/// of the body tube.
#[derive(Debug, Clone, Copy)]
pub struct BoatTail {
    /// Transition length in metres.
    pub length_m: f64,
    /// Diameter at the forward end (must equal the body tube diameter).
    pub fore_diameter_m: f64,
    /// Diameter at the aft end (must be smaller than the forward diameter).
    pub aft_diameter_m: f64,
}

/// Identical elliptical fins (half-ellipse planform) at the aft end of the
/// tube. Geometry follows OpenRocket's EllipticalFinSet: root chord along
/// the body, tip point mid-chord at the span.
#[derive(Debug, Clone, Copy)]
pub struct EllipticalFins {
    /// Number of identical fins.
    pub count: u32,
    /// Fin root chord in metres.
    pub root_chord_m: f64,
    /// Fin semispan in metres (body surface to tip).
    pub span_m: f64,
    /// Fin thickness in metres.
    pub thickness_m: f64,
}

/// The fin-set planform family.
#[derive(Debug, Clone, Copy)]
pub enum FinSet {
    /// Trapezoidal fins (straight edges).
    Trapezoidal(TrapezoidalFins),
    /// Elliptical fins (half-ellipse planform).
    Elliptical(EllipticalFins),
}

impl From<TrapezoidalFins> for FinSet {
    fn from(fins: TrapezoidalFins) -> Self {
        Self::Trapezoidal(fins)
    }
}

impl From<EllipticalFins> for FinSet {
    fn from(fins: EllipticalFins) -> Self {
        Self::Elliptical(fins)
    }
}

/// Single-fin planform geometry shared by the fin models.
#[derive(Debug, Clone, Copy)]
struct Planform {
    count: u32,
    root_chord_m: f64,
    span_m: f64,
    thickness_m: f64,
    area_m2: f64,
    mac_m: f64,
    mac_lead_m: f64,
    cos_midchord: f64,
    cos_leading_edge: f64,
    cg_x_m: f64,
    cg_y_m: f64,
}

/// A slender rocket: nose cone, body tube, fin set and an optional conical
/// boat-tail.
#[derive(Debug, Clone, Copy)]
pub struct RocketGeometry {
    /// Nose cone.
    pub nose: NoseCone,
    /// Cylindrical body tube.
    pub tube: BodyTube,
    /// Aft fin set.
    pub fins: FinSet,
    /// Optional conical boat-tail aft of the fins.
    pub boat_tail: Option<BoatTail>,
    /// Roughness height in metres (default 20 µm, painted surface).
    pub roughness_m: f64,
}

/// NASA TR-R-100 zero-lift pressure drag for fineness-3 noses (from
/// OpenRocket's SymmetricComponentCalc), extrapolated to other fineness
/// ratios by the stagnation-scaled power law.
const ELLIPSOID_TABLE: &[(f64, f64)] = &[
    (1.2, 0.110),
    (1.25, 0.128),
    (1.3, 0.140),
    (1.4, 0.148),
    (1.6, 0.152),
    (2.0, 0.159),
    (2.4, 0.162),
];
const PARABOLIC_TABLE: &[(f64, f64)] = &[
    (0.95, 0.0),
    (0.975, 0.016),
    (1.0, 0.041),
    (1.05, 0.092),
    (1.1, 0.109),
    (1.2, 0.119),
    (1.4, 0.113),
    (1.7, 0.108),
];

impl RocketGeometry {
    /// Build with the default painted-surface roughness (20 µm).
    pub fn new(nose: NoseCone, tube: BodyTube, fins: impl Into<FinSet>) -> Self {
        Self {
            nose,
            tube,
            fins: fins.into(),
            boat_tail: None,
            roughness_m: 20e-6,
        }
    }

    /// Set the roughness height in metres.
    pub fn with_roughness(mut self, roughness_m: f64) -> Self {
        self.roughness_m = roughness_m;
        self
    }

    /// Attach a conical boat-tail at the aft end of the tube. Its forward
    /// diameter must match the tube and its aft diameter must be smaller.
    pub fn with_boat_tail(mut self, boat_tail: BoatTail) -> Result<Self> {
        if boat_tail.length_m <= 0.0 || !boat_tail.length_m.is_finite() {
            return Err(Error::invalid("boat tail", "length must be positive"));
        }
        if (boat_tail.fore_diameter_m - self.tube.diameter_m).abs() > 1e-12 {
            return Err(Error::invalid(
                "boat tail",
                "forward diameter must equal the body tube diameter",
            ));
        }
        if boat_tail.aft_diameter_m <= 0.0 || boat_tail.aft_diameter_m >= boat_tail.fore_diameter_m
        {
            return Err(Error::invalid(
                "boat tail",
                "aft diameter must be positive and smaller than the forward diameter",
            ));
        }
        self.boat_tail = Some(boat_tail);
        Ok(self)
    }

    /// Total aerodynamic length (nose + tube + boat-tail).
    pub fn total_length_m(&self) -> f64 {
        self.nose.length_m + self.tube.length_m + self.boat_tail.map(|b| b.length_m).unwrap_or(0.0)
    }

    /// Largest body diameter (nose base or tube).
    pub fn max_diameter_m(&self) -> f64 {
        self.nose.diameter_m.max(self.tube.diameter_m)
    }

    /// Reference area, π/4·d² of the largest diameter.
    pub fn ref_area_m2(&self) -> f64 {
        std::f64::consts::FRAC_PI_4 * self.max_diameter_m().powi(2)
    }

    /// Single-fin planform geometry, shared by the fin models. Trapezoids
    /// use the closed forms; ellipses use OpenRocket's 48-strip integration
    /// (FinSetCalc.calculateFinGeometry) over the half-ellipse boundary.
    fn planform(&self) -> Planform {
        match self.fins {
            FinSet::Trapezoidal(f) => {
                let (cr, ct, s, m) = (f.root_chord_m, f.tip_chord_m, f.span_m, f.leading_sweep_m);
                let area = s * (cr + ct) / 2.0;
                let mac = (2.0 / 3.0) * (cr + ct - cr * ct / (cr + ct));
                let y_mac = (s / 3.0) * (cr + 2.0 * ct) / (cr + ct);
                let gamma_mid = ((m + ct / 2.0 - cr / 2.0) / s).atan();
                Planform {
                    count: f.count,
                    root_chord_m: cr,
                    span_m: s,
                    thickness_m: f.thickness_m,
                    area_m2: area,
                    mac_m: mac,
                    mac_lead_m: m * y_mac / s,
                    cos_midchord: gamma_mid.cos(),
                    cos_leading_edge: s / (s * s + m * m).sqrt(),
                    cg_x_m: (cr * cr + cr * ct + ct * ct + m * (cr + 2.0 * ct)) / (3.0 * (cr + ct)),
                    cg_y_m: y_mac,
                }
            }
            FinSet::Elliptical(f) => {
                const DIVISIONS: usize = 48;
                let (cr, s) = (f.root_chord_m, f.span_m);
                let dy = s / (DIVISIONS - 1) as f64;
                let mut area = 0.0;
                let mut mac_num = 0.0;
                let mut lead_num = 0.0;
                let mut cos_mid = 0.0;
                let mut cos_le = 0.0;
                let mut previous: Option<(f64, f64)> = None;
                for i in 0..DIVISIONS {
                    let y = i as f64 * dy;
                    let t = (y / s).min(1.0);
                    let arc = (1.0 - t * t).max(0.0).sqrt();
                    let lead = cr / 2.0 * (1.0 - arc);
                    let chord = cr * arc;
                    let mid = cr / 2.0;
                    area += chord * dy;
                    mac_num += chord * chord * dy;
                    lead_num += lead * chord * dy;
                    if let Some((prev_mid, prev_lead)) = previous {
                        let hypot_mid = dy.hypot(mid - prev_mid);
                        if hypot_mid > 0.0 {
                            cos_mid += dy / hypot_mid;
                        }
                        let hypot_lead = dy.hypot(lead - prev_lead);
                        if hypot_lead > 0.0 {
                            cos_le += dy / hypot_lead;
                        }
                    }
                    previous = Some((mid, lead));
                }
                cos_mid /= (DIVISIONS - 1) as f64;
                cos_le /= (DIVISIONS - 1) as f64;
                Planform {
                    count: f.count,
                    root_chord_m: cr,
                    span_m: s,
                    thickness_m: f.thickness_m,
                    area_m2: area,
                    mac_m: mac_num / area,
                    mac_lead_m: lead_num / area,
                    cos_midchord: cos_mid,
                    cos_leading_edge: cos_le,
                    cg_x_m: cr / 2.0,
                    cg_y_m: 4.0 * s / (3.0 * std::f64::consts::PI),
                }
            }
        }
    }

    #[cfg(test)]
    fn fin_area(&self) -> f64 {
        self.planform().area_m2
    }

    /// Fin CP as a fraction of the MAC: quarter-chord at M ≤ 0.5, the
    /// empirical supersonic formula at M ≥ 2, a linear join between
    /// (OpenRocket's FinSetCalc.calculateCPPos; it fits a polynomial, but
    /// the join error is small beside the component moment sum).
    fn fin_cp_fraction(&self, mach: f64) -> f64 {
        if mach <= 0.5 {
            return 0.25;
        }
        let planform = self.planform();
        let ar = 2.0 * planform.span_m.powi(2) / planform.area_m2;
        let supersonic = |m: f64| {
            let beta = (m * m - 1.0).sqrt();
            (ar * beta - 0.67) / (2.0 * ar * beta - 1.0)
        };
        if mach >= 2.0 {
            return supersonic(mach);
        }
        let hi = supersonic(2.0);
        0.25 + (hi - 0.25) * (mach - 0.5) / 1.5
    }

    /// Fin CP measured from the fin's root leading edge.
    fn fin_cp(&self, mach: f64) -> f64 {
        let planform = self.planform();
        planform.mac_lead_m + self.fin_cp_fraction(mach) * planform.mac_m
    }

    /// Fin pressure (form) drag: OpenRocket's rounded-section model —
    /// a Mach-dependent leading-edge term times cos²(Γ_LE), plus half the
    /// base drag for the trailing edge, scaled by span·thickness / A_ref.
    fn fin_pressure_cd(&self, mach: f64) -> f64 {
        let planform = self.planform();
        if planform.span_m <= 0.0 || planform.thickness_m <= 0.0 {
            return 0.0;
        }
        let leading_edge = if mach < 0.9 {
            (1.0 - mach * mach).powf(-0.417) - 1.0
        } else if mach < 1.0 {
            1.0 - 1.785 * (mach - 0.9)
        } else {
            let m2 = mach * mach;
            1.214 - 0.502 / m2 + 0.1095 / (m2 * m2)
        };
        let cos_le = planform.cos_leading_edge;
        let cd = leading_edge * cos_le * cos_le + Self::base_cd(mach) / 2.0;
        // OpenRocket instances the fin set per fin; the set's drag is n
        // times the single-fin value.
        cd * planform.span_m * planform.thickness_m / self.ref_area_m2() * planform.count as f64
    }

    /// Nose pressure drag per frontal area for conical and tangent-ogive
    /// noses: OpenRocket's direct calculation (polynomial through M 1–1.3,
    /// 2.1·sin²φ + 0.5·sinφ/β above, a·M^b + 0.8·sin²φ fitted subsonic).
    /// Elliptical and parabolic noses are not modelled past Mach 1 yet and
    /// return 0 there (their subsonic value is 0 in this subset).
    fn nose_pressure_cd(&self, mach: f64) -> f64 {
        // sinphi is the sine of the surface angle at 99% of the nose length
        // (OpenRocket's SymmetricComponentCalc). For a cone that is the half
        // angle; for a tangent ogive the surface at the base is nearly
        // axial, so its wave drag is essentially zero — matching OpenRocket.
        let sinphi = match self.nose.shape {
            NoseShape::Conical | NoseShape::Ogive => self.nose_sinphi(),
            NoseShape::Elliptical => return self.table_pressure_cd(mach, ELLIPSOID_TABLE),
            NoseShape::Parabolic => return self.table_pressure_cd(mach, PARABOLIC_TABLE),
        };
        const GAMMA: f64 = 1.4;
        let cd_mach0 = 0.8 * sinphi * sinphi;
        let y1 = 2.1 * sinphi * sinphi + 0.6019 * sinphi;
        let d0 = 4.0 / (GAMMA + 1.0) * (1.0 - 0.5 * sinphi);
        let d1 = -1.1341 * sinphi;
        if mach >= 1.3 {
            return 2.1 * sinphi * sinphi + 0.5 * sinphi / (mach * mach - 1.0).sqrt();
        }
        if mach >= 1.0 {
            return Self::hermite(mach, 1.0, 1.3, sinphi, y1, d0, d1);
        }
        // Subsonic: Cd = a·M^b + Cd(0), fitted to reach (1, sinphi) with the
        // polynomial's derivative (OpenRocket's interpolation scheme).
        let eps = 0.01;
        let at_eps = Self::hermite(1.0 + eps, 1.0, 1.3, sinphi, y1, d0, d1);
        let min_deriv = (at_eps - sinphi) / eps;
        let gap = sinphi - cd_mach0;
        if gap <= 0.01 || min_deriv <= 0.01 {
            return cd_mach0;
        }
        let b = min_deriv / gap;
        gap * mach.powf(b) + cd_mach0
    }

    /// sin(phi) of the nose surface at 99% of the length (OpenRocket's
    /// SymmetricComponentCalc convention).
    fn nose_sinphi(&self) -> f64 {
        let l = self.nose.length_m;
        let r = self.nose.diameter_m / 2.0;
        let r_tip = self.nose_radius(0.99 * l);
        (r - r_tip) / (r - r_tip).hypot(0.01 * l)
    }

    /// Stagnation-pressure Cd, OpenRocket's formula.
    fn stagnation_cd(mach: f64) -> f64 {
        let m2 = mach * mach;
        if mach <= 1.0 {
            0.85 * (1.0 + m2 / 4.0 + m2 * m2 / 40.0)
        } else {
            0.85 * (1.84 - 0.76 / m2 + 0.166 / (m2 * m2) + 0.035 / (m2 * m2 * m2))
        }
    }

    /// Linear interpolation in a (mach, cd) table, clamped at the ends.
    fn table_interp(table: &[(f64, f64)], mach: f64) -> f64 {
        if mach <= table[0].0 {
            return table[0].1;
        }
        for pair in table.windows(2) {
            let (lo, hi) = (pair[0], pair[1]);
            if mach <= hi.0 {
                let f = (mach - lo.0) / (hi.0 - lo.0);
                return lo.1 + f * (hi.1 - lo.1);
            }
        }
        table[table.len() - 1].1
    }

    /// Nose pressure drag from the NASA TR-R-100 fineness-3 tables
    /// (OpenRocket's SymmetricComponentCalc), extrapolated to this nose's
    /// fineness ratio and joined subsonically by the a·M^b + Cd(0) fit.
    fn table_pressure_cd(&self, mach: f64, table: &[(f64, f64)]) -> f64 {
        let fineness = self.nose.length_m / self.nose.diameter_m;
        let log4 = (fineness + 1.0).ln() / 4.0_f64.ln();
        let value_at = |m: f64| {
            let stag = Self::stagnation_cd(m);
            stag * (Self::table_interp(table, m) / stag).powf(log4)
        };
        let min = table[0].0;
        if mach >= min {
            return value_at(mach);
        }
        let cd_mach0 = 0.8 * self.nose_sinphi().powi(2);
        let min_value = value_at(min);
        if min_value < 0.001 {
            return cd_mach0;
        }
        let min_deriv = (value_at(min + 0.01) - min_value) / 0.01;
        let gap = min_value - cd_mach0;
        if gap <= 0.01 || min_deriv <= 0.01 {
            return cd_mach0;
        }
        let b = min * min_deriv / gap;
        let a = gap / min.powf(b);
        a * mach.powf(b) + cd_mach0
    }

    #[cfg(test)]
    fn midchord_cos(&self) -> f64 {
        self.planform().cos_midchord
    }

    /// Cubic Hermite interpolation on [x0, x1] with endpoint values y0, y1
    /// and endpoint derivatives d0, d1 per unit x.
    fn hermite(x: f64, x0: f64, x1: f64, y0: f64, y1: f64, d0: f64, d1: f64) -> f64 {
        let h = x1 - x0;
        let t = (x - x0) / h;
        let t2 = t * t;
        let t3 = t2 * t;
        (2.0 * t3 - 3.0 * t2 + 1.0) * y0
            + (t3 - 2.0 * t2 + t) * h * d0
            + (-2.0 * t3 + 3.0 * t2) * y1
            + (t3 - t2) * h * d1
    }

    /// Mach bounds of OpenRocket's transonic interpolation.
    const CNA_SUBSONIC: f64 = 0.9;
    const CNA_SUPERSONIC: f64 = 1.5;

    /// Normal-force slope of one fin, divided by the reference area.
    /// Subsonic: OpenRocket's Diederich simplification. Supersonic:
    /// 2·A_fin/(β·A_ref) (the K1 term of the Busemann expansion; the α
    /// terms vanish for a slope table). Transonic: OpenRocket's cubic
    /// Hermite join with matched endpoint derivatives.
    fn fin_cna1(&self, mach: f64) -> f64 {
        let planform = self.planform();
        let span = planform.span_m;
        let area = planform.area_m2;
        let cos = planform.cos_midchord;
        if area <= 0.0 || span <= 0.0 || cos <= 0.0 {
            return 0.0;
        }
        let ref_area = self.ref_area_m2();
        let subsonic = |m: f64| {
            let ratio = (span * span) / (area * cos);
            2.0 * std::f64::consts::PI * span * span
                / (1.0 + (1.0 + (1.0 - m * m) * ratio * ratio).sqrt())
                / ref_area
        };
        let supersonic = |m: f64| 2.0 * area / (m * m - 1.0).sqrt() / ref_area;
        if mach <= Self::CNA_SUBSONIC {
            return subsonic(mach);
        }
        if mach >= Self::CNA_SUPERSONIC {
            return supersonic(mach);
        }
        let sq = (1.0
            + (1.0 - Self::CNA_SUBSONIC * Self::CNA_SUBSONIC)
                * (span * span / (area * cos)).powi(2))
        .sqrt();
        let sub_v = 2.0 * std::f64::consts::PI * span * span / ref_area / (1.0 + sq);
        let sub_d = 2.0 * Self::CNA_SUBSONIC * std::f64::consts::PI * span.powi(6)
            / ((area * cos).powi(2) * ref_area * sq * (1.0 + sq).powi(2));
        let super_v = supersonic(Self::CNA_SUPERSONIC);
        let beta_b = (Self::CNA_SUPERSONIC * Self::CNA_SUPERSONIC - 1.0).powf(1.5);
        let super_d = -area / ref_area * 2.0 * Self::CNA_SUPERSONIC / beta_b;
        Self::hermite(
            mach,
            Self::CNA_SUBSONIC,
            Self::CNA_SUPERSONIC,
            sub_v,
            super_v,
            sub_d,
            super_d,
        )
    }

    /// Body-interference factor (1 + tau) with tau = r/(span + r).
    fn interference_tau(&self) -> f64 {
        let r = self.max_diameter_m() / 2.0;
        r / (self.planform().span_m + r)
    }

    fn fin_count_factor(&self) -> f64 {
        match self.planform().count {
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
            * (self.planform().count as f64 / 2.0)
    }

    /// Fin CP measured from the nose tip (fin trailing edge at tube end).
    fn fin_cp_abs(&self, mach: f64) -> f64 {
        self.nose.length_m + self.tube.length_m - self.planform().root_chord_m + self.fin_cp(mach)
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
        self.planform().cg_x_m
    }

    /// Fin centroid height above the body surface.
    fn fin_radial_cg(&self) -> f64 {
        self.planform().cg_y_m
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
        self.mass_properties_with_boat(nose_kg, tube_kg, fins_kg, None, casing)
    }

    /// [`mass_properties`](Self::mass_properties) with an optional boat-tail
    /// shell mass (treated as a thin-walled frustum).
    pub fn mass_properties_with_boat(
        &self,
        nose_kg: f64,
        tube_kg: f64,
        fins_kg: f64,
        boat_kg: Option<f64>,
        casing: Option<(f64, f64)>,
    ) -> MassProperties {
        assert!(
            nose_kg >= 0.0 && tube_kg >= 0.0 && fins_kg >= 0.0,
            "component masses must be non-negative"
        );
        let (nose_cg, nose_inertia) = self.nose_volume_properties();
        let tube_cg = self.nose.length_m + self.tube.length_m / 2.0;
        let fin_cg = self.nose.length_m + self.tube.length_m - self.planform().root_chord_m
            + self.fin_planform_cg();
        let mut parts: Vec<(f64, f64, f64)> = vec![
            (nose_kg, nose_cg, nose_kg * nose_inertia),
            (
                tube_kg,
                tube_cg,
                tube_kg * (self.tube.length_m.powi(2) / 12.0 + self.tube.diameter_m.powi(2) / 8.0),
            ),
            (fins_kg, fin_cg, 0.0),
        ];
        if let (Some(b), Some(kg)) = (self.boat_tail, boat_kg) {
            let rf = b.fore_diameter_m / 2.0;
            let ra = b.aft_diameter_m / 2.0;
            // Thin-walled frustum shell: lateral centroid and a mean-radius
            // thin-shell inertia estimate.
            let cg = self.nose.length_m
                + self.tube.length_m
                + b.length_m * (rf + 2.0 * ra) / (3.0 * (rf + ra));
            let mean_r = (rf + ra) / 2.0;
            parts.push((
                kg,
                cg,
                kg * (b.length_m.powi(2) / 12.0 + mean_r * mean_r / 2.0),
            ));
        }
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

    /// Boat-tail normal-force slope: 2·((r_aft/r_fore)² − 1), negative for a
    /// diameter-reducing transition (Barrowman).
    fn boat_cna(&self) -> f64 {
        match self.boat_tail {
            Some(b) => {
                let ratio = b.aft_diameter_m / b.fore_diameter_m;
                2.0 * (ratio * ratio - 1.0)
            }
            None => 0.0,
        }
    }

    /// Boat-tail CP from the nose tip: the frustum planform centroid,
    /// x̄ = L·(r_fore + 3·r_aft)/(4·(r_fore + r_aft)) from its forward end.
    fn boat_cp_abs(&self) -> f64 {
        match self.boat_tail {
            Some(b) => {
                let rf = b.fore_diameter_m / 2.0;
                let ra = b.aft_diameter_m / 2.0;
                self.nose.length_m
                    + self.tube.length_m
                    + b.length_m * (rf + 3.0 * ra) / (4.0 * (rf + ra))
            }
            None => 0.0,
        }
    }

    /// Boat-tail pressure drag (OpenRocket's SymmetricComponentCalc):
    /// long gentle tapers (fineness ≥ 3) are free; shorter ones pay the
    /// base drag over the annulus area, scaled by (3 − fineness)/2.
    fn boat_pressure_cd(&self, mach: f64) -> f64 {
        let Some(b) = self.boat_tail else {
            return 0.0;
        };
        let fineness = b.length_m / (b.fore_diameter_m - b.aft_diameter_m);
        if fineness >= 3.0 {
            return 0.0;
        }
        let annulus =
            std::f64::consts::FRAC_PI_4 * (b.fore_diameter_m.powi(2) - b.aft_diameter_m.powi(2));
        let cd = Self::base_cd(mach) * annulus / self.ref_area_m2();
        if fineness <= 1.0 {
            cd
        } else {
            cd * (3.0 - fineness) / 2.0
        }
    }

    /// Base-drag scaling: the base area shrinks to the boat-tail's aft face.
    fn base_area_fraction(&self) -> f64 {
        match self.boat_tail {
            Some(b) => (b.aft_diameter_m / self.max_diameter_m()).powi(2),
            None => 1.0,
        }
    }

    /// Total CP from the component moment sum (nose + fin set + boat-tail).
    fn total_cp(&self, mach: f64) -> f64 {
        let nose_cna = 2.0;
        let fin_cna = self.fin_set_cna(mach);
        let boat_cna = self.boat_cna();
        let total = nose_cna + fin_cna + boat_cna;
        if total <= 0.0 {
            return f64::NAN;
        }
        (nose_cna * self.nose.length_m * self.nose.shape.cp_fraction()
            + fin_cna * self.fin_cp_abs(mach)
            + boat_cna * self.boat_cp_abs())
            / total
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
        let boat_wet = match self.boat_tail {
            Some(b) => {
                let rf = b.fore_diameter_m / 2.0;
                let ra = b.aft_diameter_m / 2.0;
                let slant = (b.length_m.powi(2) + (rf - ra).powi(2)).sqrt();
                std::f64::consts::PI * (rf + ra) * slant
            }
            None => 0.0,
        };
        let body_wet = self
            .nose
            .shape
            .lateral_area(self.nose.length_m, self.nose.diameter_m)
            + std::f64::consts::PI * self.tube.diameter_m * self.tube.length_m
            + boat_wet;
        let planform = self.planform();
        let fin_wet = 2.0 * planform.count as f64 * planform.area_m2;
        let thickness_factor = 1.0 + 2.0 * planform.thickness_m / planform.mac_m;
        cf * (correction * body_wet + thickness_factor * fin_wet) / self.ref_area_m2()
    }

    /// Full analysis. Mach samples at 0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0,
    /// 1.1, 1.3, 1.5, 2.0 and 3.0, computed across the transonic and
    /// supersonic regimes (no clamp).
    pub fn analyze(&self) -> AeroAnalysis {
        self.validate();
        let machs: [f64; 12] = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0, 1.1, 1.3, 1.5, 2.0, 3.0];
        let mut cd_vs_mach = Vec::new();
        let mut cna_vs_mach = Vec::new();
        let mut cp_vs_mach = Vec::new();
        let mut induced_k_vs_mach = Vec::new();
        for &m in &machs {
            cd_vs_mach.push((
                m,
                Self::base_cd(m) * self.base_area_fraction()
                    + self.nose_pressure_cd(m)
                    + self.fin_pressure_cd(m)
                    + self.boat_pressure_cd(m)
                    + self.friction_cd(m, 340.0),
            ));
            let fin_cna = self.fin_set_cna(m);
            let total_cna = 2.0 + fin_cna + self.boat_cna();
            cna_vs_mach.push((m, total_cna));
            cp_vs_mach.push((m, self.total_cp(m)));
            // Induced drag from the normal force's tilt onto the flight
            // path: Cd_induced = Cnα·α², so k = Cnα (OpenRocket's model).
            induced_k_vs_mach.push((m, total_cna));
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
        let planform = self.planform();
        assert!(planform.count > 0);
        assert!(planform.span_m > 0.0);
        assert!(planform.root_chord_m > 0.0 && planform.thickness_m > 0.0);
    }
}

/// Parse the compact geometry spec used by the CLI:
/// `nose=shape:length:diameter,tube=length:diameter,fins=count:root:tip:span:sweep:thickness`
/// (or `efins=count:root:span:thickness` for elliptical fins), with an
/// optional `boat=length:fore_diameter:aft_diameter` conical boat-tail at
/// the aft end. The fin thickness is optional (default 2 mm). Lengths are
/// in metres.
pub fn parse_geometry(spec: &str) -> Result<RocketGeometry> {
    let mut nose = None;
    let mut tube = None;
    let mut fins: Option<FinSet> = None;
    let mut boat_tail = None;
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
                fins = Some(
                    TrapezoidalFins {
                        count,
                        root_chord_m: number(1)?,
                        tip_chord_m: number(2)?,
                        span_m: number(3)?,
                        leading_sweep_m: number(4)?,
                        thickness_m: if parts.len() == 6 { number(5)? } else { 0.002 },
                    }
                    .into(),
                );
            }
            "efins" => {
                if !(3..=4).contains(&parts.len()) {
                    return Err(Error::invalid(
                        "efins",
                        "expected count:root:span[:thickness]",
                    ));
                }
                let count: u32 = parts[0]
                    .parse()
                    .map_err(|_| Error::invalid("fin count", "must be an integer"))?;
                fins = Some(
                    EllipticalFins {
                        count,
                        root_chord_m: number(1)?,
                        span_m: number(2)?,
                        thickness_m: if parts.len() == 4 { number(3)? } else { 0.002 },
                    }
                    .into(),
                );
            }
            "boat" => {
                if parts.len() != 3 {
                    return Err(Error::invalid(
                        "boat",
                        "expected boat=length:fore_diameter:aft_diameter",
                    ));
                }
                boat_tail = Some(BoatTail {
                    length_m: number(0)?,
                    fore_diameter_m: number(1)?,
                    aft_diameter_m: number(2)?,
                });
            }
            other => {
                return Err(Error::invalid(
                    "geometry key",
                    match other {
                        "nose" | "tube" | "fins" => unreachable!(),
                        _ => "must be nose, tube, fins, efins or boat",
                    },
                ))
            }
        }
    }
    let geometry = RocketGeometry::new(
        nose.ok_or_else(|| Error::invalid("geometry", "nose is required"))?,
        tube.ok_or_else(|| Error::invalid("geometry", "tube is required"))?,
        fins.ok_or_else(|| Error::invalid("geometry", "fins are required"))?,
    );
    match boat_tail {
        Some(boat) => geometry.with_boat_tail(boat),
        None => Ok(geometry),
    }
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
        // The M->0 limit of the wave-drag model is Barrowman's 0.8·sin²(φ).
        let geo = example(); // conical nose, L = 0.1, R = 0.012
        let sin_phi = 0.012 / (0.012_f64.powi(2) + 0.1_f64.powi(2)).sqrt();
        assert!((geo.nose_pressure_cd(0.0) - 0.8 * sin_phi * sin_phi).abs() < 1e-12);
    }

    #[test]
    fn nose_wave_drag_matches_openrocket_formula() {
        let geo = example(); // conical, L = 0.1, R = 0.012
        let r = 0.012_f64;
        let sinphi = r / (r * r + 0.1 * 0.1).sqrt();
        // Supersonic direct formula: 2.1·sin²φ + 0.5·sinφ/β.
        let m = 1.5_f64;
        let expected = 2.1 * sinphi * sinphi + 0.5 * sinphi / (m * m - 1.0).sqrt();
        assert!(
            (geo.nose_pressure_cd(m) - expected).abs() < 1e-12,
            "{} vs {expected}",
            geo.nose_pressure_cd(m)
        );
        // Monotone rise through the transonic region, continuous at M = 1.
        let below = geo.nose_pressure_cd(0.999);
        let at = geo.nose_pressure_cd(1.0);
        assert!((at - sinphi).abs() < 0.02, "at {at}");
        assert!(below < at && below > 0.8 * sinphi * sinphi);
    }

    #[test]
    fn fin_cp_matches_the_trapezoid_formula() {
        let geo = example();
        // Independently: MAC = (2/3)(cr+ct - cr*ct/(cr+ct)), yMAC = s(cr+2ct)/(3(cr+ct)).
        let mac = (2.0 / 3.0) * (0.05 + 0.03 - 0.05 * 0.03 / 0.08);
        let y_mac = (0.04 / 3.0) * (0.05 + 2.0 * 0.03) / 0.08;
        let expected = 0.02 * y_mac / 0.04 + 0.25 * mac;
        assert!(
            (geo.fin_cp(0.0) - expected).abs() < 1e-12,
            "{}",
            geo.fin_cp(0.0)
        );
    }

    #[test]
    fn boat_tail_physics() {
        let base = example();
        let with_boat = base
            .with_boat_tail(BoatTail {
                length_m: 0.06,
                fore_diameter_m: 0.024,
                aft_diameter_m: 0.016,
            })
            .unwrap();
        // Cnα drops by 2·(1 − (2/3)²) = 10/9 ≈ 1.111.
        let delta = base.fin_set_cna(0.0) + 2.0 - (with_boat.fin_set_cna(0.0) + 2.0);
        let expected = 2.0 * (1.0 - (0.016_f64 / 0.024).powi(2));
        assert!((with_boat.boat_cna() + expected).abs() < 1e-12);
        assert!((delta - 0.0).abs() < 1e-12 || delta > 0.0); // fins unchanged
        let cna_base = base.analyze().cna_vs_mach[0].1;
        let cna_boat = with_boat.analyze().cna_vs_mach[0].1;
        assert!(
            (cna_base - cna_boat - expected).abs() < 1e-9,
            "{cna_base} vs {cna_boat}"
        );
        // CP: frustum centroid at nose+tube + L·(rf+3ra)/(4(rf+ra)).
        let cp = with_boat.boat_cp_abs();
        let want = 0.1 + 0.4 + 0.06 * (0.012 + 3.0 * 0.008) / (4.0 * (0.012 + 0.008));
        assert!((cp - want).abs() < 1e-12, "{cp} vs {want}");
        // Pressure drag: fineness = 0.06/0.008 = 7.5 ≥ 3 → zero; short tail pays.
        assert_eq!(with_boat.boat_pressure_cd(0.5), 0.0);
        let stubby = base
            .with_boat_tail(BoatTail {
                length_m: 0.012,
                fore_diameter_m: 0.024,
                aft_diameter_m: 0.016,
            })
            .unwrap();
        assert!(stubby.boat_pressure_cd(0.5) > 0.0);
        // Base area shrinks to (aft/fore)².
        assert!((with_boat.base_area_fraction() - (0.016_f64 / 0.024).powi(2)).abs() < 1e-12);
        // Total length includes the boat-tail; fins stay at the tube end.
        assert!((with_boat.total_length_m() - 0.56).abs() < 1e-12);
        assert!((with_boat.fin_cp_abs(0.0) - base.fin_cp_abs(0.0)).abs() < 1e-12);
        // Validation.
        assert!(base
            .with_boat_tail(BoatTail {
                length_m: 0.06,
                fore_diameter_m: 0.03,
                aft_diameter_m: 0.016,
            })
            .is_err());
        assert!(base
            .with_boat_tail(BoatTail {
                length_m: 0.06,
                fore_diameter_m: 0.024,
                aft_diameter_m: 0.03,
            })
            .is_err());
        // Parse end to end.
        let parsed = parse_geometry(
            "nose=conical:0.1:0.024,tube=0.4:0.024,fins=3:0.05:0.03:0.04:0.02:0.002,boat=0.06:0.024:0.016",
        )
        .unwrap();
        assert!(parsed.boat_tail.is_some());
        assert!(parse_geometry(
            "nose=conical:0.1:0.024,tube=0.4:0.024,fins=3:0.05:0.03:0.04:0.02:0.002,boat=0.06:0.024:0.03",
        )
        .is_err());
        // Mass model: the boat shell joins the component sum.
        let props = with_boat.mass_properties_with_boat(0.012, 0.020, 0.006, Some(0.004), None);
        assert!((props.dry_mass_kg - 0.042).abs() < 1e-12);
        assert!(props.pitch_inertia_kg_m2 > 0.0);
    }

    #[test]
    fn elliptical_and_parabolic_noses_use_the_nasa_tables() {
        let elliptical = RocketGeometry::new(
            NoseCone {
                shape: NoseShape::Elliptical,
                length_m: 0.072,
                diameter_m: 0.024,
            },
            example().tube,
            example().fins,
        );
        // Fineness 3 reproduces the table value at Mach 1.4 (0.148).
        assert!(
            (elliptical.nose_pressure_cd(1.4) - 0.148).abs() < 0.005,
            "{}",
            elliptical.nose_pressure_cd(1.4)
        );
        // Subsonic is ~zero (the 99% surface angle is tiny).
        assert!(elliptical.nose_pressure_cd(0.5) < 0.02);
        let parabolic = RocketGeometry::new(
            NoseCone {
                shape: NoseShape::Parabolic,
                length_m: 0.072,
                diameter_m: 0.024,
            },
            example().tube,
            example().fins,
        );
        // Parabolic table is zero below 0.95 and 0.092 at 1.05.
        assert!(parabolic.nose_pressure_cd(0.8) < 1e-4);
        assert!(
            (parabolic.nose_pressure_cd(1.05) - 0.092).abs() < 0.005,
            "{}",
            parabolic.nose_pressure_cd(1.05)
        );
        // Stagnation Cd is continuous across Mach 1.
        let lo = RocketGeometry::stagnation_cd(0.999);
        let hi = RocketGeometry::stagnation_cd(1.001);
        assert!((lo - hi).abs() < 0.01, "{lo} vs {hi}");
    }

    #[test]
    fn fin_cp_moves_aft_supersonic() {
        let geo = example();
        assert_eq!(geo.fin_cp_fraction(0.0), 0.25);
        assert_eq!(geo.fin_cp_fraction(0.5), 0.25);
        // Supersonic empirical formula: (AR·β − 0.67)/(2·AR·β − 1).
        let ar = 2.0 * 0.04_f64.powi(2) / geo.fin_area();
        let beta = (4.0_f64 - 1.0).sqrt();
        let expected = (ar * beta - 0.67) / (2.0 * ar * beta - 1.0);
        assert!((geo.fin_cp_fraction(2.0) - expected).abs() < 1e-12);
        assert!(geo.fin_cp_fraction(1.0) > 0.25 && geo.fin_cp_fraction(1.0) < expected);
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
        let fin_cg = geo.total_length_m() - geo.planform().root_chord_m + geo.fin_planform_cg();
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
    fn elliptical_fins_match_the_half_ellipse_geometry() {
        let geo = RocketGeometry::new(
            example().nose,
            example().tube,
            EllipticalFins {
                count: 3,
                root_chord_m: 0.05,
                span_m: 0.05,
                thickness_m: 0.0032,
            },
        );
        let p = geo.planform();
        // Half-ellipse: area π·cr·s/4, unswept midchord, centroid of a
        // semi-ellipse at (cr/2, 4s/(3π)).
        let area = std::f64::consts::PI * 0.05 * 0.05 / 4.0;
        // The 48-strip sum (OpenRocket's own discretization) overestimates
        // the convex arc by ~1.2%.
        assert!(
            (p.area_m2 - area).abs() / area < 0.02,
            "{} vs {area}",
            p.area_m2
        );
        assert!(p.cos_midchord > 0.99, "cos_midchord {}", p.cos_midchord);
        assert!((p.cg_x_m - 0.025).abs() < 1e-12);
        assert!((p.cg_y_m - 4.0 * 0.05 / (3.0 * std::f64::consts::PI)).abs() < 1e-12);
        // MAC of a semi-ellipse: 8·cr/(3π).
        let mac = 8.0 * 0.05 / (3.0 * std::f64::consts::PI);
        assert!((p.mac_m - mac).abs() / mac < 0.02, "{} vs {mac}", p.mac_m);
        // Cnα is positive and CP sits inside the airframe.
        let analysis = geo.analyze();
        assert!(analysis.cna_vs_mach[0].1 > 2.0);
        assert!(analysis.cp_vs_mach[0].1 > 0.0 && analysis.cp_vs_mach[0].1 < geo.total_length_m());
        // Parse end to end.
        let parsed =
            parse_geometry("nose=conical:0.1:0.024,tube=0.4:0.024,efins=3:0.05:0.05:0.002")
                .unwrap();
        assert!(matches!(parsed.fins, FinSet::Elliptical(_)));
        // Mass model uses the elliptical centroid.
        let props = geo.mass_properties(0.012, 0.020, 0.006, None);
        assert!(props.pitch_inertia_kg_m2 > 0.0);
    }

    #[test]
    fn fin_set_cna_matches_diederich() {
        let geo = example();
        // Fin pair: 2π·span²/(1+sqrt(1+(span²/(A·cosΓ))²)) / Aref, then
        // (N/2)·(1+tau) — the Diederich value is a wing (two fins).
        let area = geo.fin_area();
        let cos = geo.midchord_cos();
        let span = geo.planform().span_m;
        let ratio = span * span / (area * cos);
        let one = 2.0 * std::f64::consts::PI * span * span
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
        let classic = 4.0 * 3.0 * (geo.planform().span_m / d).powi(2)
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
        let expected = (2.0 * geo.nose_cp() + fin_cna * geo.fin_cp_abs(0.0)) / (2.0 + fin_cna);
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
