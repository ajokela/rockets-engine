//! Adapter to ballistics-engine 0.48's public derivative kernel.
//!
//! Uses the body's own Cd-versus-Mach curve, fixed diameter, standard gravity and
//! a level world frame. SI mass and its imperial mirror are updated at every stage,
//! so aerodynamic deceleration responds to fuel consumption. Modified point-mass
//! effects — spin/Magnus with yaw of repose, tip-off yaw and Coriolis — are
//! opt-in via the `with_*` builders; reference-BC and postprocessing effects
//! (Litz spin drift) stay disabled.

use crate::{finite_vec, positive, Error, FlightState, ForceModel, Result, Vec3};
use ballistics_engine::{
    atmosphere::{calculate_air_density_cimp, AtmoSock},
    constants::GRAINS_TO_KG,
    derivatives::compute_derivatives,
    drag::DragTable,
    wind::{WindSegment, WindSock},
    BallisticInputs,
};

/// Atmospheric inputs supported by the upstream derivative kernel.
#[derive(Debug, Clone, Copy)]
pub enum Atmosphere {
    /// Station measurements; the upstream model applies its local altitude lapse.
    /// Humidity affects station density; this derivative path uses dry-air sound speed.
    Standard {
        /// Elevation of the weather station above sea level, in metres.
        station_altitude_m: f64,
        /// Station temperature in degrees Celsius (greater than absolute zero).
        temperature_c: f64,
        /// Positive station pressure in hectopascals.
        pressure_hpa: f64,
        /// Relative humidity as a percentage in [0, 100].
        humidity_percent: f64,
    },
    /// Constant density and sound speed, independent of position.
    /// Upstream's direct-mode sentinel requires 0 <= density < 2 and sound speed > 200.
    Uniform {
        /// Air density in kilograms per cubic metre; zero disables drag.
        density_kg_m3: f64,
        /// Sound speed in metres per second.
        speed_of_sound_mps: f64,
    },
}

impl Default for Atmosphere {
    fn default() -> Self {
        Self::Standard {
            station_altitude_m: 0.0,
            temperature_c: 15.0,
            pressure_hpa: 1013.25,
            humidity_percent: 0.0,
        }
    }
}

impl Atmosphere {
    fn parameters(self) -> Result<(f64, f64, f64, f64)> {
        match self {
            Self::Standard {
                station_altitude_m,
                temperature_c,
                pressure_hpa,
                humidity_percent,
            } => {
                if !station_altitude_m.is_finite()
                    || !temperature_c.is_finite()
                    || temperature_c <= -273.15
                    || !humidity_percent.is_finite()
                    || !(0.0..=100.0).contains(&humidity_percent)
                {
                    return Err(Error::invalid("standard atmosphere", "requires finite altitude, temperature above absolute zero and humidity in [0, 100]"));
                }
                positive(pressure_hpa, "pressure_hpa")?;
                let ratio =
                    calculate_air_density_cimp(temperature_c, pressure_hpa, humidity_percent)
                        / 1.225;
                positive(ratio, "station density ratio")?;
                Ok((station_altitude_m, temperature_c, pressure_hpa, ratio))
            }
            Self::Uniform {
                density_kg_m3,
                speed_of_sound_mps,
            } => {
                if !density_kg_m3.is_finite()
                    || !(0.0..2.0).contains(&density_kg_m3)
                    || !speed_of_sound_mps.is_finite()
                    || speed_of_sound_mps <= 200.0
                {
                    return Err(Error::invalid(
                        "uniform atmosphere",
                        "upstream requires density in [0, 2) and sound speed > 200 m/s",
                    ));
                }
                Ok((density_kg_m3, speed_of_sound_mps, 0.0, 0.0))
            }
        }
    }
}

/// Gravity and aerodynamic drag delegated to ballistics-engine's force kernel.
#[derive(Debug, Clone)]
pub struct BallisticForces {
    inputs: BallisticInputs,
    atmos_params: (f64, f64, f64, f64),
    atmosphere: Atmosphere,
    wind_mps: Vec3,
    omega_rps: Option<Vec3>,
    body_lift: Option<BodyLift>,
    atmo_sock: Option<AtmoSock>,
    wind_sock: Option<WindSock>,
}

/// Validated angle-of-attack aerodynamics: normal-force slope and induced-drag
/// factor, each piecewise-linear in Mach via the upstream table type.
#[derive(Debug, Clone)]
struct BodyLift {
    /// Normal force coefficient slope per radian of angle of attack, vs Mach.
    cna_per_rad: DragTable,
    /// Induced drag increment per radian squared (Cd += k·α²), vs Mach.
    induced_k_per_rad2: DragTable,
}

/// Earth rotation rate, rad/s (McCoy-level precision, matching the upstream kernel).
const OMEGA_EARTH_RPS: f64 = 7.2921159e-5;

impl BallisticForces {
    /// Build using the vehicle's actual Cd curve, not a G-model reference curve or BC.
    /// At least two ascending, nonnegative Mach values and positive Cd values are required.
    /// Wind is a world-frame air velocity vector (positive X is a tailwind).
    pub fn new(
        diameter_m: f64,
        cd_vs_mach: Vec<(f64, f64)>,
        atmosphere: Atmosphere,
        wind_mps: Vec3,
    ) -> Result<Self> {
        positive(diameter_m, "diameter_m")?;
        let diameter_in = diameter_m / 0.0254;
        positive(diameter_in * diameter_in, "diameter area")?;
        if !finite_vec(&wind_mps) {
            return Err(Error::invalid("wind_mps", "must be finite"));
        }
        let (mach, cd) = cd_vs_mach.into_iter().unzip();
        let drag = DragTable::try_new(mach, cd).map_err(Error::Model)?;
        let inputs = BallisticInputs {
            bullet_diameter: diameter_m,
            caliber_inches: diameter_in,
            bullet_length: diameter_m,
            custom_drag_table: Some(drag),
            custom_drag_reference: false,
            altitude: 0.0,
            shooting_angle: 0.0,
            enable_advanced_effects: false,
            enable_magnus: false,
            enable_coriolis: false,
            use_enhanced_spin_drift: false,
            tipoff_yaw: 0.0,
            ..BallisticInputs::default()
        };
        Ok(Self {
            inputs,
            atmos_params: atmosphere.parameters()?,
            atmosphere,
            wind_mps,
            omega_rps: None,
            body_lift: None,
            atmo_sock: None,
            wind_sock: None,
        })
    }

    /// Apply a positive multiplier to the entire Cd curve. Default is 1.0.
    pub fn with_drag_scale(mut self, scale: f64) -> Result<Self> {
        positive(scale, "drag scale")?;
        if self
            .inputs
            .custom_drag_table
            .as_ref()
            .unwrap()
            .cd_values
            .iter()
            .any(|cd| !(cd * scale).is_finite())
        {
            return Err(Error::invalid("drag scale", "scaled Cd must remain finite"));
        }
        self.inputs.cd_scale = scale;
        Ok(self)
    }

    /// Change uniform world-frame wind between public integration steps.
    pub fn set_wind(&mut self, wind_mps: Vec3) -> Result<()> {
        if !finite_vec(&wind_mps) {
            return Err(Error::invalid("wind_mps", "must be finite"));
        }
        self.wind_mps = wind_mps;
        Ok(())
    }

    /// Modified point-mass spin: enable the kernel's Magnus force and yaw of
    /// repose for a spinning body. `twist_rate_in` is the effective twist in
    /// inches per turn — rifling language upstream; for a fin-canted rocket it
    /// is the roll rate the cant produces per unit velocity. `body_length_m` is
    /// the full airframe length (the stability correlations need it; [`Self::new`]
    /// only knows the diameter). `right_hand` selects the roll direction.
    ///
    /// The correlations are McCoy bullet shapes, so treat magnitudes as
    /// approximate for blunt or finned bodies; signs and scaling are physical.
    pub fn with_spin(
        mut self,
        twist_rate_in: f64,
        body_length_m: f64,
        right_hand: bool,
    ) -> Result<Self> {
        positive(twist_rate_in, "twist_rate_in")?;
        positive(body_length_m, "body_length_m")?;
        self.inputs.twist_rate = twist_rate_in;
        self.inputs.is_twist_right = right_hand;
        self.inputs.bullet_length = body_length_m;
        self.inputs.enable_magnus = true;
        Ok(self)
    }

    /// Modified point-mass Coriolis deflection, for planet-scale arcs.
    /// `latitude_deg` is the launch latitude in [-90, 90]; `shot_azimuth_deg` is
    /// the compass bearing fired along, 0 = north, 90 = east. Do not combine
    /// with an inertial-frame [`crate::CentralGravity`] model: Coriolis is the
    /// rotating-frame term and would double-count Earth rotation.
    pub fn with_coriolis(mut self, latitude_deg: f64, shot_azimuth_deg: f64) -> Result<Self> {
        if !latitude_deg.is_finite() || !(-90.0..=90.0).contains(&latitude_deg) {
            return Err(Error::invalid(
                "latitude_deg",
                "must be finite and in [-90, 90]",
            ));
        }
        if !shot_azimuth_deg.is_finite() {
            return Err(Error::invalid("shot_azimuth_deg", "must be finite"));
        }
        let lat = latitude_deg.to_radians();
        let az = shot_azimuth_deg.to_radians();
        self.omega_rps = Some(Vec3::new(
            OMEGA_EARTH_RPS * lat.cos() * az.cos(),  // X: downrange
            OMEGA_EARTH_RPS * lat.sin(),             // Y: vertical
            -OMEGA_EARTH_RPS * lat.cos() * az.sin(), // Z: lateral
        ));
        self.inputs.latitude = Some(latitude_deg);
        self.inputs.shot_azimuth = az;
        self.inputs.enable_coriolis = true;
        Ok(self)
    }

    /// Launch tip-off: an initial yaw in radians that decays exponentially with
    /// downrange distance over `decay_distance_m`. Models the rail/launcher
    /// disturbance; it perturbs the effective angle of attack early in flight.
    pub fn with_tipoff_yaw(mut self, yaw_rad: f64, decay_distance_m: f64) -> Result<Self> {
        if !yaw_rad.is_finite() {
            return Err(Error::invalid("tipoff yaw_rad", "must be finite"));
        }
        positive(decay_distance_m, "tipoff decay_distance_m")?;
        self.inputs.tipoff_yaw = yaw_rad;
        self.inputs.tipoff_decay_distance = decay_distance_m;
        Ok(self)
    }

    /// Angle-of-attack aerodynamics without attitude integration: a quasi-static
    /// normal force and induced drag from the commanded axis's angle to the
    /// relative wind.
    ///
    /// `cna_vs_mach` is the normal-force coefficient slope per radian of angle of
    /// attack (body + fins) versus Mach; `induced_k_vs_mach` adds Cd += k·α².
    /// Both need at least two strictly ascending Mach points and nonnegative,
    /// finite values. The angle of attack is measured each RK stage between the
    /// solver-resolved thrust axis and the wind-relative velocity, so a guided
    /// turn at angle of attack both turns harder (body lift) and pays drag for it.
    /// Reference area derives from the diameter passed to [`Self::new`].
    pub fn with_body_lift(
        mut self,
        cna_vs_mach: Vec<(f64, f64)>,
        induced_k_vs_mach: Vec<(f64, f64)>,
    ) -> Result<Self> {
        let table = |pairs: Vec<(f64, f64)>, name: &'static str| -> Result<DragTable> {
            if pairs.len() < 2 {
                return Err(Error::invalid(name, "needs at least two Mach points"));
            }
            for (i, (mach, value)) in pairs.iter().enumerate() {
                if !mach.is_finite() || *mach < 0.0 {
                    return Err(Error::invalid(name, "Mach must be finite and nonnegative"));
                }
                if i > 0 && *mach <= pairs[i - 1].0 {
                    return Err(Error::invalid(name, "Mach must strictly ascend"));
                }
                if !value.is_finite() || *value < 0.0 {
                    return Err(Error::invalid(
                        name,
                        "values must be finite and nonnegative",
                    ));
                }
            }
            let (mach_values, cd_values) = pairs.into_iter().unzip();
            Ok(DragTable::new(mach_values, cd_values))
        };
        self.body_lift = Some(BodyLift {
            cna_per_rad: table(cna_vs_mach, "cna_vs_mach")?,
            induced_k_per_rad2: table(induced_k_vs_mach, "induced_k_vs_mach")?,
        });
        Ok(self)
    }

    /// Downrange-segmented atmosphere (upstream `AtmoSock`): station-referenced
    /// zones, each `(temperature_c, pressure_hpa, humidity_percent, until_downrange_m)`.
    /// The zone is selected by downrange distance `position.x`; the base atmosphere
    /// must be [`Atmosphere::Standard`], and each zone's humidity applies to its
    /// station density while this derivative path uses dry-air sound speed.
    pub fn with_atmosphere_zones(mut self, zones: Vec<(f64, f64, f64, f64)>) -> Result<Self> {
        if !matches!(self.atmosphere, Atmosphere::Standard { .. }) {
            return Err(Error::invalid(
                "atmosphere zones",
                "require the Standard atmosphere, not Uniform",
            ));
        }
        if zones.is_empty() {
            return Err(Error::invalid(
                "atmosphere zones",
                "needs at least one zone",
            ));
        }
        for (temp_c, pressure_hpa, humidity, until_m) in &zones {
            if !temp_c.is_finite()
                || !pressure_hpa.is_finite()
                || *pressure_hpa <= 0.0
                || !humidity.is_finite()
                || !(0.0..=100.0).contains(humidity)
                || !until_m.is_finite()
                || *until_m <= 0.0
            {
                return Err(Error::invalid(
                    "atmosphere zone",
                    "needs finite temperature, positive pressure, humidity in [0, 100] and positive until_m",
                ));
            }
        }
        self.atmo_sock = Some(AtmoSock::new(zones));
        Ok(self)
    }

    /// Downrange-segmented wind (upstream `WindSock`). Each zone is
    /// `(speed_mps, angle_deg, until_downrange_m, vertical_mps)`, with the angle
    /// following the upstream wind-FROM convention (0° = headwind, 90° = from the
    /// right). Vertical is positive updraft. Zones are evaluated statelessly at
    /// each stage's `position.x`, so wind can change mid-integration.
    /// Overrides the uniform wind set in [`Self::new`]/[`Self::set_wind`].
    pub fn with_wind_segments(mut self, zones: Vec<(f64, f64, f64, f64)>) -> Result<Self> {
        if zones.is_empty() {
            return Err(Error::invalid(
                "wind segments",
                "needs at least one segment",
            ));
        }
        let segments: Vec<WindSegment> = zones
            .into_iter()
            .map(
                |(speed_mps, angle_deg, until_m, vertical_mps)| WindSegment {
                    speed_kmh: speed_mps * 3.6,
                    angle_deg,
                    until_m,
                    vertical_mps,
                },
            )
            .collect();
        let sock = WindSock::try_new(segments)
            .map_err(|e| Error::Model(format!("invalid wind segments: {e:?}")))?;
        self.wind_sock = Some(sock);
        Ok(self)
    }

    /// Wind at a stage: segmented downrange wind if configured, else the uniform vector.
    fn effective_wind(&self, state: &FlightState) -> Vec3 {
        match &self.wind_sock {
            Some(sock) => sock.vector_for_range_stateless(state.position_m.x),
            None => self.wind_mps,
        }
    }

    /// Local air density (kg/m³) and sound speed (m/s) at a geometric altitude,
    /// consistent with the atmosphere handed to the kernel.
    fn local_air(&self, altitude_m: f64) -> (f64, f64) {
        match self.atmosphere {
            Atmosphere::Standard { .. } => {
                let (base_alt, temp_c, press_hpa, ratio) = self.atmos_params;
                ballistics_engine::atmosphere::get_local_atmosphere(
                    altitude_m, base_alt, temp_c, press_hpa, ratio,
                )
            }
            Atmosphere::Uniform { .. } => {
                let (density, speed_of_sound, _, _) = self.atmos_params;
                (density, speed_of_sound)
            }
        }
    }

    /// Quasi-static AoA force increment: normal force toward the body-axis side of
    /// the velocity, plus induced drag along it. Zero when aligned or in still air.
    fn body_lift_accel(
        &self,
        state: &FlightState,
        mass_kg: f64,
        wind_mps: Vec3,
        body_axis: Vec3,
    ) -> Result<Vec3> {
        let Some(lift) = &self.body_lift else {
            return Ok(Vec3::zeros());
        };
        let v_air = state.velocity_mps - wind_mps;
        let speed = v_air.norm();
        // Degenerate cases: no relative wind, or a zero/ill-posed axis → no increment.
        if speed < 1e-12 || body_axis.amax() == 0.0 {
            return Ok(Vec3::zeros());
        }
        let v_hat = v_air / speed;
        let axis = body_axis / body_axis.norm();
        // Angle of attack from the axis to the relative wind.
        let perp = axis - axis.dot(&v_hat) * v_hat;
        let sin_alpha = perp.norm();
        let cos_alpha = axis.dot(&v_hat).clamp(-1.0, 1.0);
        let alpha = sin_alpha.atan2(cos_alpha);
        if alpha < 1e-12 {
            return Ok(Vec3::zeros());
        }
        let (rho, sound) = self.local_air(state.position_m.y);
        if rho <= 0.0 || sound <= 0.0 {
            return Ok(Vec3::zeros());
        }
        let mach = speed / sound;
        let q = 0.5 * rho * speed * speed;
        let area = std::f64::consts::FRAC_PI_4 * self.inputs.bullet_diameter.powi(2);
        let normal_force = q * area * lift.cna_per_rad.interpolate(mach) * alpha;
        let induced_drag = q * area * lift.induced_k_per_rad2.interpolate(mach) * alpha * alpha;
        let accel = (normal_force * (perp / sin_alpha) - induced_drag * v_hat) / mass_kg;
        if !finite_vec(&accel) {
            return Err(Error::NumericalFailure(
                "body lift returned non-finite acceleration",
            ));
        }
        Ok(accel)
    }

    /// Axis-aware counterpart to [`Self::acceleration_in_wind`], for wrappers that
    /// sample their own wind but want the body-lift increment.
    pub fn acceleration_with_axis_in_wind(
        &self,
        state: &FlightState,
        mass_kg: f64,
        wind_mps: Vec3,
        body_axis: Vec3,
    ) -> Result<Vec3> {
        let base = self.acceleration_in_wind(state, mass_kg, wind_mps)?;
        Ok(base + self.body_lift_accel(state, mass_kg, wind_mps, body_axis)?)
    }

    /// Evaluate using externally sampled wind, for a spatial or time-varying force wrapper.
    pub fn acceleration_in_wind(
        &self,
        state: &FlightState,
        mass_kg: f64,
        wind_mps: Vec3,
    ) -> Result<Vec3> {
        state.validate()?;
        positive(mass_kg, "mass_kg")?;
        if !finite_vec(&wind_mps) {
            return Err(Error::invalid("wind_mps", "must be finite"));
        }
        let mut inputs = self.inputs.clone();
        inputs.bullet_mass = mass_kg;
        inputs.weight_grains = mass_kg / GRAINS_TO_KG;
        positive(inputs.weight_grains, "mass in grains")?;
        let derivatives = compute_derivatives(
            state.position_m,
            state.velocity_mps,
            &inputs,
            wind_mps,
            self.atmos_params,
            inputs.bc_value,
            self.omega_rps,
            state.time_s,
            self.atmo_sock.as_ref(),
        );
        let acceleration = Vec3::new(derivatives[3], derivatives[4], derivatives[5]);
        if !finite_vec(&acceleration) {
            return Err(Error::NumericalFailure(
                "ballistics force model returned non-finite acceleration",
            ));
        }
        Ok(acceleration)
    }
}

impl ForceModel for BallisticForces {
    fn acceleration(&self, state: &FlightState, mass_kg: f64) -> Result<Vec3> {
        self.acceleration_in_wind(state, mass_kg, self.effective_wind(state))
    }

    fn acceleration_with_axis(
        &self,
        state: &FlightState,
        mass_kg: f64,
        body_axis: Vec3,
    ) -> Result<Vec3> {
        self.acceleration_with_axis_in_wind(state, mass_kg, self.effective_wind(state), body_axis)
    }
}
