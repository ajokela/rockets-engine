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
    atmosphere::calculate_air_density_cimp, constants::GRAINS_TO_KG,
    derivatives::compute_derivatives, drag::DragTable, BallisticInputs,
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
    wind_mps: Vec3,
    omega_rps: Option<Vec3>,
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
            wind_mps,
            omega_rps: None,
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
            None,
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
        self.acceleration_in_wind(state, mass_kg, self.wind_mps)
    }
}
