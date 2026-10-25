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
    atmosphere::{
        calculate_air_density_cimp, get_local_atmosphere, reduce_qnh_to_station_pressure, AtmoSock,
    },
    constants::GRAINS_TO_KG,
    derivatives::compute_derivatives,
    drag::DragTable,
    wind::{WindSegment, WindSock},
    BallisticInputs,
};

/// Atmospheric inputs supported by the upstream derivative kernel.
#[derive(Debug, Clone, Copy, PartialEq)]
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

/// Lowest and highest station altitudes [`Atmosphere::icao_standard`] takes, in
/// metres above sea level: the ICAO troposphere, where one lapse rate holds.
pub(crate) const ICAO_STATION_RANGE_M: (f64, f64) = (-1000.0, 11000.0);

/// Earth radius upstream's ICAO table uses to turn geometric altitude into
/// geopotential height.
const GEOPOTENTIAL_EARTH_RADIUS_M: f64 = 6_356_766.0;

impl Atmosphere {
    /// A dry station at `station_altitude_m` above sea level reading the ICAO
    /// standard atmosphere for that altitude. Air at every altitude is then
    /// the standard atmosphere's, whatever the station altitude, so a launch
    /// site at elevation E flies through standard air with its station (and
    /// any atmosphere zones, which are readings at the station) at E.
    ///
    /// At 0 m this equals [`Atmosphere::default`] exactly. The altitude must
    /// be finite and within the troposphere, -1000 to 11000 m. Temperature
    /// follows the ICAO lapse of 6.5 K per kilometre of geopotential height;
    /// pressure is upstream's `reduce_qnh_to_station_pressure` of the
    /// 1013.25 hPa standard sea-level pressure.
    pub fn icao_standard(station_altitude_m: f64) -> Result<Self> {
        let (lowest, highest) = ICAO_STATION_RANGE_M;
        if !station_altitude_m.is_finite() || !(lowest..=highest).contains(&station_altitude_m) {
            return Err(Error::invalid(
                "station_altitude_m",
                "must be finite and within the ICAO troposphere, -1000 to 11000 m",
            ));
        }
        let geopotential_m = GEOPOTENTIAL_EARTH_RADIUS_M * station_altitude_m
            / (GEOPOTENTIAL_EARTH_RADIUS_M + station_altitude_m);
        let temperature_k = 288.15 - 0.0065 * geopotential_m;
        Ok(Self::Standard {
            station_altitude_m,
            temperature_c: temperature_k - 273.15,
            pressure_hpa: reduce_qnh_to_station_pressure(1013.25, station_altitude_m),
            humidity_percent: 0.0,
        })
    }

    pub(crate) fn parameters(self) -> Result<(f64, f64, f64, f64)> {
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

/// Local air density (kg/m³) and sound speed (m/s) for an atmosphere and its
/// kernel parameter tuple, at a geometric altitude above sea level.
fn local_air(
    atmosphere: Atmosphere,
    atmos_params: (f64, f64, f64, f64),
    altitude_m: f64,
) -> (f64, f64) {
    match atmosphere {
        Atmosphere::Standard { .. } => {
            let (base_alt, temp_c, press_hpa, ratio) = atmos_params;
            get_local_atmosphere(altitude_m, base_alt, temp_c, press_hpa, ratio)
        }
        Atmosphere::Uniform { .. } => {
            let (density, speed_of_sound, _, _) = atmos_params;
            (density, speed_of_sound)
        }
    }
}

/// One atmosphere and its optional downrange zones: the air-at-position lookup
/// every force reads, so drag, body lift, recovery drag and the weathercock all
/// see the same air.
///
/// [`Air::at`] reproduces the upstream 0.48.0 `compute_derivatives` lookup that
/// the drag kernel runs on the same atmosphere tuple and zone ladder.
#[derive(Debug, Clone)]
pub(crate) struct Air {
    atmosphere: Atmosphere,
    params: (f64, f64, f64, f64),
    zones: Option<AtmoSock>,
}

impl Air {
    pub(crate) fn new(atmosphere: Atmosphere) -> Result<Self> {
        Ok(Self {
            params: atmosphere.parameters()?,
            atmosphere,
            zones: None,
        })
    }

    /// Attach station-referenced zones, each `(temperature_c, pressure_hpa,
    /// humidity_percent, until_downrange_m)`; see
    /// [`BallisticForces::with_atmosphere_zones`] for the contract.
    pub(crate) fn with_zones(mut self, zones: Vec<(f64, f64, f64, f64)>) -> Result<Self> {
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
        self.zones = Some(AtmoSock::new(zones));
        Ok(self)
    }

    /// The kernel's atmosphere tuple.
    pub(crate) fn params(&self) -> (f64, f64, f64, f64) {
        self.params
    }

    /// The zone ladder handed to the kernel, if any.
    pub(crate) fn zones(&self) -> Option<&AtmoSock> {
        self.zones.as_ref()
    }

    /// Air density (kg/m³) and sound speed (m/s) at a world position, with Y
    /// the altitude above sea level and X the downrange distance.
    ///
    /// As upstream does: a `Uniform` atmosphere returns its constants. A
    /// `Standard` one lapses its station reading from the station altitude to
    /// Y; with zones, the zone selected by X (`AtmoSock::atmo_for_range`)
    /// replaces the station temperature and pressure, and its density ratio
    /// (CIPM density at the zone's temperature, pressure and humidity, over
    /// 1.225) replaces the station's, before that same lapse. Density matches
    /// the kernel's bit for bit. Sound speed is `get_local_atmosphere`'s
    /// dry-air value, as these forces have always used; the kernel's own
    /// moist formula at zero humidity agrees with it to about 4e-7.
    pub(crate) fn at(&self, position_m: Vec3) -> (f64, f64) {
        match (self.atmosphere, &self.zones) {
            (Atmosphere::Standard { .. }, Some(sock)) => {
                let (temp_c, pressure_hpa, humidity) = sock.atmo_for_range(position_m.x);
                let ratio = calculate_air_density_cimp(temp_c, pressure_hpa, humidity) / 1.225;
                get_local_atmosphere(position_m.y, self.params.0, temp_c, pressure_hpa, ratio)
            }
            _ => local_air(self.atmosphere, self.params, position_m.y),
        }
    }
}

/// Validated Mach table with nonnegative values, shared by the lift and
/// weathercock parameter curves.
pub(crate) fn mach_table(pairs: Vec<(f64, f64)>, name: &'static str) -> Result<DragTable> {
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
}

/// Gravity and aerodynamic drag delegated to ballistics-engine's force kernel.
#[derive(Debug, Clone)]
pub struct BallisticForces {
    inputs: BallisticInputs,
    air: Air,
    wind_mps: Vec3,
    /// An arbitrary wind field (winds-engine), sampled per state; overrides
    /// the uniform vector and any zone ladder.
    wind_field: Option<SharedWindField>,
    turbulence: Option<Turbulence>,
    omega_rps: Option<Vec3>,
    body_lift: Option<BodyLift>,
    wind_sock: Option<WindSock>,
}

/// A winds-engine field shared (Arc) between the force model, the
/// weathercock and the recovery descent, with `Debug` and `Clone` so the
/// containing models keep their derives.
#[derive(Clone)]
pub(crate) struct SharedWindField(
    pub(crate) std::sync::Arc<dyn winds_engine::WindField + Send + Sync>,
);

impl std::fmt::Debug for SharedWindField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("SharedWindField(..)")
    }
}

/// Number of sinusoid components per axis in the turbulence gust model.
const TURBULENCE_COMPONENTS: usize = 8;

/// Deterministic frozen-field turbulence: a multisine gust per axis with a
/// pink (1/f power) spectrum, fully fixed by its seed. Stateless, so RK4
/// substeps, snapshots and replays all see identical wind at identical times.
#[derive(Debug, Clone)]
pub(crate) struct Turbulence {
    /// Per-axis (angular frequency rad/s, amplitude m/s, phase rad) components.
    axes: [[(f64, f64, f64); TURBULENCE_COMPONENTS]; 3],
}

impl Turbulence {
    /// SplitMix64 finalizer as a stateless hash → [0, 1).
    fn hash01(mut z: u64) -> f64 {
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        let z = z ^ (z >> 31);
        (z >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Gusts with RMS `sigma_mps` per axis and energy concentrated near the
    /// period `tau_s` (frequencies log-spaced over [1/(10τ), 10/τ]).
    pub(crate) fn new(sigma_mps: f64, tau_s: f64, seed: u64) -> Result<Self> {
        if !sigma_mps.is_finite() || sigma_mps < 0.0 {
            return Err(Error::invalid("turbulence sigma", "must be nonnegative"));
        }
        if !tau_s.is_finite() || tau_s <= 0.0 {
            return Err(Error::invalid("turbulence tau", "must be positive"));
        }
        let lo = 0.1 / tau_s;
        let ratio = 100.0_f64;
        let step = ratio.powf(1.0 / (TURBULENCE_COMPONENTS - 1) as f64);
        let norm: f64 = (0..TURBULENCE_COMPONENTS)
            .map(|i| 1.0 / (lo * step.powi(i as i32)))
            .sum();
        let scale = (2.0 * sigma_mps * sigma_mps / norm).sqrt();
        let mut axes = [[(0.0, 0.0, 0.0); TURBULENCE_COMPONENTS]; 3];
        for (axis, comps) in axes.iter_mut().enumerate() {
            for (i, comp) in comps.iter_mut().enumerate() {
                let omega = lo * step.powi(i as i32);
                let amplitude = scale / omega.sqrt();
                let phase = 2.0
                    * std::f64::consts::PI
                    * Self::hash01(
                        seed ^ 0x9E3779B97F4A7C15u64.wrapping_mul(1 + axis as u64 * 64 + i as u64),
                    );
                *comp = (omega, amplitude, phase);
            }
        }
        Ok(Self { axes })
    }

    pub(crate) fn gust_vec(&self, t: f64) -> Vec3 {
        Vec3::new(
            self.axes[0]
                .iter()
                .map(|(w, a, p)| a * (w * t + p).sin())
                .sum(),
            self.axes[1]
                .iter()
                .map(|(w, a, p)| a * (w * t + p).sin())
                .sum(),
            self.axes[2]
                .iter()
                .map(|(w, a, p)| a * (w * t + p).sin())
                .sum(),
        )
    }
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
            air: Air::new(atmosphere)?,
            wind_mps,
            wind_field: None,
            turbulence: None,
            omega_rps: None,
            body_lift: None,
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
            mach_table(pairs, name)
        };
        self.body_lift = Some(BodyLift {
            cna_per_rad: table(cna_vs_mach, "cna_vs_mach")?,
            induced_k_per_rad2: table(induced_k_vs_mach, "induced_k_vs_mach")?,
        });
        Ok(self)
    }

    /// Change the effective twist between public steps, for a caller-integrated
    /// roll model (see [`crate::RollDynamics`]). Pair with [`Self::with_spin`]
    /// for the body length and handedness; the rate must stay positive.
    pub fn set_twist(&mut self, twist_rate_in: f64) -> Result<()> {
        positive(twist_rate_in, "twist_rate_in")?;
        self.inputs.twist_rate = twist_rate_in;
        Ok(())
    }

    /// Downrange-segmented atmosphere (upstream `AtmoSock`): station-referenced
    /// zones, each `(temperature_c, pressure_hpa, humidity_percent, until_downrange_m)`.
    /// Each zone is a reading taken at the station altitude: the zone selected
    /// by downrange distance `position.x` replaces the station reading, and the
    /// lapse from the station altitude to Y applies on top. The base atmosphere
    /// must be [`Atmosphere::Standard`], and each zone's humidity applies to its
    /// station density while this derivative path uses dry-air sound speed.
    /// Drag, spin and body lift all read the zoned air; give
    /// [`crate::RecoveryForces`] and [`crate::Weathercock`] the same ladder
    /// through their own `with_atmosphere_zones`.
    pub fn with_atmosphere_zones(mut self, zones: Vec<(f64, f64, f64, f64)>) -> Result<Self> {
        self.air = self.air.with_zones(zones)?;
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

    /// Add deterministic frozen-field turbulence: multisine gusts with a
    /// pink spectrum, RMS `sigma_mps` per axis and energy concentrated near
    /// the period `tau_s`. `seed` fixes the phases (dispersion reseeds per
    /// run), so runs are exactly reproducible.
    pub fn with_turbulence(mut self, sigma_mps: f64, tau_s: f64, seed: u64) -> Result<Self> {
        self.turbulence = Some(Turbulence::new(sigma_mps, tau_s, seed)?);
        Ok(self)
    }

    /// Sample an arbitrary winds-engine field instead of the uniform vector
    /// or zone ladder. Shared as an `Arc` so the weathercock and the
    /// recovery descent can sample the same air. Stateless and
    /// deterministic, so integration stays reproducible.
    pub fn with_wind_field(
        mut self,
        field: std::sync::Arc<dyn winds_engine::WindField + Send + Sync>,
    ) -> Result<Self> {
        self.wind_field = Some(SharedWindField(field));
        Ok(self)
    }

    /// Wind at a stage: the configured field if any, else segmented
    /// downrange wind if configured, else the uniform vector; plus
    /// frozen-field turbulence if configured.
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
        let (rho, sound) = self.air.at(state.position_m);
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
            self.air.params(),
            inputs.bc_value,
            self.omega_rps,
            state.time_s,
            self.air.zones(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn turbulence_rms_matches_sigma() {
        let gust = Turbulence::new(2.5, 2.0, 42).unwrap();
        // Long-run statistics per axis: mean ~0, RMS ~ sigma.
        for axis in 0..3 {
            let samples: Vec<f64> = (0..20_000)
                .map(|i| gust.gust_vec(i as f64 * 0.05)[axis])
                .collect();
            let mean = samples.iter().sum::<f64>() / samples.len() as f64;
            let var =
                samples.iter().map(|s| (s - mean).powi(2)).sum::<f64>() / samples.len() as f64;
            assert!(mean.abs() < 0.1, "axis {axis} mean {mean}");
            assert!(
                (var.sqrt() - 2.5).abs() / 2.5 < 0.05,
                "axis {axis} rms {}",
                var.sqrt()
            );
        }
    }

    #[test]
    fn turbulence_is_deterministic_per_seed() {
        let a = Turbulence::new(1.0, 2.0, 7).unwrap();
        let b = Turbulence::new(1.0, 2.0, 7).unwrap();
        let c = Turbulence::new(1.0, 2.0, 8).unwrap();
        for i in 0..100 {
            let t = i as f64 * 0.1;
            assert_eq!(a.gust_vec(t), b.gust_vec(t));
        }
        assert_ne!(a.gust_vec(3.3), c.gust_vec(3.3));
    }

    #[test]
    fn turbulence_validates_its_numbers() {
        assert!(Turbulence::new(-1.0, 2.0, 0).is_err());
        assert!(Turbulence::new(1.0, 0.0, 0).is_err());
        assert!(Turbulence::new(f64::NAN, 2.0, 0).is_err());
    }

    fn pad_station() -> Atmosphere {
        Atmosphere::Standard {
            station_altitude_m: 1500.0,
            temperature_c: 5.25,
            pressure_hpa: 845.6,
            humidity_percent: 30.0,
        }
    }

    const POSITIONS: [(f64, f64); 6] = [
        (0.0, 0.0),
        (50.0, 1500.0),
        (99.0, 1800.0),
        (500.0, 4000.0),
        (5_000.0, 12_000.0),
        (50_000.0, -200.0),
    ];

    #[test]
    fn air_without_zones_is_the_station_lapse() {
        // Byte-identical to what recovery, the weathercock and body lift read
        // before the shared lookup: get_local_atmosphere from the station.
        for atmosphere in [Atmosphere::default(), pad_station()] {
            let (base_alt, temp_c, press_hpa, ratio) = atmosphere.parameters().unwrap();
            let air = Air::new(atmosphere).unwrap();
            for (x, y) in POSITIONS {
                assert_eq!(
                    air.at(Vec3::new(x, y, 7.0)),
                    get_local_atmosphere(y, base_alt, temp_c, press_hpa, ratio)
                );
            }
        }
        let uniform = Air::new(Atmosphere::Uniform {
            density_kg_m3: 0.9,
            speed_of_sound_mps: 330.0,
        })
        .unwrap();
        assert_eq!(uniform.at(Vec3::new(10.0, 9000.0, 0.0)), (0.9, 330.0));
    }

    #[test]
    fn a_zone_reading_replaces_the_station_reading() {
        // A zone carrying a reading reads exactly like a station with that
        // reading at the same altitude, and the zone is chosen by X.
        let low = Atmosphere::Standard {
            station_altitude_m: 1500.0,
            temperature_c: -5.0,
            pressure_hpa: 700.0,
            humidity_percent: 50.0,
        };
        let zoned = Air::new(pad_station())
            .unwrap()
            .with_zones(vec![(5.25, 845.6, 30.0, 100.0), (-5.0, 700.0, 50.0, 1e6)])
            .unwrap();
        let near = Air::new(pad_station()).unwrap();
        let far = Air::new(low).unwrap();
        for (x, y) in POSITIONS {
            let p = Vec3::new(x, y, 0.0);
            let expected = if x < 100.0 { near.at(p) } else { far.at(p) };
            assert_eq!(zoned.at(p), expected, "at x {x}, y {y}");
        }
    }

    #[test]
    fn air_density_is_the_drag_kernels_density() {
        // With a Mach-independent Cd the kernel's drag depends on the air only
        // through its density, so the zoned kernel must match a kernel handed
        // the lookup's density directly, at every position and zone.
        let cd = vec![(0.0, 0.4), (5.0, 0.4)];
        let zones = vec![(5.25, 845.6, 30.0, 100.0), (-5.0, 700.0, 50.0, 1e6)];
        let zoned = BallisticForces::new(0.05, cd.clone(), pad_station(), Vec3::zeros())
            .unwrap()
            .with_atmosphere_zones(zones.clone())
            .unwrap();
        let air = Air::new(pad_station()).unwrap().with_zones(zones).unwrap();
        for (x, y) in POSITIONS {
            let state = FlightState {
                time_s: 0.0,
                position_m: Vec3::new(x, y, 0.0),
                velocity_mps: Vec3::new(150.0, 40.0, 0.0),
            };
            let (density_kg_m3, speed_of_sound_mps) = air.at(state.position_m);
            let direct = BallisticForces::new(
                0.05,
                cd.clone(),
                Atmosphere::Uniform {
                    density_kg_m3,
                    speed_of_sound_mps,
                },
                Vec3::zeros(),
            )
            .unwrap();
            assert_eq!(
                zoned.acceleration_in_wind(&state, 0.3, Vec3::zeros()),
                direct.acceleration_in_wind(&state, 0.3, Vec3::zeros()),
                "at x {x}, y {y}"
            );
        }
    }

    #[test]
    fn icao_standard_at_sea_level_is_the_default() {
        assert_eq!(Atmosphere::icao_standard(0.0), Ok(Atmosphere::default()));
    }

    #[test]
    fn icao_standard_matches_upstreams_standard_atmosphere() {
        // Upstream's resolver turns a default (15 °C, 1013.25 hPa) reading at a
        // real altitude into its ICAO-table values; above its 1 m sentinel the
        // constructor must agree with it.
        use ballistics_engine::atmosphere::resolve_station_conditions;
        for altitude in [1.5, 10.0, 100.0, 1500.0, 4000.0, 8848.0, 10_999.0, -400.0] {
            let Ok(Atmosphere::Standard {
                station_altitude_m,
                temperature_c,
                pressure_hpa,
                humidity_percent,
            }) = Atmosphere::icao_standard(altitude)
            else {
                panic!("standard station at {altitude} m");
            };
            let (t, p) = resolve_station_conditions(15.0, 1013.25, altitude);
            assert_eq!(station_altitude_m, altitude);
            assert_eq!(humidity_percent, 0.0);
            assert!(
                (temperature_c - t).abs() < 1e-9,
                "{altitude} m: {temperature_c} vs {t}"
            );
            assert!(
                (pressure_hpa - p).abs() < 1e-9 * p,
                "{altitude} m: {pressure_hpa} vs {p}"
            );
        }
    }

    #[test]
    fn standard_air_is_the_same_whatever_the_station_altitude() {
        // A standard station at 1500 m and one at sea level describe one
        // atmosphere. Temperature, so sound speed, is the same at every
        // altitude; density differs by the constant 2.5e-5 by which the
        // station's CIPM density departs from upstream's ideal-gas lapse.
        let pad = Air::new(Atmosphere::icao_standard(1500.0).unwrap()).unwrap();
        let sea = Air::new(Atmosphere::default()).unwrap();
        for y in [0.0, 1500.0, 3000.0, 10_000.0, 15_000.0] {
            let (rho_pad, c_pad) = pad.at(Vec3::new(0.0, y, 0.0));
            let (rho_sea, c_sea) = sea.at(Vec3::new(0.0, y, 0.0));
            assert!((rho_pad - rho_sea).abs() < 3e-5 * rho_sea, "rho at {y} m");
            assert!((c_pad - c_sea).abs() < 1e-12 * c_sea, "sound at {y} m");
        }
    }

    #[test]
    fn icao_standard_refuses_altitudes_outside_the_troposphere() {
        for altitude in [f64::NAN, f64::INFINITY, -1000.5, 11_000.5] {
            assert!(Atmosphere::icao_standard(altitude).is_err(), "{altitude}");
        }
    }

    #[test]
    fn wind_field_is_sampled_per_state() {
        let profile = winds_engine::ProfileField::new(vec![
            winds_engine::ProfileLevel {
                altitude_m: 0.0,
                speed_mps: 0.0,
                direction_from_deg: 0.0,
            },
            winds_engine::ProfileLevel {
                altitude_m: 1000.0,
                speed_mps: 10.0,
                direction_from_deg: 0.0,
            },
        ])
        .unwrap();
        let forces = BallisticForces::new(
            0.05,
            vec![(0.0, 0.4), (3.0, 0.4)],
            Atmosphere::default(),
            Vec3::zeros(),
        )
        .unwrap()
        .with_wind_field(std::sync::Arc::new(profile))
        .unwrap();
        let state = FlightState {
            time_s: 1.0,
            position_m: Vec3::new(0.0, 500.0, 0.0),
            velocity_mps: Vec3::zeros(),
        };
        // FROM 0° blows toward −X; 5 m/s at 500 m altitude.
        let wind = forces.effective_wind(&state);
        assert!((wind.x + 5.0).abs() < 1e-9, "wind {wind:?}");
        assert!(wind.y.abs() < 1e-12 && wind.z.abs() < 1e-9);
    }

    #[test]
    fn turbulence_adds_to_the_effective_wind() {
        let forces = BallisticForces::new(
            0.05,
            vec![(0.0, 0.4), (3.0, 0.4)],
            Atmosphere::default(),
            Vec3::new(3.0, 0.0, 0.0),
        )
        .unwrap()
        .with_turbulence(2.0, 2.0, 42)
        .unwrap();
        let state = FlightState {
            time_s: 1.7,
            position_m: Vec3::zeros(),
            velocity_mps: Vec3::zeros(),
        };
        let expected = Vec3::new(3.0, 0.0, 0.0) + forces.turbulence.as_ref().unwrap().gust_vec(1.7);
        assert_eq!(forces.effective_wind(&state), expected);
    }
}
