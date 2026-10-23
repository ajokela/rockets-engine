use crate::{nonnegative, positive, Error, Result};

/// One measured or synthetic thrust sample, relative to motor ignition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustPoint {
    /// Seconds since ignition; the first point must be at zero.
    pub time_s: f64,
    /// Nonnegative force in newtons.
    pub thrust_n: f64,
}

/// Validated, piecewise-linear thrust curve with exact piecewise-quadratic impulse.
#[derive(Debug, Clone)]
pub struct ThrustCurve {
    points: Vec<ThrustPoint>,
    cumulative_impulse_ns: Vec<f64>,
}

impl ThrustCurve {
    /// Construct from at least two strictly time-ordered samples, starting at zero.
    /// Total impulse must be positive. Endpoint thrust need not be zero; the integrator
    /// explicitly handles the discontinuities at ignition and burnout.
    pub fn new(points: Vec<ThrustPoint>) -> Result<Self> {
        if points.len() < 2 || points[0].time_s != 0.0 {
            return Err(Error::invalid(
                "thrust curve",
                "needs at least two points starting at time zero",
            ));
        }
        let mut impulse = vec![0.0];
        for (i, point) in points.iter().enumerate() {
            nonnegative(point.time_s, "thrust point time_s")?;
            nonnegative(point.thrust_n, "thrust point thrust_n")?;
            if i > 0 {
                let previous = points[i - 1];
                let dt = point.time_s - previous.time_s;
                positive(dt, "thrust curve time spacing")?;
                let area = (previous.thrust_n * 0.5 + point.thrust_n * 0.5) * dt;
                let total = impulse[i - 1] + area;
                if !total.is_finite() {
                    return Err(Error::invalid(
                        "thrust curve",
                        "total impulse must be finite",
                    ));
                }
                impulse.push(total);
            }
        }
        positive(*impulse.last().unwrap(), "total impulse")?;
        Ok(Self {
            points,
            cumulative_impulse_ns: impulse,
        })
    }

    /// Read-only access to the validated samples.
    pub fn points(&self) -> &[ThrustPoint] {
        &self.points
    }

    /// Time of the last sample, in seconds after ignition.
    pub fn duration_s(&self) -> f64 {
        self.points.last().unwrap().time_s
    }

    /// Integral of thrust over the entire curve, in newton seconds.
    pub fn total_impulse_ns(&self) -> f64 {
        *self.cumulative_impulse_ns.last().unwrap()
    }

    /// Interpolated thrust. Returns zero outside the closed curve interval.
    /// At the final sample this returns the left-limit force; [`Motor::sample`]
    /// reports zero thrust at and after burnout instead.
    pub fn thrust_at(&self, elapsed_s: f64) -> Result<f64> {
        if !elapsed_s.is_finite() {
            return Err(Error::invalid("elapsed_s", "must be finite"));
        }
        Ok(self.thrust(elapsed_s))
    }

    /// Exact integrated impulse through this time, clamped to the curve interval.
    pub fn impulse_at(&self, elapsed_s: f64) -> Result<f64> {
        if !elapsed_s.is_finite() {
            return Err(Error::invalid("elapsed_s", "must be finite"));
        }
        Ok(self.impulse(elapsed_s))
    }

    fn segment(&self, elapsed_s: f64) -> usize {
        self.points
            .partition_point(|p| p.time_s <= elapsed_s)
            .saturating_sub(1)
            .min(self.points.len() - 2)
    }

    pub(crate) fn thrust(&self, elapsed_s: f64) -> f64 {
        if elapsed_s < 0.0 || elapsed_s > self.duration_s() {
            return 0.0;
        }
        let i = self.segment(elapsed_s);
        let a = self.points[i];
        let b = self.points[i + 1];
        let fraction = (elapsed_s - a.time_s) / (b.time_s - a.time_s);
        a.thrust_n * (1.0 - fraction) + b.thrust_n * fraction
    }

    fn impulse(&self, elapsed_s: f64) -> f64 {
        if elapsed_s <= 0.0 {
            return 0.0;
        }
        if elapsed_s >= self.duration_s() {
            return self.total_impulse_ns();
        }
        let i = self.segment(elapsed_s);
        let area = (self.points[i].thrust_n * 0.5 + self.thrust(elapsed_s) * 0.5)
            * (elapsed_s - self.points[i].time_s);
        (self.cumulative_impulse_ns[i] + area).min(self.total_impulse_ns())
    }
}

/// Motor lifecycle. A spent motor stays attached and contributes to dry mass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorPhase {
    /// The vehicle has no motor.
    Unpowered,
    /// Waiting for the scheduled ignition time.
    AwaitingIgnition,
    /// Inside the burn schedule, including any zero-thrust sections of the curve.
    Burning,
    /// Burn schedule complete; no thrust or propellant remains.
    BurnedOut,
}

/// Instantaneous motor output, derived entirely from configuration and simulation time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorSample {
    /// Current lifecycle phase.
    pub phase: MotorPhase,
    /// Thrust in newtons, before applying its commanded direction.
    pub thrust_n: f64,
    /// Remaining consumable propellant in kilograms.
    pub propellant_mass_kg: f64,
}

/// A single, prescribed motor burn with constant effective exhaust velocity.
///
/// Mass flow is proportional to thrust: consumed mass = initial propellant mass ×
/// delivered impulse / total impulse. There is no throttle or restart; multiple
/// burns are modeled as multiple [`Stage`]s on [`Rocket::staged`].
#[derive(Debug, Clone)]
pub struct Motor {
    curve: ThrustCurve,
    propellant_mass_kg: f64,
    ignition_time_s: f64,
    event_times_s: Vec<f64>,
}

impl Motor {
    /// Schedule a motor relative to the same simulation epoch as [`crate::FlightState`].
    pub fn new(curve: ThrustCurve, propellant_mass_kg: f64, ignition_time_s: f64) -> Result<Self> {
        positive(propellant_mass_kg, "propellant_mass_kg")?;
        nonnegative(ignition_time_s, "ignition_time_s")?;
        positive(
            curve.total_impulse_ns() / propellant_mass_kg,
            "effective exhaust velocity",
        )?;
        let event_times_s: Vec<_> = curve
            .points
            .iter()
            .map(|p| ignition_time_s + p.time_s)
            .collect();
        if event_times_s.iter().any(|t| !t.is_finite())
            || event_times_s.windows(2).any(|p| p[1] <= p[0])
        {
            return Err(Error::invalid(
                "motor schedule",
                "event times must be finite and distinguishable",
            ));
        }
        Ok(Self {
            curve,
            propellant_mass_kg,
            ignition_time_s,
            event_times_s,
        })
    }

    /// The immutable thrust curve.
    pub fn curve(&self) -> &ThrustCurve {
        &self.curve
    }
    /// Initial propellant mass in kilograms.
    pub fn propellant_mass_kg(&self) -> f64 {
        self.propellant_mass_kg
    }
    /// Ignition time relative to the flight epoch.
    pub fn ignition_time_s(&self) -> f64 {
        self.ignition_time_s
    }
    /// Exact scheduled burnout time relative to the flight epoch.
    pub fn burnout_time_s(&self) -> f64 {
        *self.event_times_s.last().unwrap()
    }

    /// Get phase, force and fuel at a finite nonnegative simulation time.
    pub fn sample(&self, time_s: f64) -> Result<MotorSample> {
        nonnegative(time_s, "time_s")?;
        Ok(self.sample_unchecked(time_s))
    }

    pub(crate) fn sample_unchecked(&self, time_s: f64) -> MotorSample {
        if time_s < self.ignition_time_s {
            return MotorSample {
                phase: MotorPhase::AwaitingIgnition,
                thrust_n: 0.0,
                propellant_mass_kg: self.propellant_mass_kg,
            };
        }
        if time_s >= self.burnout_time_s() {
            return MotorSample {
                phase: MotorPhase::BurnedOut,
                thrust_n: 0.0,
                propellant_mass_kg: 0.0,
            };
        }
        let elapsed = time_s - self.ignition_time_s;
        let fraction =
            (self.curve.impulse(elapsed) / self.curve.total_impulse_ns()).clamp(0.0, 1.0);
        MotorSample {
            phase: MotorPhase::Burning,
            thrust_n: self.curve.thrust(elapsed),
            propellant_mass_kg: self.propellant_mass_kg * (1.0 - fraction),
        }
    }

    // Called only within an interval split at every motor event. Keeping the same
    // phase through all four RK stages gives the correct one-sided endpoint force.
    pub(crate) fn interval_thrust(&self, time_s: f64, burning: bool) -> f64 {
        if !burning {
            return 0.0;
        }
        self.curve
            .thrust((time_s - self.ignition_time_s).clamp(0.0, self.curve.duration_s()))
    }
}

/// One separable stage: structural mass carried until a scheduled separation,
/// plus an optional motor with its own burn schedule.
///
/// Separation is right-continuous: at exactly `separation_time_s` the stage's
/// dry mass, motor and unspent propellant leave the vehicle and its motor stops
/// contributing thrust. A `None` separation keeps the stage attached for the
/// whole flight, so its empty casing remains part of the vehicle mass.
#[derive(Debug, Clone)]
pub struct Stage {
    dry_mass_kg: f64,
    motor: Option<Motor>,
    separation_time_s: Option<f64>,
}

impl Stage {
    /// A stage needs nonnegative dry mass and at least one of mass or a motor.
    /// Separation may fall before, during or after the motor's burn: dropping a
    /// burning stage (hot staging) ends its thrust, and dropping an unlit stage
    /// removes its unspent propellant along with the casing.
    pub fn new(
        dry_mass_kg: f64,
        motor: Option<Motor>,
        separation_time_s: Option<f64>,
    ) -> Result<Self> {
        nonnegative(dry_mass_kg, "stage dry_mass_kg")?;
        if dry_mass_kg == 0.0 && motor.is_none() {
            return Err(Error::invalid(
                "stage",
                "needs a positive dry mass or a motor",
            ));
        }
        if let Some(t) = separation_time_s {
            nonnegative(t, "stage separation_time_s")?;
        }
        Ok(Self {
            dry_mass_kg,
            motor,
            separation_time_s,
        })
    }

    /// Structural mass dropped at separation, in kilograms.
    pub fn dry_mass_kg(&self) -> f64 {
        self.dry_mass_kg
    }
    /// The stage's motor, if it has one.
    pub fn motor(&self) -> Option<&Motor> {
        self.motor.as_ref()
    }
    /// Scheduled separation time relative to the flight epoch; `None` never separates.
    pub fn separation_time_s(&self) -> Option<f64> {
        self.separation_time_s
    }

    fn attached(&self, time_s: f64) -> bool {
        self.separation_time_s.is_none_or(|s| time_s < s)
    }
}

/// Base structural mass plus an ordered set of stages.
///
/// Total mass is analytic in simulation time: the base mass, the dry mass of
/// every attached stage and the remaining propellant of every attached motor.
/// Thrust is the sum over attached, currently burning motors, so serial and
/// parallel (strap-on) staging share one model. Motor knots, ignitions,
/// burnouts and separations all split the integration grid.
#[derive(Debug, Clone)]
pub struct Rocket {
    base_mass_kg: f64,
    stages: Vec<Stage>,
    event_times_s: Vec<f64>,
}

/// Motor output and total current vehicle mass.
///
/// For a staged vehicle the motor fields are aggregates over every attached
/// motor: summed thrust, summed remaining propellant and a combined phase
/// (`Burning` if any attached motor burns, else `AwaitingIgnition` if any is
/// still scheduled, else `BurnedOut` if any motor existed, else `Unpowered`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RocketSample {
    /// Base mass plus attached stage dry mass and remaining propellant, in kilograms.
    pub mass_kg: f64,
    /// Aggregated motor output across attached stages.
    pub motor: MotorSample,
}

impl Rocket {
    /// A single-stage vehicle whose optional motor burns once and stays attached.
    /// Dry mass includes the empty motor casing. `None` creates a ballistic body.
    pub fn new(dry_mass_kg: f64, motor: Option<Motor>) -> Result<Self> {
        let stages = motor
            .map(|m| Stage {
                dry_mass_kg: 0.0,
                motor: Some(m),
                separation_time_s: None,
            })
            .into_iter()
            .collect();
        Self::assemble(dry_mass_kg, stages)
    }

    /// A multi-stage vehicle. `base_mass_kg` is the structure that never
    /// separates (payload, bus, final casing) and must be positive.
    pub fn staged(base_mass_kg: f64, stages: Vec<Stage>) -> Result<Self> {
        Self::assemble(base_mass_kg, stages)
    }

    fn assemble(base_mass_kg: f64, stages: Vec<Stage>) -> Result<Self> {
        positive(base_mass_kg, "base_mass_kg")?;
        let mut event_times_s = Vec::new();
        for stage in &stages {
            if let Some(motor) = &stage.motor {
                event_times_s.extend_from_slice(&motor.event_times_s);
            }
            if let Some(t) = stage.separation_time_s {
                event_times_s.push(t);
            }
        }
        event_times_s.sort_by(f64::total_cmp);
        Ok(Self {
            base_mass_kg,
            stages,
            event_times_s,
        })
    }

    /// Mass that never separates: for [`Rocket::new`] the full dry mass including
    /// the motor casing; for [`Rocket::staged`] the final mass after every stage
    /// has separated and all propellant is spent.
    pub fn dry_mass_kg(&self) -> f64 {
        self.base_mass_kg
    }
    /// The single permanent motor of a [`Rocket::new`] vehicle. Staged vehicles
    /// use [`Rocket::stages`]; this returns `Some` only for the single-motor shape.
    pub fn motor(&self) -> Option<&Motor> {
        match self.stages.as_slice() {
            [Stage {
                motor: Some(m),
                separation_time_s: None,
                ..
            }] => Some(m),
            _ => None,
        }
    }
    /// All stages in the order supplied at construction.
    pub fn stages(&self) -> &[Stage] {
        &self.stages
    }
    /// Evaluate mass and aggregated motor output at a finite nonnegative time.
    pub fn sample(&self, time_s: f64) -> Result<RocketSample> {
        nonnegative(time_s, "time_s")?;
        Ok(self.sample_unchecked(time_s))
    }
    pub(crate) fn sample_unchecked(&self, time_s: f64) -> RocketSample {
        let mut mass_kg = self.base_mass_kg;
        let mut thrust_n = 0.0;
        let mut propellant_mass_kg = 0.0;
        let mut any_motor = false;
        let mut any_burning = false;
        let mut any_awaiting = false;
        for stage in &self.stages {
            if !stage.attached(time_s) {
                continue;
            }
            mass_kg += stage.dry_mass_kg;
            if let Some(motor) = &stage.motor {
                any_motor = true;
                let sample = motor.sample_unchecked(time_s);
                thrust_n += sample.thrust_n;
                propellant_mass_kg += sample.propellant_mass_kg;
                match sample.phase {
                    MotorPhase::Burning => any_burning = true,
                    MotorPhase::AwaitingIgnition => any_awaiting = true,
                    _ => {}
                }
            }
        }
        let phase = if !any_motor {
            MotorPhase::Unpowered
        } else if any_burning {
            MotorPhase::Burning
        } else if any_awaiting {
            MotorPhase::AwaitingIgnition
        } else {
            MotorPhase::BurnedOut
        };
        RocketSample {
            mass_kg: mass_kg + propellant_mass_kg,
            motor: MotorSample {
                phase,
                thrust_n,
                propellant_mass_kg,
            },
        }
    }

    pub(crate) fn next_event_after(&self, time_s: f64) -> Option<f64> {
        self.event_times_s
            .get(self.event_times_s.partition_point(|t| *t <= time_s))
            .copied()
    }

    /// Per-stage state frozen at an integration interval's start. Intervals
    /// never straddle a motor event or a separation, so these flags hold
    /// through all four RK stages and yield the correct one-sided mass and
    /// force at discontinuities: at a separation the closing RK stage still
    /// sees the departing stage's mass and thrust (the left limit), and the
    /// next interval opens without them (the right limit).
    pub(crate) fn interval_stages(&self, time_s: f64) -> Vec<IntervalStage> {
        self.stages
            .iter()
            .map(|stage| {
                let attached = stage.attached(time_s);
                let burning = attached
                    && stage
                        .motor
                        .as_ref()
                        .is_some_and(|m| m.sample_unchecked(time_s).phase == MotorPhase::Burning);
                IntervalStage { attached, burning }
            })
            .collect()
    }

    /// Vehicle mass at `time_s` under interval-frozen attachment: base mass,
    /// attached stages' dry mass and their motors' remaining propellant.
    /// Propellant varies within the interval; attachment does not.
    pub(crate) fn interval_mass(&self, time_s: f64, stages: &[IntervalStage]) -> f64 {
        self.base_mass_kg
            + self
                .stages
                .iter()
                .zip(stages)
                .map(|(stage, &flags)| {
                    if !flags.attached {
                        0.0
                    } else {
                        stage.dry_mass_kg
                            + stage
                                .motor
                                .as_ref()
                                .map_or(0.0, |m| m.sample_unchecked(time_s).propellant_mass_kg)
                    }
                })
                .sum::<f64>()
    }

    /// Total thrust at `time_s` under the interval-start burning flags.
    pub(crate) fn interval_thrust(&self, time_s: f64, stages: &[IntervalStage]) -> f64 {
        self.stages
            .iter()
            .zip(stages)
            .map(|(stage, &flags)| {
                if flags.burning {
                    stage
                        .motor
                        .as_ref()
                        .map_or(0.0, |m| m.interval_thrust(time_s, true))
                } else {
                    0.0
                }
            })
            .sum()
    }
}

/// Interval-frozen attachment and burning state for one stage; see
/// [`Rocket::interval_stages`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct IntervalStage {
    pub(crate) attached: bool,
    pub(crate) burning: bool,
}
