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
/// delivered impulse / total impulse. There is no throttle, restart or staging in v0.1.
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

    pub(crate) fn next_event_after(&self, time_s: f64) -> Option<f64> {
        self.event_times_s
            .get(self.event_times_s.partition_point(|t| *t <= time_s))
            .copied()
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

/// Vehicle dry mass and its optional attached motor.
#[derive(Debug, Clone)]
pub struct Rocket {
    dry_mass_kg: f64,
    motor: Option<Motor>,
}

/// Motor output and total current vehicle mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RocketSample {
    /// Dry mass plus remaining propellant, in kilograms.
    pub mass_kg: f64,
    /// Current motor output.
    pub motor: MotorSample,
}

impl Rocket {
    /// Dry mass includes the empty motor casing. `None` creates a ballistic body.
    pub fn new(dry_mass_kg: f64, motor: Option<Motor>) -> Result<Self> {
        positive(dry_mass_kg, "dry_mass_kg")?;
        positive(
            dry_mass_kg + motor.as_ref().map_or(0.0, |m| m.propellant_mass_kg),
            "wet mass",
        )?;
        Ok(Self { dry_mass_kg, motor })
    }
    /// Mass after all propellant is spent.
    pub fn dry_mass_kg(&self) -> f64 {
        self.dry_mass_kg
    }
    /// Read-only access to the motor, if present.
    pub fn motor(&self) -> Option<&Motor> {
        self.motor.as_ref()
    }
    /// Evaluate mass and motor output at a finite nonnegative simulation time.
    pub fn sample(&self, time_s: f64) -> Result<RocketSample> {
        nonnegative(time_s, "time_s")?;
        Ok(self.sample_unchecked(time_s))
    }
    pub(crate) fn sample_unchecked(&self, time_s: f64) -> RocketSample {
        let motor = self.motor.as_ref().map_or(
            MotorSample {
                phase: MotorPhase::Unpowered,
                thrust_n: 0.0,
                propellant_mass_kg: 0.0,
            },
            |m| m.sample_unchecked(time_s),
        );
        RocketSample {
            mass_kg: self.dry_mass_kg + motor.propellant_mass_kg,
            motor,
        }
    }
}
