use crate::{FlightState, Result, RocketSample, Vec3};

/// World-frame controls held constant for one caller-requested simulation step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlInput {
    /// Nonzero thrust-axis vector. The solver normalizes it before applying thrust.
    /// This is an externally supplied orientation, not an instantaneous attitude solver.
    pub thrust_direction: Vec3,
    /// Additional acceleration requested by an external steering/actuator model.
    /// Its magnitude is capped by [`crate::IntegratorSettings::max_steering_accel_mps2`].
    /// It can act during coast, allowing callers to model fins or other control authority.
    pub steering_acceleration_mps2: Vec3,
}

impl ControlInput {
    /// Specify a thrust axis with no additional steering acceleration.
    /// Inputs are validated when the simulator steps.
    pub fn along(thrust_direction: Vec3) -> Self {
        Self {
            thrust_direction,
            steering_acceleration_mps2: Vec3::zeros(),
        }
    }
}

impl Default for ControlInput {
    fn default() -> Self {
        Self::along(Vec3::x())
    }
}

/// Snapshot supplied once to guidance for each [`crate::Simulator::step_guided`] call.
#[derive(Debug, Clone, Copy)]
pub struct GuidanceContext<'a> {
    /// State at the start of the requested step.
    pub state: &'a FlightState,
    /// Mass and motor phase at the start of the requested step.
    pub rocket: RocketSample,
    /// Caller-requested duration, not the smaller internal RK4 timestep.
    pub step_duration_s: f64,
    /// Solver-enforced cap on additional steering acceleration.
    pub max_steering_accel_mps2: f64,
}

/// Application-owned controller hook, independent of entities, targets and render timing.
///
/// Controllers are called once per public step, never inside RK4 stages. The command
/// is then held constant over every internal substep. Stateful controllers should save
/// their own state alongside [`FlightState`] and use a fixed update cadence for replay.
/// Controller mutations are not rolled back if subsequent integration fails.
pub trait Guidance {
    /// Produce a finite control command, or explain why control cannot be computed.
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput>;
}

impl<F> Guidance for F
where
    F: FnMut(GuidanceContext<'_>) -> Result<ControlInput>,
{
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput> {
        self(context)
    }
}

/// Owned, type-erased guidance handle for heterogeneous decorator chains.
/// A newtype rather than `Box<dyn Guidance>` so it cannot collide with the
/// blanket closure implementations.
pub struct BoxedGuidance(pub Box<dyn Guidance>);

impl std::fmt::Debug for BoxedGuidance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BoxedGuidance").finish_non_exhaustive()
    }
}

impl Guidance for BoxedGuidance {
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput> {
        self.0.command(context)
    }
}

/// Unguided orientation: thrust always follows the specified fixed world axis.
/// This does not simulate passive body alignment or attitude dynamics.
#[derive(Debug, Clone, Copy)]
pub struct FixedDirection(pub Vec3);

impl Guidance for FixedDirection {
    fn command(&mut self, _context: GuidanceContext<'_>) -> Result<ControlInput> {
        Ok(ControlInput::along(self.0))
    }
}

/// Passive gravity-turn orientation: thrust follows the current velocity vector.
/// At rest (or at an exact apex with zero horizontal speed) the fallback axis is
/// used until there is a measurable direction of motion.
#[derive(Debug, Clone, Copy)]
pub struct ProgradeDirection {
    /// Axis used while the speed is too small to define a direction.
    pub fallback_axis: Vec3,
}

impl ProgradeDirection {
    /// Fall back to this axis while stationary. Typically the launch direction.
    pub fn new(fallback_axis: Vec3) -> Self {
        Self { fallback_axis }
    }
}

impl Default for ProgradeDirection {
    fn default() -> Self {
        Self::new(Vec3::y())
    }
}

impl Guidance for ProgradeDirection {
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput> {
        let v = context.state.velocity_mps;
        let axis = if v.norm() > 1e-9 {
            v / v.norm()
        } else {
            self.fallback_axis
        };
        Ok(ControlInput::along(axis))
    }
}

/// A fixed thrust-axis misalignment, applied to any inner guidance command.
/// Models the small, constant boresight error that is the dominant dispersion
/// source for unguided rockets: `tilt_rad` is the angular offset magnitude and
/// `clock_rad` its orientation about the commanded axis. Zero tilt is
/// bit-identical to the inner command.
#[derive(Debug, Clone, Copy)]
pub struct ThrustMisalignment<G> {
    inner: G,
    tilt_rad: f64,
    clock_rad: f64,
    basis_u: Option<Vec3>,
}

impl<G> ThrustMisalignment<G> {
    /// Wrap an inner guidance source. `tilt_rad >= 0`; `clock_rad` measured
    /// about the commanded axis from an arbitrary but deterministic reference.
    pub fn new(inner: G, tilt_rad: f64, clock_rad: f64) -> Result<Self> {
        if !tilt_rad.is_finite() || tilt_rad < 0.0 || !clock_rad.is_finite() {
            return Err(crate::Error::invalid(
                "thrust misalignment",
                "tilt must be finite and nonnegative, clock finite",
            ));
        }
        Ok(Self {
            inner,
            tilt_rad,
            clock_rad,
            basis_u: None,
        })
    }

    /// Current reference-basis vector, for snapshot/replay continuity.
    pub fn basis_u(&self) -> Option<Vec3> {
        self.basis_u
    }

    /// Restore a previously snapshotted reference basis (or `None` to reseed).
    pub fn set_basis_u(&mut self, basis_u: Option<Vec3>) {
        self.basis_u = basis_u;
    }
}

impl<G: Guidance> Guidance for ThrustMisalignment<G> {
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput> {
        let command = self.inner.command(context)?;
        if self.tilt_rad == 0.0 {
            return Ok(command);
        }
        let axis = command.thrust_direction.normalize();
        // Parallel-transport the previous basis when possible; seed on the
        // first call or when the old basis degenerates against the new axis.
        let u = match self.basis_u {
            Some(previous) => {
                let transported = previous - axis * axis.dot(&previous);
                if transported.norm() > 1e-9 {
                    transported.normalize()
                } else {
                    let seed = if axis.z.abs() < 0.9 {
                        Vec3::z()
                    } else {
                        Vec3::y()
                    };
                    axis.cross(&seed).normalize()
                }
            }
            None => {
                let seed = if axis.z.abs() < 0.9 {
                    Vec3::z()
                } else {
                    Vec3::y()
                };
                axis.cross(&seed).normalize()
            }
        };
        self.basis_u = Some(u);
        let v = axis.cross(&u).normalize();
        let tilt_axis = u * self.clock_rad.cos() + v * self.clock_rad.sin();
        let misaligned = axis * self.tilt_rad.cos() + tilt_axis * self.tilt_rad.sin();
        Ok(ControlInput {
            thrust_direction: misaligned,
            steering_acceleration_mps2: command.steering_acceleration_mps2,
        })
    }
}

/// First-order actuator lag on the steering acceleration, as a stateful
/// guidance decorator. The thrust axis passes through untouched; the steering
/// vector relaxes toward the inner command with time constant `tau_s` using an
/// exact discrete update `current += (target - current)·(1 − exp(−dt/τ))`.
/// Save/restore [`Self::current`] alongside the flight state for replay.
#[derive(Debug, Clone, Copy)]
pub struct SteeringLag<G> {
    inner: G,
    tau_s: f64,
    current: Vec3,
}

impl<G> SteeringLag<G> {
    /// `tau_s` is the actuator time constant in seconds and must be positive.
    pub fn new(inner: G, tau_s: f64) -> Result<Self> {
        if !tau_s.is_finite() || tau_s <= 0.0 {
            return Err(crate::Error::invalid(
                "steering lag tau_s",
                "must be finite and positive",
            ));
        }
        Ok(Self {
            inner,
            tau_s,
            current: Vec3::zeros(),
        })
    }
    /// Current lagged steering vector (the state to snapshot).
    pub fn current(&self) -> Vec3 {
        self.current
    }
    /// Restore a previously snapshotted steering vector.
    pub fn set_current(&mut self, current: Vec3) {
        self.current = current;
    }
}

impl<G: Guidance> Guidance for SteeringLag<G> {
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput> {
        let mut command = self.inner.command(context)?;
        let weight = 1.0 - (-context.step_duration_s / self.tau_s).exp();
        self.current += (command.steering_acceleration_mps2 - self.current) * weight;
        command.steering_acceleration_mps2 = self.current;
        Ok(command)
    }
}
