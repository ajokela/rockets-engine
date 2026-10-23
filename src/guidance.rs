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

/// Unguided orientation: thrust always follows the specified fixed world axis.
/// This does not simulate passive body alignment or attitude dynamics.
#[derive(Debug, Clone, Copy)]
pub struct FixedDirection(pub Vec3);

impl Guidance for FixedDirection {
    fn command(&mut self, _context: GuidanceContext<'_>) -> Result<ControlInput> {
        Ok(ControlInput::along(self.0))
    }
}
