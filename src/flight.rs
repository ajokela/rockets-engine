use crate::{
    finite_vec, nonnegative, positive, ControlInput, Error, Guidance, GuidanceContext, MotorPhase,
    Result, Rocket, RocketSample, Vec3,
};

/// Complete translational state. Motor phase and fuel are derived from time and configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlightState {
    /// Seconds since the simulation epoch used by the motor schedule.
    pub time_s: f64,
    /// World position in metres; Y is up.
    pub position_m: Vec3,
    /// World velocity in metres per second.
    pub velocity_mps: Vec3,
}

impl FlightState {
    /// Check a state created directly or restored from a snapshot.
    pub fn validate(&self) -> Result<()> {
        nonnegative(self.time_s, "state.time_s")?;
        if !finite_vec(&self.position_m) || !finite_vec(&self.velocity_mps) {
            return Err(Error::invalid("state vectors", "must be finite"));
        }
        Ok(())
    }
}

/// Environmental acceleration hook (gravity, drag, wind, etc.), without motor or controls.
///
/// Called at every RK4 stage with the stage's position, velocity, time and current mass.
/// Implementations must be deterministic and side-effect free for reproducible integration.
pub trait ForceModel {
    /// Acceleration in world-frame metres per second squared.
    fn acceleration(&self, state: &FlightState, mass_kg: f64) -> Result<Vec3>;
}

impl<F> ForceModel for F
where
    F: Fn(&FlightState, f64) -> Result<Vec3>,
{
    fn acceleration(&self, state: &FlightState, mass_kg: f64) -> Result<Vec3> {
        self(state, mass_kg)
    }
}

/// Constant gravity with no atmosphere. Use a zero vector for an inertial vacuum.
#[derive(Debug, Clone, Copy)]
pub struct UniformGravity(pub Vec3);

impl Default for UniformGravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.80665, 0.0))
    }
}

impl ForceModel for UniformGravity {
    fn acceleration(&self, _state: &FlightState, _mass_kg: f64) -> Result<Vec3> {
        Ok(self.0)
    }
}

/// Integration accuracy, work budget and external control authority.
#[derive(Debug, Clone, Copy)]
pub struct IntegratorSettings {
    /// Maximum internal RK4 timestep in simulation seconds. Default: 0.01 s.
    pub max_step_s: f64,
    /// Maximum internal substeps in a single public call. Default: 100,000.
    pub max_substeps: usize,
    /// Magnitude cap for additional steering acceleration in m/s². Default: zero.
    /// This does not limit changes in the externally supplied thrust-axis orientation.
    pub max_steering_accel_mps2: f64,
}

impl Default for IntegratorSettings {
    fn default() -> Self {
        Self {
            max_step_s: 0.01,
            max_substeps: 100_000,
            max_steering_accel_mps2: 0.0,
        }
    }
}

impl IntegratorSettings {
    fn validate(&self) -> Result<()> {
        positive(self.max_step_s, "max_step_s")?;
        nonnegative(self.max_steering_accel_mps2, "max_steering_accel_mps2")?;
        if self.max_substeps == 0 {
            return Err(Error::invalid("max_substeps", "must be nonzero"));
        }
        Ok(())
    }
}

/// Result of a successful advancement. An error never overwrites the caller's input state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepResult {
    /// State at exactly the requested end time.
    pub state: FlightState,
    /// Mass and motor output at the end time (burnout has zero thrust).
    pub rocket: RocketSample,
    /// Number of internal RK4 steps performed.
    pub substeps: usize,
}

/// Stateless RK4 stepper over an immutable vehicle, force model and settings.
///
/// Motor ignition, each thrust knot and burnout split integration intervals so no step
/// straddles a motor discontinuity. Mass is evaluated analytically at every RK stage.
/// Collision detection and terrain termination belong to the host application.
#[derive(Debug, Clone)]
pub struct Simulator<E> {
    rocket: Rocket,
    forces: E,
    settings: IntegratorSettings,
}

impl<E: ForceModel> Simulator<E> {
    /// Create a reusable simulator, validating settings before any integration.
    pub fn new(rocket: Rocket, forces: E, settings: IntegratorSettings) -> Result<Self> {
        settings.validate()?;
        Ok(Self {
            rocket,
            forces,
            settings,
        })
    }

    /// The vehicle configuration needed alongside a snapshot to resume a flight.
    pub fn rocket(&self) -> &Rocket {
        &self.rocket
    }
    /// Current environmental force model.
    pub fn forces(&self) -> &E {
        &self.forces
    }
    /// Access the environment to update externally sampled weather between steps.
    pub fn forces_mut(&mut self) -> &mut E {
        &mut self.forces
    }
    /// Validated integrator settings.
    pub fn settings(&self) -> IntegratorSettings {
        self.settings
    }

    fn end_time(&self, state: &FlightState, duration_s: f64) -> Result<f64> {
        state.validate()?;
        positive(duration_s, "duration_s")?;
        let end = state.time_s + duration_s;
        if !end.is_finite() || end <= state.time_s {
            return Err(Error::invalid(
                "duration_s",
                "must advance time to a finite, distinguishable value",
            ));
        }
        if duration_s > self.settings.max_step_s * self.settings.max_substeps as f64 {
            return Err(Error::StepLimitExceeded {
                limit: self.settings.max_substeps,
            });
        }
        Ok(end)
    }

    /// Advance using an externally supplied command, held fixed for the requested duration.
    /// Call at a fixed cadence to obtain repeatable controller and integration behavior.
    pub fn step(
        &self,
        state: &FlightState,
        duration_s: f64,
        control: ControlInput,
    ) -> Result<StepResult> {
        let end = self.end_time(state, duration_s)?;
        let (axis, steering) = self.resolve_control(control)?;
        let mut current = *state;
        let mut substeps = 0;
        let mut grid_index = 1usize;
        while current.time_s < end {
            if substeps >= self.settings.max_substeps {
                return Err(Error::StepLimitExceeded {
                    limit: self.settings.max_substeps,
                });
            }
            // Anchor to the public start time instead of repeatedly adding dt. This
            // avoids an extra near-zero step from accumulated floating-point drift.
            // Motor events split a grid interval without resetting that grid.
            let grid_boundary =
                (state.time_s + grid_index as f64 * self.settings.max_step_s).min(end);
            let mut boundary = grid_boundary;
            if let Some(event) = self
                .rocket
                .motor()
                .and_then(|m| m.next_event_after(current.time_s))
            {
                boundary = boundary.min(event);
            }
            if boundary <= current.time_s {
                return Err(Error::NumericalFailure(
                    "internal timestep cannot advance time",
                ));
            }
            current = self.rk4(current, boundary, axis, steering)?;
            substeps += 1;
            if boundary == grid_boundary {
                grid_index += 1;
            }
        }
        Ok(StepResult {
            state: current,
            rocket: self.rocket.sample_unchecked(current.time_s),
            substeps,
        })
    }

    /// Evaluate guidance once, then advance under its held command.
    /// Invalid state/duration is rejected before calling guidance. A later model error
    /// leaves the input flight state unchanged but does not roll back the controller.
    pub fn step_guided<G: Guidance + ?Sized>(
        &self,
        state: &FlightState,
        duration_s: f64,
        guidance: &mut G,
    ) -> Result<StepResult> {
        self.end_time(state, duration_s)?;
        let command = guidance.command(GuidanceContext {
            state,
            rocket: self.rocket.sample_unchecked(state.time_s),
            step_duration_s: duration_s,
            max_steering_accel_mps2: self.settings.max_steering_accel_mps2,
        })?;
        self.step(state, duration_s, command)
    }

    fn resolve_control(&self, control: ControlInput) -> Result<(Vec3, Vec3)> {
        if !finite_vec(&control.thrust_direction)
            || !finite_vec(&control.steering_acceleration_mps2)
        {
            return Err(Error::invalid("control", "all components must be finite"));
        }
        // Scale first: ordinary norm() can overflow or underflow for finite vectors.
        let scale = control.thrust_direction.amax();
        if scale == 0.0 {
            return Err(Error::invalid("thrust_direction", "must be nonzero"));
        }
        let scaled = control.thrust_direction / scale;
        let axis = scaled / scaled.norm();
        let scale = control.steering_acceleration_mps2.amax();
        let steering = if scale == 0.0 || self.settings.max_steering_accel_mps2 == 0.0 {
            Vec3::zeros()
        } else {
            let scaled = control.steering_acceleration_mps2 / scale;
            let norm = scaled.norm();
            if scale > self.settings.max_steering_accel_mps2 / norm {
                scaled * (self.settings.max_steering_accel_mps2 / norm)
            } else {
                control.steering_acceleration_mps2
            }
        };
        Ok((axis, steering))
    }

    fn derivative(
        &self,
        state: FlightState,
        axis: Vec3,
        steering: Vec3,
        burning: bool,
    ) -> Result<(Vec3, Vec3)> {
        state
            .validate()
            .map_err(|_| Error::NumericalFailure("non-finite intermediate state"))?;
        let mass = self.rocket.sample_unchecked(state.time_s).mass_kg;
        let thrust = self
            .rocket
            .motor()
            .map_or(0.0, |m| m.interval_thrust(state.time_s, burning));
        let acceleration =
            self.forces.acceleration(&state, mass)? + axis * (thrust / mass) + steering;
        if !finite_vec(&acceleration) {
            return Err(Error::NumericalFailure("non-finite acceleration"));
        }
        Ok((state.velocity_mps, acceleration))
    }

    fn rk4(&self, start: FlightState, end: f64, axis: Vec3, steering: Vec3) -> Result<FlightState> {
        let dt = end - start.time_s;
        let burning = self.rocket.sample_unchecked(start.time_s).motor.phase == MotorPhase::Burning;
        let stage = |derivative: (Vec3, Vec3), delta: f64, time_s: f64| FlightState {
            time_s,
            position_m: start.position_m + derivative.0 * delta,
            velocity_mps: start.velocity_mps + derivative.1 * delta,
        };
        let midpoint = start.time_s + dt * 0.5;
        let k1 = self.derivative(start, axis, steering, burning)?;
        let k2 = self.derivative(stage(k1, dt * 0.5, midpoint), axis, steering, burning)?;
        let k3 = self.derivative(stage(k2, dt * 0.5, midpoint), axis, steering, burning)?;
        let k4 = self.derivative(stage(k3, dt, end), axis, steering, burning)?;
        let result = FlightState {
            time_s: end,
            position_m: start.position_m + (k1.0 + k2.0 * 2.0 + k3.0 * 2.0 + k4.0) * (dt / 6.0),
            velocity_mps: start.velocity_mps + (k1.1 + k2.1 * 2.0 + k3.1 * 2.0 + k4.1) * (dt / 6.0),
        };
        result
            .validate()
            .map_err(|_| Error::NumericalFailure("non-finite output state"))?;
        Ok(result)
    }
}
