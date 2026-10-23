# rockets-engine

A Rust companion to [`ballistics-engine`](https://crates.io/crates/ballistics-engine) for powered, three-dimensional point-mass flight. It adds prescribed motor burns, changing mass, incremental RK4 integration and application-owned guidance hooks. The default adapter calls the existing ballistics-engine gravity/drag derivative function; it does not fork or modify that engine.

The crate is published on crates.io and consumes `ballistics-engine` 0.48.0 from crates.io. Rust 1.99.1 is pinned for development, matching the game workspace.

## Use

```toml
[dependencies]
rockets-engine = "0.1"
```

## Run

```sh
cargo test --locked --all-features
cargo run --locked --example boost_coast
cargo run --locked --example guidance_hook
cargo doc --locked --no-deps --open
```

`boost_coast` prints CSV for a synthetic small rocket through ignition, powered flight and coast. Its numbers are illustrative. `guidance_hook` demonstrates externally supplied control acceleration without a target-tracking algorithm.

## Basic use

```rust
use rockets_engine::{
    ControlInput, FlightState, IntegratorSettings, Motor, Rocket,
    Simulator, ThrustCurve, ThrustPoint, UniformGravity, Vec3,
};

let curve = ThrustCurve::new(vec![
    ThrustPoint { time_s: 0.0, thrust_n: 0.0 },
    ThrustPoint { time_s: 0.1, thrust_n: 20.0 },
    ThrustPoint { time_s: 0.9, thrust_n: 15.0 },
    ThrustPoint { time_s: 1.0, thrust_n: 0.0 },
])?;
let motor = Motor::new(curve, 0.04, 0.2)?; // 40 g propellant; ignition at t=0.2 s
let rocket = Rocket::new(0.18, Some(motor))?; // Dry mass includes the motor casing
let simulator = Simulator::new(
    rocket,
    UniformGravity::default(), // Gravity only; see the atmospheric adapter below
    IntegratorSettings::default(),
)?;
let state = FlightState {
    time_s: 0.0,
    position_m: Vec3::new(0.0, 2.0, 0.0),
    velocity_mps: Vec3::new(2.0, 8.0, 0.0),
};
let result = simulator.step(
    &state,
    0.05,
    ControlInput::along(Vec3::new(0.3, 1.0, 0.0)),
)?;
assert_eq!(result.state.time_s, 0.05);
# Ok::<(), rockets_engine::Error>(())
```

To use the same aerodynamic force kernel as the game, replace `UniformGravity` with `BallisticForces`:

```rust
# #[cfg(feature = "ballistics")]
# {
use rockets_engine::{ballistics::{Atmosphere, BallisticForces}, Vec3};
let forces = BallisticForces::new(
    0.025, // Diameter in metres
    vec![(0.0, 0.45), (0.8, 0.5), (1.2, 0.7), (3.0, 0.4)], // Actual Mach/Cd pairs
    Atmosphere::default(),
    Vec3::new(2.0, 0.0, 0.0), // Air velocity: positive X is a tailwind
)?;
# let _ = forces;
# }
# Ok::<(), rockets_engine::Error>(())
```

## Model and units

- All values use SI units: metres, seconds, kilograms, newtons and m/s². Time is simulation time. Any game time compression belongs outside this crate.
- The fixed right-handed frame is X downrange, Y up, Z lateral. With `BallisticForces`, position Y is absolute altitude above sea level; the weather station has its own altitude. Launch elevation is encoded in velocity and thrust direction, not a rotating coordinate frame.
- Thrust is piecewise linear in time since ignition. Curves must start at zero time, have at least two samples, strictly increase in time and have positive total impulse. Nonzero endpoint thrust is supported.
- Fuel consumption is proportional to delivered impulse. If `I(t)` is accumulated impulse, `I_total` is total impulse and `mp` is initial propellant mass, remaining fuel is `mp × (1 − I(t)/I_total)`. This assumes constant effective exhaust velocity `I_total/mp`; the prescribed thrust already includes exhaust/pressure effects. There is no separate motor pressure compensation.
- Mass is computed analytically at every RK4 stage rather than numerically integrating and clamping an independent fuel state. Dry mass is positive and fuel cannot become negative. At burnout thrust is zero and the empty motor remains attached.
- Ignition, every curve knot and burnout split the integration grid. RK4 uses the appropriate one-sided force at discontinuities, including burns shorter than the maximum timestep.
- The default maximum internal step is 0.01 seconds. This is fixed-step RK4, not an adaptive solver; choose a timestep suitable for the forces and confirm convergence for new vehicle models.

This baseline follows the thrust-curve, propellant-mass, gravity and drag structure illustrated by [NASA's model rocket simulation](https://github.com/nasa/trick/blob/master/trick_sims/SIM_rocket/Readme.md). The force model is consistent with NASA's [rocket thrust equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-thrust-equation/).

## Guidance and other forces

`Guidance::command(GuidanceContext)` returns a `ControlInput`. A closure can implement the hook directly. The context contains the current flight state, motor output, mass, requested duration and configured steering limit.

- Guidance is called once per `step_guided` call. Its command is held constant during that call's internal RK4 stages and substeps.
- `thrust_direction` is normalized. It describes externally supplied orientation; it does not model attitude, angular velocity, gimbal travel or turning rate. `FixedDirection` provides a simple unguided orientation.
- `steering_acceleration_mps2` is a world-frame acceleration supplied by an external actuator/control model. The solver caps its vector magnitude using `max_steering_accel_mps2`, which defaults to zero. It may act during coast; its physical availability and direction are the caller's responsibility.
- The core has no target, seeker, interception or Godot entity types. Controllers can own their own state and inputs without changing the integrator.
- `ForceModel::acceleration` supplies environmental acceleration at each RK4 stage with that stage's current mass. A closure can implement this hook too. Keep it deterministic and free of side effects. `BallisticForces::acceleration_in_wind` supports wrappers that sample spatial or time-dependent wind.

The atmospheric adapter uses the body's own Cd-versus-Mach curve and updates both SI mass and the engine's imperial mass mirror. It includes gravity and drag; bullet-specific spin, yaw, Magnus, Coriolis and postprocessing effects are disabled. Standard atmosphere delegates to the upstream altitude model: humidity affects density, while this derivative path uses dry-air sound speed. Uniform atmosphere obeys the upstream direct-mode limits (`0 ≤ density < 2 kg/m³`, sound speed `> 200 m/s`); a zero density retains gravity but removes drag. `UniformGravity(Vec3::zeros())` gives an inertial vacuum.

## Save, replay and host integration

`Simulator` does not hold evolving flight state. Each step returns a new `FlightState`, end-of-step motor/mass telemetry and its internal step count. An error leaves the caller's state untouched. Invalid numeric inputs and non-finite results return errors, and each call has a bounded substep budget.

The default `serde` feature serializes `FlightState` and `MotorPhase`. To resume a flight, restore the state with the same motor schedule, vehicle, environment and solver settings. Motor fuel/phase are derived from simulation time, so they cannot drift apart in a snapshot. Save any stateful guidance controller separately. Invalid deserialized states are rejected when stepped. A controller's mutations are not rolled back if integration fails after the guidance call.

For deterministic replay, retain the same fixed public update cadence, control inputs and environment history. Internal grids are anchored to each public step; changing how calls are partitioned can change floating-point results. Cross-platform bit-for-bit identity is not promised.

For Counterbattery, a future `CbBallistics` adapter can repeatedly call `step` to precompute an unguided powered arc, or call `step_guided` at the simulation cadence for live control. Convert world coordinates once at that boundary, especially absolute altitude and side-facing X. The host must detect terrain and unit collisions over each returned movement interval; the library does not stop on a ground plane. No game integration is included in this crate.

## Scope of 0.1

The engine models translation with one prescribed motor burn. It does not yet model throttle/shutdown/restart, staging, launch rails, changing drag geometry, body attitude, lift coefficients, actuator dynamics, seekers or terrain collision localization. Guidance hooks provide the boundary for future control models; they are not a built-in autopilot.

## Validation

The tests check the ideal rocket equation and analytic position in vacuum, RK4 convergence, delayed ignition, nonzero endpoint thrust, short burns, zero-thrust intervals, fuel bounds, wind/mass/drag response, control limits, invalid inputs, work limits, and snapshot replay. Unpowered trajectories are compared against the upstream RK4 solver across multiple vehicle, wind and atmosphere configurations. These checks establish numerical behavior and compatibility, not empirical validation of a particular real motor or airframe.

```sh
cargo fmt --all -- --check
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo test --locked --all-features
cargo test --locked --no-default-features
cargo test --locked --no-default-features --features ballistics
cargo test --locked --no-default-features --features serde
```

`ballistics` enables the companion adapter and its `ballistics-engine` dependency. `serde` enables snapshot serialization. Disabling both leaves the integrator, motor, guidance and custom force hooks usable on their own.

Licensed under MIT; see the repository's `LICENSE` file.
