# rockets-engine

[![crates.io](https://img.shields.io/crates/v/rockets-engine.svg)](https://crates.io/crates/rockets-engine)
[![docs.rs](https://docs.rs/rockets-engine/badge.svg)](https://docs.rs/rockets-engine)
[![CI](https://github.com/ajokela/rockets-engine/actions/workflows/ci.yml/badge.svg)](https://github.com/ajokela/rockets-engine/actions/workflows/ci.yml)

A Rust companion to [`ballistics-engine`](https://crates.io/crates/ballistics-engine) for powered, three-dimensional point-mass flight. It adds prescribed motor burns, changing mass, incremental RK4 integration and application-owned guidance hooks. The default adapter calls the existing ballistics-engine gravity/drag derivative function; it does not fork or modify that engine.

The crate is published on crates.io and consumes `ballistics-engine` 0.48.0 from crates.io. Rust 1.99.1 is pinned for development, matching the game workspace.

## Use

```toml
[dependencies]
rockets-engine = "0.13"
```

## Run

```sh
cargo test --locked --all-features
cargo run --locked --example boost_coast
cargo run --locked --example guidance_hook
cargo run --locked --example staging
cargo run --locked --example icbm_midcourse
cargo doc --locked --no-deps --open
```

`boost_coast` prints CSV for a synthetic small rocket through ignition, powered flight and coast. Its numbers are illustrative. `guidance_hook` demonstrates externally supplied control acceleration without a target-tracking algorithm. `staging` flies a three-stage rocket through separations and inter-stage coasts. `icbm_midcourse` coasts a post-boost state around a central-gravity Earth to a reentry handoff; it needs no features and shows the planet-scale use of the integrator.

## Command line

The `cli` feature builds a `rockets` binary:

```sh
cargo install rockets-engine --features cli
rockets simulate --curve 0:0,0.1:20,0.9:15,1:0 --dry-mass 0.18 --propellant-mass 0.04 --ignition-s 0.2 --plot
```

`simulate` prints the trajectory as a table, CSV or JSON, with an optional ASCII altitude chart and an apogee/impact summary. Add `--diameter-m` and `--cd` to fly against the ballistics-engine atmosphere and drag kernel, `--wind` for moving air, and `--elevation-deg`/`--azimuth-deg`/`--speed-mps` to aim the launch. `rockets help` lists every option.

Two analysis commands build on the same options:

- `dispersion` runs Monte Carlo samples with lognormal Cd-scale, per-axis wind and fractional speed perturbations (`--runs`, `--seed`, `--cd-sigma`, `--wind-sigma`, `--speed-sigma`) and reports apogee/range statistics and impact CEP. Deterministic per seed; no external RNG dependency.
- `solve-range <metres>` bisects launch elevation to hit a downrange ground target on the low-angle branch (`--high-angle` for the other solution) and prints the elevation, range, apogee and impact speed.

`--stage` builds serial or parallel staged vehicles from the terminal: each `--stage dry=kg,prop=kg,curve=t:N;t:N,ignition=s,sep=s` adds a stage with its own motor schedule and optional separation. `--gravity central|vacuum` and `--space-above-m` cover central-gravity and atmosphere/space-switching flights.

## Browser terminal

The same command surface runs at [rockets.rs/sh/](https://rockets.rs/sh/), compiled to WebAssembly. `src/wasm.rs` is a thin binding over the shared parser in `src/command.rs`, so the native and browser builds accept identical commands; `scripts/build-wasm.sh` rebuilds the module.

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
- For local flight the fixed right-handed frame is X downrange, Y up, Z lateral. With `BallisticForces`, position Y is absolute altitude above sea level; the weather station has its own altitude. Launch elevation is encoded in velocity and thrust direction, not a rotating coordinate frame.
- For planet-scale flight, integrate in an Earth-centered frame with `CentralGravity`: position measured from Earth's center, inverse-square gravity in both magnitude and direction. `icbm_midcourse` shows the pattern. The flat-frame models (`UniformGravity`, `BallisticForces`) are invalid at intercontinental ranges; `CentralGravity` is invalid (singular) at its own center and carries no atmosphere.
- Thrust is piecewise linear in time since ignition. Curves must start at zero time, have at least two samples, strictly increase in time and have positive total impulse. Nonzero endpoint thrust is supported.
- Fuel consumption is proportional to delivered impulse. If `I(t)` is accumulated impulse, `I_total` is total impulse and `mp` is initial propellant mass, remaining fuel is `mp × (1 − I(t)/I_total)`. This assumes constant effective exhaust velocity `I_total/mp`; the prescribed thrust already includes exhaust/pressure effects. There is no separate motor pressure compensation.
- Mass is computed analytically at every RK4 stage rather than numerically integrating and clamping an independent fuel state. Dry mass is positive and fuel cannot become negative. At burnout thrust is zero and the empty motor remains attached.
- A `Rocket` is a base mass plus any number of `Stage`s, each with its own motor schedule and an optional separation time. Total thrust sums over attached burning motors, so serial and parallel (strap-on) staging share one model. At separation the stage's dry mass, motor and unspent propellant leave the vehicle; separating a burning stage (hot staging) ends its thrust. `Rocket::new` keeps the 0.1 single-motor shape.
- Ignition, every curve knot, burnout and every stage separation split the integration grid. RK4 uses the appropriate one-sided mass and force at discontinuities: a step closing on a separation still sees the departing stage, and the next step opens without it, including burns shorter than the maximum timestep.
- The default maximum internal step is 0.01 seconds, sized for motor burns. In vacuum coast (midcourse) raise it — seconds per step is plenty under central gravity; the Kepler-invariant tests integrate orbits at 1–2 s steps. This is fixed-step RK4, not an adaptive solver; choose a timestep suitable for the forces and confirm convergence for new vehicle models.

This baseline follows the thrust-curve, propellant-mass, gravity and drag structure illustrated by [NASA's model rocket simulation](https://github.com/nasa/trick/blob/master/trick_sims/SIM_rocket/Readme.md). The force model is consistent with NASA's [rocket thrust equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-thrust-equation/).

## Guidance and other forces

`Guidance::command(GuidanceContext)` returns a `ControlInput`. A closure can implement the hook directly. The context contains the current flight state, motor output, mass, requested duration and configured steering limit.

- Guidance is called once per `step_guided` call. Its command is held constant during that call's internal RK4 stages and substeps.
- `thrust_direction` is normalized. It describes externally supplied orientation; it does not model attitude, angular velocity, gimbal travel or turning rate. `FixedDirection` provides a simple unguided orientation.
- `steering_acceleration_mps2` is a world-frame acceleration supplied by an external actuator/control model. The solver caps its vector magnitude using `max_steering_accel_mps2`, which defaults to zero. It may act during coast; its physical availability and direction are the caller's responsibility.
- The core has no target, seeker, interception or Godot entity types. Controllers can own their own state and inputs without changing the integrator.
- `ForceModel::acceleration` supplies environmental acceleration at each RK4 stage with that stage's current mass. A closure can implement this hook too. Keep it deterministic and free of side effects. `BallisticForces::acceleration_in_wind` supports wrappers that sample spatial or time-dependent wind.

### Guidance and constraint composition

- `ProgradeDirection` follows the velocity vector (gravity turns), with a fallback axis at rest.
- `ThrustMisalignment` decorates any guidance source with a fixed tilt/clock boresight error — the dominant real-world dispersion term for unguided rockets.
- `SteeringLag` adds a first-order actuator time constant to steering acceleration; its lagged state is snapshot-able for replay.
- `RailForces` wraps any force model and projects environmental acceleration onto a launch rail until exit; command the thrust axis along the rail and no steering until exit, then let tip-off take over.
- `BoxedGuidance`/`BoxedForces` are type-erased handles for heterogeneous decorator/model chains.
- `with_atmosphere_zones` and `with_wind_segments` on `BallisticForces` expose the upstream downrange-segmented atmosphere and wind ladders, evaluated statelessly at each stage's downrange position.
- `with_turbulence(sigma, tau, seed)` adds deterministic frozen-field gusts: eight multisine components per axis with a pink (1/f) spectrum, RMS `sigma` per axis and energy near the period `tau` (default 2 s via `--turbulence-tau`). Stateless, so RK4 substeps, snapshots, replays and `--seed` reproduction all see identical wind at identical times; dispersion reseeds per run so every Monte-Carlo sample gets its own gust realization. The weathercock and the recovery descent take the same gust field and the same wind-zone ladder, so attitude and drift respond to the same air the drag sees. CLI: `--turbulence <sigma_mps>`.

The atmospheric adapter uses the body's own Cd-versus-Mach curve and updates both SI mass and the engine's imperial mass mirror. It includes gravity and drag; bullet-specific reference-BC and postprocessing effects (Litz spin drift) are disabled. Standard atmosphere delegates to the upstream altitude model: humidity affects density, while this derivative path uses dry-air sound speed. Uniform atmosphere obeys the upstream direct-mode limits (`0 ≤ density < 2 kg/m³`, sound speed `> 200 m/s`); a zero density retains gravity but removes drag. `UniformGravity(Vec3::zeros())` gives an inertial vacuum.

### Reduced-order dynamics (weathercock and roll)

`Weathercock` is a second-order body-axis model in the guidance layer: the axis relaxes toward the relative wind with natural frequency ωₙ² = q·A·Cnα·(CP−CG)/I and damping 2ζωₙ, and the filtered axis becomes the commanded thrust axis. Positive static margin stabilizes; negative margin diverges. `WeathercockParams` takes the Cnα and CP Mach tables, a two-point CG model (dry + propellant references, mass-weighted as propellant burns), diameter, scalar pitch inertia and damping ratio. Axis and rate are snapshot-able state, updated once per public step, so choose the cadence well below the natural period. Pair with `with_body_lift` to close the loop through angle of attack. On a launch rail the axis is pinned (the rod constrains attitude, so a standing wind no longer tips the rocket on the pad). CLI: `--weathercock` with `--cna`, `--cp`, `--cg-dry-m`, `--cg-prop-m`, `--pitch-inertia`, `--damping-ratio`.

`RollDynamics` is the scalar roll channel: fin-cant drive against roll damping, with an explicit equilibrium rate. Feed the resulting effective twist to `BallisticForces::set_twist` between public steps (pair with `with_spin` for length and handedness). Still not 6-DOF: no quaternion, no inertia tensor, no gyroscopic coupling.

### Geometry to aerodynamics

A Barrowman modeler, implemented against the OpenRocket technical documentation and its `FinSetCalc`/`SymmetricComponentCalc` sources. `RocketGeometry` (nose cone + body tube + trapezoidal fins) yields `analyze()`: Cd/Cnα/CP curves and the reference area, with `forces()` and `weathercock_params()` convenience builders. CP is the component moment sum (nose volume fraction + fin MAC fraction, with the supersonic aft shift); drag is Schoenherr/roughness-limited skin friction with fineness and fin-thickness corrections, base drag, rounded-fin form drag, and conical/ogive nose wave drag. Fin normal force is Diederich subsonic and 2·A/(β·A_ref) supersonic with OpenRocket's Hermite join across the transonic region, scaled by body and fin-count interference factors.

Conical boat-tails (`boat=L:foreD:aftD` in the spec) join the geometry: Barrowman transition Cnα (negative for a shrinking diameter), frustum planform CP, OpenRocket's taper pressure drag, and base drag on the reduced aft area; `--geometry-masses` takes an optional fourth mass for the shell. Limits, deliberately: conical/ogive/elliptical/parabolic noses, trapezoidal fins, conical boat-tails only (no shoulders or multi-diameter bodies), no fin-body interference drag. Nose wave drag covers all four shapes past Mach 1 (NASA TR-R-100 tables for elliptical/parabolic). Induced drag from angle of attack uses the normal-force tilt model (Cd += Cnα·α², OpenRocket's convention). CLI: `--geometry nose=shape:L:D,tube=L:D,fins=N:cr:ct:s:m:t` with `m` the leading-edge sweep and `t` the fin thickness (metres).

The Cnα/CP output is validated against OpenRocket's own test suite (`tests/openrocket_parity.rs` reproduces BarrowmanCalculatorTest's Estes Alpha III values, three- and four-fin variants, within half a percent), and the whole flight is validated end to end: OpenRocket 24.12 flies its Alpha III fixture on its A8 motor to 409.6 m, and `openrocket_flight_parity_alpha_iii_a8` requires our apogee within 10 % of that (we fly 442.8 m). `scripts/openrocket-parity/` reproduces the OpenRocket number and documents the configuration.

`RocketGeometry::mass_properties(nose_kg, tube_kg, fins_kg, casing)` completes the picture: it combines the component masses with geometry-derived CGs (solid-of-revolution nose, thin-walled tube, fin planform centroid, motor casing at the aft end) into a dry CG and a pitch-inertia estimate about that CG. CLI: `--geometry-masses nose,tube,fins` (kg) replaces `--dry-mass` and estimates `--cg-dry-m`, `--cg-prop-m` and `--pitch-inertia`, so a full weathercock flight needs only the geometry, the masses and a motor file.

### Model rockets

The `model` feature (default on) adds hobbyist conveniences:

- `MotorFile` parses RASP `.eng` motor text — both layouts: canonical thrustcurve.org form (designation first, manufacturer last, curves starting after t = 0 with the origin prepended) and the inline manufacturer-first form. `motor(ignition_s)` schedules it. `--motor` picks a built-in NAR-data curve (estes:a8/b6/c6/d12, aerotech:g80 — real published curves with provenance); `--motor-eng` takes inline text; `--motor-file` reads from disk in the native CLI.
- `RecoveryForces` wraps any force model and swaps in recovery drag once the vehicle is descending (strictly: `vy < 0`, so a standing wind at liftoff cannot trip the chute on the pad), with an optional absolute-time deployment gate (`--recover <cdA>`, `--recovery-after-s <t>`). Terminal velocity `sqrt(2mg/(ρ·CdA))` emerges naturally. Deployment is a pure function of state, so integration stays deterministic. Dual deployment adds a main parachute that swaps in below an altitude (`with_main`, `--recover-main <cdA> --main-altitude-m <m>`). The descent samples the same wind zones and turbulence as the ascent.
- `Airframe::sport_24mm()` and `Airframe::high_power_54mm()` are *illustrative* presets (not measured kit data): diameter, Cd, Cnα, CP, CG, pitch inertia, induced-k and recovery Cd·A in one bundle. `--airframe sport24` or `hp54` fills the same options and composes with `--weathercock`.

### Modified point-mass

`BallisticForces` climbs one rung above point-mass through three opt-in builders, all evaluated in-loop by the upstream kernel:

- `with_spin(twist_rate_in, body_length_m, right_hand)` — Magnus force and yaw of repose for a spinning body. `twist_rate_in` is rifling language (inches per turn); for a fin-canted rocket it is the roll rate the cant produces per unit velocity. The correlations are McCoy bullet shapes, so treat magnitudes as approximate for blunt or finned bodies; signs and scaling are physical.
- `with_coriolis(latitude_deg, shot_azimuth_deg)` — rotating-frame deflection for planet-scale arcs, exact `−2Ω×v`. Do not combine with an inertial-frame `CentralGravity` model, which already accounts for Earth rotation by not rotating.
- `with_tipoff_yaw(yaw_rad, decay_distance_m)` — rail tip-off: an initial yaw decaying exponentially with downrange distance, perturbing the early angle of attack.

The CLI exposes the same surface: `--twist-in`/`--body-length-m`/`--twist-left`, `--coriolis lat:az`, `--tipoff yaw:decay`.

### Angle-of-attack aerodynamics

`with_body_lift(cna_vs_mach, induced_k_vs_mach)` adds quasi-static body lift without integrating attitude. The solver now hands every RK stage's commanded thrust axis to the force model through `ForceModel::acceleration_with_axis` (a provided method — orientation-blind models are bit-identical and existing implementations keep working). The adapter measures the angle of attack between that axis and the wind-relative velocity and adds:

- **Normal force** `N = q·A·Cnα(Mach)·α`, perpendicular to the relative wind on the axis side — a guided turn at angle of attack turns harder because the airframe works.
- **Induced drag** `Cd += k(Mach)·α²` — the same turn costs speed.

`Cnα` is per radian (body + fins), `k` per radian squared; both are piecewise-linear Mach tables needing two or more ascending points. Reference area derives from the diameter. CLI: `--cna mach:cna,... --lift-k mach:k,...`. This is still prescribed-attitude point-mass — the axis is commanded, not earned; trim, weathercock lag and stability margins belong to a future reduced-order layer.

The upstream ICAO atmosphere is defined to about 80 km; do not fly `BallisticForces` above it. Wrap it in `AltitudeSwitch` with a vacuum model above a cutoff (`UniformGravity` for flat-frame flight, `CentralGravity` for Earth-centered arcs) when a trajectory leaves the atmosphere.

## Save, replay and host integration

`Simulator` does not hold evolving flight state. Each step returns a new `FlightState`, end-of-step motor/mass telemetry and its internal step count. An error leaves the caller's state untouched. Invalid numeric inputs and non-finite results return errors, and each call has a bounded substep budget.

The default `serde` feature serializes `FlightState` and `MotorPhase`. To resume a flight, restore the state with the same motor schedule, vehicle, environment and solver settings. Motor fuel/phase are derived from simulation time, so they cannot drift apart in a snapshot. Save any stateful guidance controller separately. Invalid deserialized states are rejected when stepped. A controller's mutations are not rolled back if integration fails after the guidance call.

For deterministic replay, retain the same fixed public update cadence, control inputs and environment history. Internal grids are anchored to each public step; changing how calls are partitioned can change floating-point results. Cross-platform bit-for-bit identity is not promised.

For Counterbattery, a future `CbBallistics` adapter can repeatedly call `step` to precompute an unguided powered arc, or call `step_guided` at the simulation cadence for live control. Convert world coordinates once at that boundary, especially absolute altitude and side-facing X. The host must detect terrain and unit collisions over each returned movement interval; the library does not stop on a ground plane. No game integration is included in this crate.

## Scope of 0.2

The engine models translation with prescribed motor burns across serial or parallel stages, in flat or Earth-centered frames. It does not yet model throttle or restart within a stage, launch rails, changing drag geometry, body attitude, lift coefficients, actuator dynamics, seekers, Earth rotation (Coriolis and launch-site velocity are the caller's responsibility in an ECI frame), orbit perturbations such as J2, or terrain collision localization. Guidance hooks provide the boundary for future control models; they are not a built-in autopilot.

## Validation

The tests check the ideal rocket equation and analytic position in vacuum, RK4 convergence, delayed ignition, nonzero endpoint thrust, short burns, zero-thrust intervals, fuel bounds, wind/mass/drag response, control limits, invalid inputs, work limits, and snapshot replay. Unpowered trajectories are compared against the upstream RK4 solver across multiple vehicle, wind and atmosphere configurations. Staging tests cover exact mass drops at separation, hot-staging thrust overlap, inter-stage coast phases, parallel boosters and two-burn Δv against Tsiolkovsky — including a regression for one-sided mass at a step closing exactly on a separation. Central-gravity tests assert Kepler invariants: a circular orbit closes to millimetres and an ICBM-scale midcourse ellipse conserves specific energy and angular momentum to better than 1 part in 10¹⁰. These checks establish numerical behavior and compatibility, not empirical validation of a particular real motor or airframe.

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
