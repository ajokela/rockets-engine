//! Powered, three-dimensional point-mass flight, using SI units and simulation time.
//!
//! Prescribed motors supply thrust and consume propellant in proportion to impulse;
//! [`Stage`]s add serial or parallel staging with scheduled separations. [`Simulator`]
//! integrates translation while [`ForceModel`] supplies environmental acceleration and
//! [`Guidance`] supplies optional control commands. [`UniformGravity`] covers local
//! flat-frame flight and [`CentralGravity`] covers planet-scale arcs; the default
//! `ballistics` feature reuses ballistics-engine's atmosphere and drag kernel.
//!
//! Coordinates are right-handed; the conventional local frame is X/downrange, Y/up,
//! Z/lateral, but [`CentralGravity`] works in an Earth-centered frame of the caller's
//! choosing. No time compression occurs inside this crate. This is a point-mass model,
//! not an attitude, actuator or aerodynamic-lift simulation.
//!
//! See the README and `examples/boost_coast.rs` for integration and model assumptions.

// wasm-bindgen generates unsafe blocks, so the wasm build cannot keep the forbid.
#![cfg_attr(not(feature = "wasm"), forbid(unsafe_code))]
#![warn(missing_docs)]
#![doc = include_str!("../README.md")]

#[cfg(any(feature = "cli", feature = "wasm"))]
mod analysis;
#[cfg(feature = "ballistics")]
pub mod ballistics;
#[cfg(any(feature = "cli", feature = "wasm"))]
pub mod command;
#[cfg(feature = "ballistics")]
mod dynamics;
mod error;
mod flight;
mod guidance;
mod motor;
#[cfg(feature = "wasm")]
pub mod wasm;

#[cfg(feature = "ballistics")]
pub use dynamics::{RollDynamics, Weathercock, WeathercockParams};
pub use error::{Error, Result};
pub use flight::{
    AltitudeSwitch, BoxedForces, CentralGravity, FlightState, ForceModel, IntegratorSettings,
    RailForces, Simulator, StepResult, UniformGravity, EARTH_MU_M3_S2,
};
pub use guidance::{
    BoxedGuidance, ControlInput, FixedDirection, Guidance, GuidanceContext, ProgradeDirection,
    SteeringLag, ThrustMisalignment,
};
pub use motor::{
    Motor, MotorPhase, MotorSample, Rocket, RocketSample, Stage, ThrustCurve, ThrustPoint,
};

/// Three-dimensional SI vector in the fixed world frame.
pub type Vec3 = nalgebra::Vector3<f64>;

pub(crate) fn finite_vec(value: &Vec3) -> bool {
    value.iter().all(|x| x.is_finite())
}

pub(crate) fn nonnegative(value: f64, field: &'static str) -> Result<()> {
    if !value.is_finite() || value < 0.0 {
        return Err(Error::invalid(field, "must be finite and nonnegative"));
    }
    Ok(())
}

pub(crate) fn positive(value: f64, field: &'static str) -> Result<()> {
    if !value.is_finite() || value <= 0.0 {
        return Err(Error::invalid(field, "must be finite and positive"));
    }
    Ok(())
}
