//! Powered, three-dimensional point-mass flight, using SI units and simulation time.
//!
//! A prescribed motor supplies thrust and consumes propellant in proportion to impulse.
//! [`Simulator`] integrates translation while [`ForceModel`] supplies environmental
//! acceleration and [`Guidance`] supplies optional control commands. The default
//! `ballistics` feature reuses ballistics-engine's gravity, atmosphere and drag kernel.
//!
//! Coordinates are fixed, right-handed X/downrange, Y/up, Z/lateral. With the ballistics
//! adapter, Y is altitude above sea level. No time compression occurs inside this crate.
//! This is a point-mass model, not an attitude, actuator or aerodynamic-lift simulation.
//!
//! See the README and `examples/boost_coast.rs` for integration and model assumptions.

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![doc = include_str!("../README.md")]

#[cfg(feature = "ballistics")]
pub mod ballistics;
mod error;
mod flight;
mod guidance;
mod motor;

pub use error::{Error, Result};
pub use flight::{
    FlightState, ForceModel, IntegratorSettings, Simulator, StepResult, UniformGravity,
};
pub use guidance::{ControlInput, FixedDirection, Guidance, GuidanceContext};
pub use motor::{Motor, MotorPhase, MotorSample, Rocket, RocketSample, ThrustCurve, ThrustPoint};

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
