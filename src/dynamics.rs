//! Reduced-order body dynamics: a second-order weathercock filter and a scalar
//! roll channel. These are a few integrated scalars, not a 6-DOF attitude model:
//! no quaternion, no inertia tensor, no gyroscopic coupling.
//!
//! The weathercock lives in the guidance layer, so it composes with the existing
//! integrator unchanged: its filtered axis becomes the commanded thrust axis,
//! and `with_body_lift` closes the loop through the resulting angle of attack.

use crate::{
    ballistics::{local_air, mach_table, Atmosphere},
    ControlInput, Error, Guidance, GuidanceContext, Result, Vec3,
};
use ballistics_engine::drag::DragTable;

/// Aerodynamic and inertial parameters for the weathercock filter.
#[derive(Debug, Clone)]
pub struct WeathercockParams {
    cna_per_rad: DragTable,
    cp_m: DragTable,
    cg_dry_m: f64,
    cg_propellant_m: f64,
    diameter_m: f64,
    pitch_inertia_kg_m2: f64,
    damping_ratio: f64,
}

impl WeathercockParams {
    /// `cna_vs_mach`: normal-force slope per radian (as in body lift).
    /// `cp_vs_mach`: centre of pressure, metres aft of the nose reference.
    /// `cg_dry_m`/`cg_propellant_m`: two-point CG model — the dry structure's
    /// and the propellant's centres of gravity along the same reference. The
    /// instantaneous CG is mass-weighted from the current propellant fraction.
    /// `pitch_inertia_kg_m2`: scalar pitch/yaw inertia about the CG.
    /// `damping_ratio`: ζ of the second-order response (0 = undamped).
    pub fn new(
        cna_vs_mach: Vec<(f64, f64)>,
        cp_vs_mach: Vec<(f64, f64)>,
        cg_dry_m: f64,
        cg_propellant_m: f64,
        diameter_m: f64,
        pitch_inertia_kg_m2: f64,
        damping_ratio: f64,
    ) -> Result<Self> {
        if !cg_dry_m.is_finite() || !cg_propellant_m.is_finite() {
            return Err(Error::invalid("cg", "CG references must be finite"));
        }
        if !diameter_m.is_finite() || diameter_m <= 0.0 {
            return Err(Error::invalid("diameter_m", "must be finite and positive"));
        }
        if !pitch_inertia_kg_m2.is_finite() || pitch_inertia_kg_m2 <= 0.0 {
            return Err(Error::invalid(
                "pitch_inertia_kg_m2",
                "must be finite and positive",
            ));
        }
        if !damping_ratio.is_finite() || damping_ratio < 0.0 {
            return Err(Error::invalid(
                "damping_ratio",
                "must be finite and nonnegative",
            ));
        }
        Ok(Self {
            cna_per_rad: mach_table(cna_vs_mach, "cna_vs_mach")?,
            cp_m: mach_table(cp_vs_mach, "cp_vs_mach")?,
            cg_dry_m,
            cg_propellant_m,
            diameter_m,
            pitch_inertia_kg_m2,
            damping_ratio,
        })
    }
}

/// Second-order weathercock: the body axis relaxes toward the relative wind
/// with natural frequency ωₙ² = q·A·Cnα·(CP−CG)/I and exponential damping
/// 2ζωₙ. Positive static margin (CP aft of CG) is stable; negative margin
/// diverges, as it should. The axis and rate are snapshot-able state, updated
/// once per public step with an explicit first-order-in-dt scheme, so choose
/// the public cadence well below the axis's natural period.
#[derive(Debug, Clone)]
pub struct Weathercock {
    params: WeathercockParams,
    atmosphere: Atmosphere,
    atmos_params: (f64, f64, f64, f64),
    wind_mps: Vec3,
    axis: Vec3,
    rate: Vec3,
}

impl Weathercock {
    /// `initial_axis` is the pad orientation (normalized at construction).
    /// `atmosphere` and `wind_mps` should match the force model's air.
    pub fn new(
        params: WeathercockParams,
        initial_axis: Vec3,
        atmosphere: Atmosphere,
        wind_mps: Vec3,
    ) -> Result<Self> {
        if initial_axis.amax() == 0.0 || !initial_axis.iter().all(|x| x.is_finite()) {
            return Err(Error::invalid("initial_axis", "must be finite and nonzero"));
        }
        if !wind_mps.iter().all(|x| x.is_finite()) {
            return Err(Error::invalid("wind_mps", "must be finite"));
        }
        Ok(Self {
            atmos_params: atmosphere.parameters()?,
            params,
            atmosphere,
            wind_mps,
            axis: initial_axis.normalize(),
            rate: Vec3::zeros(),
        })
    }

    /// Current body axis (unit vector) — snapshot state for replay.
    pub fn axis(&self) -> Vec3 {
        self.axis
    }
    /// Current angular-rate vector, perpendicular to the axis.
    pub fn rate(&self) -> Vec3 {
        self.rate
    }
    /// Restore a previously snapshotted axis/rate pair.
    pub fn set_state(&mut self, axis: Vec3, rate: Vec3) -> Result<()> {
        if axis.amax() == 0.0 || !axis.iter().all(|x| x.is_finite()) {
            return Err(Error::invalid("axis", "must be finite and nonzero"));
        }
        self.axis = axis.normalize();
        self.rate = rate;
        Ok(())
    }

    fn cg_m(&self, context: &GuidanceContext<'_>) -> f64 {
        let total = context.rocket.mass_kg;
        let prop = context.rocket.motor.propellant_mass_kg;
        if total <= 0.0 {
            return self.params.cg_dry_m;
        }
        (self.params.cg_dry_m * (total - prop) + self.params.cg_propellant_m * prop) / total
    }
}

impl Guidance for Weathercock {
    fn command(&mut self, context: GuidanceContext<'_>) -> Result<ControlInput> {
        let dt = context.step_duration_s;
        let v_air = context.state.velocity_mps - self.wind_mps;
        let speed = v_air.norm();
        if speed < 1e-9 {
            // No relative wind: no restoring torque, hold the current axis.
            return Ok(ControlInput::along(self.axis));
        }
        let target = v_air / speed;
        let (rho, sound) = local_air(
            self.atmosphere,
            self.atmos_params,
            context.state.position_m.y,
        );
        let mach = speed / sound;
        let q = 0.5 * rho * speed * speed;
        let area = std::f64::consts::FRAC_PI_4 * self.params.diameter_m.powi(2);
        let cna = self.params.cna_per_rad.interpolate(mach);
        let margin = self.params.cp_m.interpolate(mach) - self.cg_m(&context);
        let omega_sq = q * area * cna * margin / self.params.pitch_inertia_kg_m2;

        // Restoring (or diverging) angular acceleration about axis × target.
        // axis' = axis + (axis×target)×axis·dt rotates toward the target.
        let cross = self.axis.cross(&target);
        let anti_parallel = cross.norm() < 1e-12 && self.axis.dot(&target) < 0.0;
        let cross = if anti_parallel && self.rate.norm() < 1e-12 {
            // Exact inversion is an unstable equilibrium; seed a deterministic
            // perturbation so the flip starts instead of sticking numerically.
            let seed = if self.axis.z.abs() < 0.9 {
                Vec3::z()
            } else {
                Vec3::y()
            };
            self.axis.cross(&seed).normalize() * 1e-6
        } else {
            cross
        };
        self.rate += cross * (omega_sq * dt);
        let omega = omega_sq.abs().sqrt();
        if omega > 0.0 {
            self.rate *= (-2.0 * self.params.damping_ratio * omega * dt).exp();
        }
        // Keep the rate perpendicular, then rotate the axis by rate·dt.
        self.rate -= self.axis * self.axis.dot(&self.rate);
        let delta = self.rate.cross(&self.axis) * dt;
        self.axis = (self.axis + delta).normalize();
        Ok(ControlInput::along(self.axis))
    }
}

/// Scalar roll channel: roll rate driven by fin cant and opposed by roll
/// damping. Drive torque = q·A·d·Clδ·δ, damping torque = q·A·d²·Clp·ω/(2V).
/// Equilibrium rate = 2·Clδ·δ·V/(d·Clp). Feed the resulting effective twist
/// (`speed / rate`, in inches per turn) to `BallisticForces::set_twist`
/// between public steps.
#[derive(Debug, Clone, Copy)]
pub struct RollDynamics {
    rate_rad_s: f64,
    cant_rad: f64,
    cl_delta: f64,
    clp: f64,
    inertia_kg_m2: f64,
    diameter_m: f64,
}

impl RollDynamics {
    /// `cant_rad`: fin cant angle. `cl_delta`: roll-drive slope per radian of
    /// cant. `clp`: roll-damping coefficient (positive = damping).
    /// `inertia_kg_m2`: roll inertia about the body axis.
    pub fn new(
        cant_rad: f64,
        cl_delta: f64,
        clp: f64,
        inertia_kg_m2: f64,
        diameter_m: f64,
    ) -> Result<Self> {
        if !cant_rad.is_finite() || !cl_delta.is_finite() || !clp.is_finite() || clp <= 0.0 {
            return Err(Error::invalid(
                "roll parameters",
                "cant and cl_delta must be finite, clp finite and positive",
            ));
        }
        if !inertia_kg_m2.is_finite() || inertia_kg_m2 <= 0.0 {
            return Err(Error::invalid(
                "roll inertia_kg_m2",
                "must be finite and positive",
            ));
        }
        if !diameter_m.is_finite() || diameter_m <= 0.0 {
            return Err(Error::invalid("diameter_m", "must be finite and positive"));
        }
        Ok(Self {
            rate_rad_s: 0.0,
            cant_rad,
            cl_delta,
            clp,
            inertia_kg_m2,
            diameter_m,
        })
    }

    /// Current roll rate in rad/s (snapshot state).
    pub fn rate_rad_s(&self) -> f64 {
        self.rate_rad_s
    }
    /// Restore a previously snapshotted roll rate.
    pub fn set_rate_rad_s(&mut self, rate: f64) {
        self.rate_rad_s = rate;
    }

    /// Advance one public step given dynamic pressure (Pa) and airspeed (m/s).
    pub fn update(&mut self, dt_s: f64, q_pa: f64, speed_mps: f64) {
        let d = self.diameter_m;
        let area = std::f64::consts::FRAC_PI_4 * d * d;
        let drive = q_pa * area * d * self.cl_delta * self.cant_rad;
        let damping =
            q_pa * area * d * d * self.clp * self.rate_rad_s / (2.0 * speed_mps.max(1e-9));
        self.rate_rad_s += (drive - damping) / self.inertia_kg_m2 * dt_s;
    }

    /// Equilibrium roll rate for the current cant and airspeed, rad/s.
    pub fn equilibrium_rate(&self, speed_mps: f64) -> f64 {
        2.0 * self.cl_delta * self.cant_rad * speed_mps / (self.diameter_m * self.clp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{FlightState, MotorPhase, MotorSample, RocketSample};

    fn params(margin_cp: f64) -> WeathercockParams {
        WeathercockParams::new(
            vec![(0.0, 2.0), (3.0, 2.0)],
            vec![(0.0, margin_cp), (3.0, margin_cp)],
            0.3,
            0.3,
            0.1,
            0.05,
            0.3,
        )
        .unwrap()
    }

    fn context(state: &FlightState) -> GuidanceContext<'_> {
        GuidanceContext {
            state,
            rocket: RocketSample {
                mass_kg: 1.0,
                motor: MotorSample {
                    phase: MotorPhase::Unpowered,
                    thrust_n: 0.0,
                    propellant_mass_kg: 0.0,
                },
            },
            step_duration_s: 0.01,
            max_steering_accel_mps2: 0.0,
        }
    }

    #[test]
    fn stable_body_weathercocks_toward_the_wind() {
        let params = params(0.6); // CP 0.6, CG 0.3: stable margin
        let mut w = Weathercock::new(
            params,
            Vec3::new(1.0, 0.1, 0.0),
            Atmosphere::default(),
            Vec3::zeros(),
        )
        .unwrap();
        let state = FlightState {
            time_s: 0.0,
            position_m: Vec3::zeros(),
            velocity_mps: Vec3::new(100.0, 0.0, 0.0),
        };
        for _ in 0..500 {
            w.command(context(&state)).unwrap();
        }
        let dot = w.axis().dot(&Vec3::x());
        assert!(dot > 0.999, "should align with the wind, dot {dot}");
    }

    #[test]
    fn unstable_body_diverges() {
        let params = params(0.1); // CP 0.1, CG 0.3: negative margin
        let mut w = Weathercock::new(
            params,
            Vec3::new(1.0, 0.01, 0.0),
            Atmosphere::default(),
            Vec3::zeros(),
        )
        .unwrap();
        let state = FlightState {
            time_s: 0.0,
            position_m: Vec3::zeros(),
            velocity_mps: Vec3::new(100.0, 0.0, 0.0),
        };
        for _ in 0..500 {
            w.command(context(&state)).unwrap();
        }
        assert!(w.axis().dot(&Vec3::x()) < 0.9, "should tumble away");
    }

    #[test]
    fn aligned_axis_stays_aligned() {
        let params = params(0.6);
        let mut w =
            Weathercock::new(params, Vec3::x(), Atmosphere::default(), Vec3::zeros()).unwrap();
        let state = FlightState {
            time_s: 0.0,
            position_m: Vec3::zeros(),
            velocity_mps: Vec3::new(100.0, 0.0, 0.0),
        };
        for _ in 0..100 {
            w.command(context(&state)).unwrap();
        }
        assert!((w.axis() - Vec3::x()).norm() < 1e-12);
    }

    #[test]
    fn roll_reaches_the_cant_equilibrium() {
        let mut roll = RollDynamics::new(0.02, 1.0, 1.0, 0.001, 0.1).unwrap();
        let q = 5000.0;
        let v = 200.0;
        // Roll time constant here is ~1 s; integrate 10 s.
        for _ in 0..10000 {
            roll.update(0.001, q, v);
        }
        let expected = roll.equilibrium_rate(v);
        assert!(
            (roll.rate_rad_s() - expected).abs() / expected.abs() < 0.01,
            "rate {} vs equilibrium {expected}",
            roll.rate_rad_s()
        );
    }

    #[test]
    fn uncanted_roll_decays_to_zero() {
        let mut roll = RollDynamics::new(0.0, 1.0, 1.0, 0.001, 0.1).unwrap();
        roll.set_rate_rad_s(50.0);
        for _ in 0..10000 {
            roll.update(0.001, 5000.0, 200.0);
        }
        assert!(roll.rate_rad_s().abs() < 0.01);
    }
}
