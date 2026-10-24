//! Shared command engine behind the `rockets` CLI and the browser terminal.
//!
//! One hand-rolled parser serves both front ends so the native binary and the
//! WASM build can never drift apart: the same argv produces the same text.
//! Native `main` passes `std::env::args`; the terminal passes the typed line.

use crate::{
    AltitudeSwitch, BoxedForces, BoxedGuidance, CentralGravity, ControlInput, Error,
    FixedDirection, FlightState, ForceModel, Guidance, GuidanceContext, IntegratorSettings, Motor,
    MotorPhase, ProgradeDirection, RailForces, Rocket, Simulator, Stage, SteeringLag, ThrustCurve,
    ThrustMisalignment, ThrustPoint, UniformGravity, Vec3,
};

/// Entry point shared by the CLI and the terminal. Returns the text to print;
/// `Err` text is already formatted for display.
pub fn run(args: &[String]) -> Result<String, String> {
    let Some((head, rest)) = args.split_first() else {
        return Ok(help());
    };
    match head.as_str() {
        "help" | "--help" | "-h" => Ok(help()),
        "version" | "--version" | "-V" => Ok(version()),
        "simulate" | "sim" => simulate(rest),
        "dispersion" => crate::analysis::dispersion(rest),
        "solve-range" => crate::analysis::solve_range(rest),
        other => Err(format!("error: unknown command '{other}'\n\n{}", help())),
    }
}

/// Split a typed command line into argv, honoring single and double quotes.
pub fn tokenize(line: &str) -> Vec<String> {
    let mut args = Vec::new();
    let mut current = String::new();
    let mut quote = None;
    for c in line.chars() {
        match quote {
            Some(q) if c == q => quote = None,
            Some(_) => current.push(c),
            None if c == '\'' || c == '"' => quote = Some(c),
            None if c.is_whitespace() => {
                if !current.is_empty() {
                    args.push(std::mem::take(&mut current));
                }
            }
            None => current.push(c),
        }
    }
    if !current.is_empty() {
        args.push(current);
    }
    args
}

fn version() -> String {
    format!("rockets {}", env!("CARGO_PKG_VERSION"))
}

fn help() -> String {
    "rockets — point-mass powered flight\n\
     \n\
     COMMANDS\n\
     \x20 simulate [OPTIONS]     integrate a flight and print the trajectory\n\
     \x20 dispersion [OPTIONS]   Monte Carlo over simulate; CEP and spread stats\n\
     \x20 solve-range <m> [OPTS] find the launch elevation for a downrange target\n\
     \x20 version                print the engine version\n\
     \x20 help                   this text\n\
     \n\
     VEHICLE\n\
     \x20 --curve <t:N,...>        thrust points, seconds:newtons (required)\n\
     \x20 --dry-mass <kg>          vehicle mass without propellant (required)\n\
     \x20 --propellant-mass <kg>   consumable propellant (required)\n\
     \x20 --ignition-s <t>         ignition delay [0]\n\
     \x20 --stage <spec>          repeat for staging; replaces the single-motor\n\
     \x20                            options. spec: dry=kg,prop=kg,curve=t:N;t:N,\n\
     \x20                            ignition=s,sep=s (curve/ignition/sep optional)\n\
     \x20 --duration-s <t>         total sim time [burnout + 10 s]\n\
     \x20 --dt-s <t>               output cadence [0.05]\n\
     \n\
     LAUNCH & GUIDANCE\n\
     \x20 --elevation-deg <deg>    launch elevation above horizontal [90]\n\
     \x20 --azimuth-deg <deg>      launch azimuth from +X toward +Z [0]\n\
     \x20 --speed-mps <v>          initial speed along the launch axis [0]\n\
     \x20 --position-m <x,y,z>     initial position [0,0,0]\n\
     \x20 --prograde              thrust follows velocity (gravity turn)\n\
     \x20 --misalign <tilt:clock>  fixed thrust-axis misalignment, radians\n\
     \x20 --steering <x,y,z>       constant steering acceleration, m/s^2\n\
     \x20 --steering-cap <a>       steering magnitude cap [the steering norm]\n\
     \x20 --steering-tau <s>       first-order actuator lag time constant\n\
     \x20 --rail <m>               launch rail length along the launch axis\n\
     \n\
     GRAVITY & SPACE\n\
     \x20 --gravity <mode>         uniform [default], central, vacuum\n\
     \x20 --mu <m3/s2>             central-gravity parameter [Earth]\n\
     \x20 --center <x,y,z>         central-gravity center [0,0,0]\n\
     \x20 --space-above-m <alt>    above this altitude use vacuum (or central)\n\
     \n\
     ATMOSPHERE & AERO (feature 'ballistics', default)\n\
     \x20 --diameter-m <d>         body diameter; enables drag with --cd\n\
     \x20 --cd <mach:cd,...>       drag coefficient curve\n\
     \x20 --cd-scale <f>           whole-curve drag multiplier [1.0]\n\
     \x20 --wind <x,y,z>           air velocity; +X is a tailwind [0,0,0]\n\
     \x20 --wind-zone <s:a:u:v>    downrange wind zone, repeatable:\n\
     \x20                            speed m/s, angle deg (0=headwind),\n\
     \x20                            until downrange m, vertical m/s\n\
     \x20 --atmo-zone <t:p:h:u>    atmosphere zone, repeatable: temp C,\n\
     \x20                            pressure hPa, humidity %, until downrange m\n\
     \x20 --twist-in <in/turn>     effective spin twist; enables Magnus + yaw\n\
     \x20                            of repose (needs --body-length-m)\n\
     \x20 --body-length-m <m>       airframe length for the spin model\n\
     \x20 --twist-left             roll left-handed instead of right\n\
     \x20 --coriolis <lat,az>      launch latitude° and bearing° (0=north)\n\
     \x20 --tipoff <yaw,decay>     rail tip-off yaw in rad, decay length in m\n\
     \x20 --cna <mach:cna,...>     normal-force slope per radian vs Mach;\n\
     \x20                            body lift from angle of attack (needs --lift-k)\n\
     \x20 --lift-k <mach:k,...>     induced drag factor k (Cd += k·α²) vs Mach\n\
     \n\
     OUTPUT\n\
     \x20 --format table|csv|json  output shape [table]\n\
     \x20 --plot                   append an altitude chart\n\
     \x20                            (ASCII in the CLI, interactive in the browser)\n\
     \n\
     DISPERSION OPTIONS\n\
     \x20 --runs <n>               number of samples [32]\n\
     \x20 --seed <n>               deterministic RNG seed [42]\n\
     \x20 --cd-sigma <f>           lognormal Cd-scale sigma [0.03]\n\
     \x20 --wind-sigma <mps>       per-axis wind perturbation sigma [1.0]\n\
     \x20 --speed-sigma <f>        fractional initial-speed sigma [0.01]\n\
     \n\
     SOLVE-RANGE OPTIONS\n\
     \x20 --high-angle            search the high-angle branch instead of low\n\
     \n\
     EXAMPLE\n\
     \x20 simulate --curve 0:0,0.1:20,0.9:15,1:0 --dry-mass 0.18 \\\n\
     \x20   --propellant-mass 0.04 --ignition-s 0.2 --plot"
        .to_string()
}

#[derive(Clone)]
pub(crate) struct Options {
    pub(crate) curve: ThrustCurve,
    pub(crate) dry_mass: f64,
    pub(crate) propellant: f64,
    pub(crate) ignition_s: f64,
    pub(crate) duration_s: Option<f64>,
    pub(crate) dt_s: f64,
    pub(crate) elevation_deg: f64,
    pub(crate) azimuth_deg: f64,
    pub(crate) speed_mps: f64,
    pub(crate) position_m: Vec3,
    pub(crate) format: Format,
    pub(crate) plot: bool,
    pub(crate) prograde: bool,
    pub(crate) misalign: Option<(f64, f64)>,
    pub(crate) steering: Vec3,
    pub(crate) steering_cap: Option<f64>,
    pub(crate) steering_tau: Option<f64>,
    pub(crate) rail_m: Option<f64>,
    pub(crate) gravity: GravityMode,
    pub(crate) mu: Option<f64>,
    pub(crate) center: Vec3,
    pub(crate) space_above_m: Option<f64>,
    pub(crate) stages: Vec<StageSpec>,
    #[cfg(feature = "ballistics")]
    pub(crate) diameter_m: Option<f64>,
    #[cfg(feature = "ballistics")]
    pub(crate) cd: Option<Vec<(f64, f64)>>,
    #[cfg(feature = "ballistics")]
    pub(crate) cd_scale: f64,
    #[cfg(feature = "ballistics")]
    pub(crate) wind_mps: Vec3,
    #[cfg(feature = "ballistics")]
    pub(crate) wind_zones: Vec<(f64, f64, f64, f64)>,
    #[cfg(feature = "ballistics")]
    pub(crate) atmo_zones: Vec<(f64, f64, f64, f64)>,
    #[cfg(feature = "ballistics")]
    pub(crate) twist_in: Option<f64>,
    #[cfg(feature = "ballistics")]
    pub(crate) body_length_m: Option<f64>,
    #[cfg(feature = "ballistics")]
    pub(crate) twist_left: bool,
    #[cfg(feature = "ballistics")]
    pub(crate) coriolis: Option<(f64, f64)>,
    #[cfg(feature = "ballistics")]
    pub(crate) tipoff: Option<(f64, f64)>,
    #[cfg(feature = "ballistics")]
    pub(crate) cna: Option<Vec<(f64, f64)>>,
    #[cfg(feature = "ballistics")]
    pub(crate) lift_k: Option<Vec<(f64, f64)>>,
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum GravityMode {
    Uniform,
    Central,
    Vacuum,
}

#[derive(Clone)]
pub(crate) struct StageSpec {
    dry_mass: f64,
    propellant: Option<f64>,
    curve: Option<ThrustCurve>,
    ignition_s: f64,
    separation_s: Option<f64>,
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum Format {
    Table,
    Csv,
    Json,
}

pub(crate) fn parse_pairs(spec: &str, what: &str) -> Result<Vec<(f64, f64)>, String> {
    let mut pairs = Vec::new();
    for item in spec.split(',') {
        let (a, b) = item
            .split_once(':')
            .ok_or_else(|| format!("error: {what} point '{item}' needs time:value"))?;
        let x: f64 = a
            .trim()
            .parse()
            .map_err(|_| format!("error: {what} point '{item}' has a non-numeric time"))?;
        let y: f64 = b
            .trim()
            .parse()
            .map_err(|_| format!("error: {what} point '{item}' has a non-numeric value"))?;
        pairs.push((x, y));
    }
    Ok(pairs)
}

fn parse_vec3(spec: &str, what: &str) -> Result<Vec3, String> {
    let parts: Vec<&str> = spec.split(',').collect();
    if parts.len() != 3 {
        return Err(format!("error: {what} needs exactly x,y,z"));
    }
    let mut v = [0.0; 3];
    for (i, p) in parts.iter().enumerate() {
        v[i] = p
            .trim()
            .parse()
            .map_err(|_| format!("error: {what} component '{p}' is not a number"))?;
    }
    Ok(Vec3::new(v[0], v[1], v[2]))
}

fn parse_f64(spec: &str, what: &str) -> Result<f64, String> {
    spec.parse()
        .map_err(|_| format!("error: {what} '{spec}' is not a number"))
}

fn parse_quad(spec: &str, what: &str) -> Result<(f64, f64, f64, f64), String> {
    let parts: Vec<&str> = spec.split(':').collect();
    if parts.len() != 4 {
        return Err(format!("error: {what} needs four colon-separated values"));
    }
    let mut v = [0.0; 4];
    for (i, p) in parts.iter().enumerate() {
        v[i] = parse_f64(p, what)?;
    }
    Ok((v[0], v[1], v[2], v[3]))
}

fn parse_stage(spec: &str) -> Result<StageSpec, String> {
    let mut dry_mass = None;
    let mut propellant = None;
    let mut curve = None;
    let mut ignition_s = 0.0;
    let mut separation_s = None;
    for item in spec.split(',') {
        let (key, val) = item
            .split_once('=')
            .ok_or_else(|| format!("error: stage item '{item}' needs key=value"))?;
        match key {
            "dry" => dry_mass = Some(parse_f64(val, "stage dry")?),
            "prop" => propellant = Some(parse_f64(val, "stage prop")?),
            "curve" => {
                let points: Vec<ThrustPoint> = val
                    .split(';')
                    .map(|p| {
                        let (t, f) = p
                            .split_once(':')
                            .ok_or_else(|| format!("error: stage curve point '{p}'"))?;
                        Ok(ThrustPoint {
                            time_s: parse_f64(t, "stage curve time")?,
                            thrust_n: parse_f64(f, "stage curve thrust")?,
                        })
                    })
                    .collect::<Result<_, String>>()?;
                curve = Some(ThrustCurve::new(points).map_err(|e| format!("error: {e}"))?);
            }
            "ignition" => ignition_s = parse_f64(val, "stage ignition")?,
            "sep" => separation_s = Some(parse_f64(val, "stage sep")?),
            other => return Err(format!("error: unknown stage key '{other}'")),
        }
    }
    let dry_mass = dry_mass.ok_or("error: --stage needs dry=<kg>")?;
    if curve.is_some() != propellant.is_some() {
        return Err("error: a stage motor needs both curve= and prop=".to_string());
    }
    Ok(StageSpec {
        dry_mass,
        propellant,
        curve,
        ignition_s,
        separation_s,
    })
}

pub(crate) struct Args {
    pub(crate) opts: Options,
    pub(crate) runs: usize,
    pub(crate) seed: u64,
    pub(crate) cd_sigma: f64,
    pub(crate) wind_sigma: f64,
    pub(crate) speed_sigma: f64,
    pub(crate) high_angle: bool,
    pub(crate) target_m: Option<f64>,
}

pub(crate) fn parse(args: &[String]) -> Result<Args, String> {
    let mut curve = None;
    let mut dry_mass = None;
    let mut propellant = None;
    let mut ignition_s = 0.0;
    let mut duration_s = None;
    let mut dt_s = 0.05;
    let mut elevation_deg = 90.0;
    let mut azimuth_deg = 0.0;
    let mut speed_mps = 0.0;
    let mut position_m = Vec3::zeros();
    let mut format = Format::Table;
    let mut plot = false;
    let mut prograde = false;
    let mut misalign = None;
    let mut steering = Vec3::zeros();
    let mut steering_cap = None;
    let mut steering_tau = None;
    let mut rail_m = None;
    let mut gravity = GravityMode::Uniform;
    let mut mu = None;
    let mut center = Vec3::zeros();
    let mut space_above_m = None;
    let mut stages = Vec::new();
    let mut runs = 32usize;
    let mut seed = 42u64;
    let mut cd_sigma = 0.03;
    let mut wind_sigma = 1.0;
    let mut speed_sigma = 0.01;
    let mut high_angle = false;
    let mut target_m = None;
    #[cfg(feature = "ballistics")]
    let mut diameter_m = None;
    #[cfg(feature = "ballistics")]
    let mut cd = None;
    #[cfg(feature = "ballistics")]
    let mut cd_scale = 1.0;
    #[cfg(feature = "ballistics")]
    let mut wind_mps = Vec3::zeros();
    #[cfg(feature = "ballistics")]
    let mut wind_zones = Vec::new();
    #[cfg(feature = "ballistics")]
    let mut atmo_zones = Vec::new();
    #[cfg(feature = "ballistics")]
    let mut twist_in = None;
    #[cfg(feature = "ballistics")]
    let mut body_length_m = None;
    #[cfg(feature = "ballistics")]
    let mut twist_left = false;
    #[cfg(feature = "ballistics")]
    let mut coriolis = None;
    #[cfg(feature = "ballistics")]
    let mut tipoff = None;
    #[cfg(feature = "ballistics")]
    let mut cna = None;
    #[cfg(feature = "ballistics")]
    let mut lift_k = None;

    let mut i = 0;
    while i < args.len() {
        let flag = args[i].as_str();
        let mut value = |name: &str| -> Result<String, String> {
            i += 1;
            args.get(i)
                .cloned()
                .ok_or_else(|| format!("error: {name} needs a value"))
        };
        match flag {
            "--curve" => {
                let spec = value(flag)?;
                let pairs = parse_pairs(&spec, "curve")?;
                let points: Vec<ThrustPoint> = pairs
                    .into_iter()
                    .map(|(time_s, thrust_n)| ThrustPoint { time_s, thrust_n })
                    .collect();
                curve = Some(ThrustCurve::new(points).map_err(|e| format!("error: {e}"))?);
            }
            "--dry-mass" => dry_mass = Some(parse_f64(&value(flag)?, flag)?),
            "--propellant-mass" => propellant = Some(parse_f64(&value(flag)?, flag)?),
            "--ignition-s" => ignition_s = parse_f64(&value(flag)?, flag)?,
            "--duration-s" => duration_s = Some(parse_f64(&value(flag)?, flag)?),
            "--dt-s" => dt_s = parse_f64(&value(flag)?, flag)?,
            "--elevation-deg" => elevation_deg = parse_f64(&value(flag)?, flag)?,
            "--azimuth-deg" => azimuth_deg = parse_f64(&value(flag)?, flag)?,
            "--speed-mps" => speed_mps = parse_f64(&value(flag)?, flag)?,
            "--position-m" => position_m = parse_vec3(&value(flag)?, flag)?,
            "--format" => {
                format = match value(flag)?.as_str() {
                    "table" => Format::Table,
                    "csv" => Format::Csv,
                    "json" => Format::Json,
                    other => {
                        return Err(format!(
                            "error: --format must be table, csv or json, not '{other}'"
                        ))
                    }
                }
            }
            "--plot" => plot = true,
            "--prograde" => prograde = true,
            "--misalign" => {
                let pairs = parse_pairs(&value(flag)?, "misalign")?;
                if pairs.len() != 1 {
                    return Err("error: --misalign needs exactly tilt:clock".to_string());
                }
                misalign = Some(pairs[0]);
            }
            "--steering" => steering = parse_vec3(&value(flag)?, flag)?,
            "--steering-cap" => steering_cap = Some(parse_f64(&value(flag)?, flag)?),
            "--steering-tau" => steering_tau = Some(parse_f64(&value(flag)?, flag)?),
            "--rail" => rail_m = Some(parse_f64(&value(flag)?, flag)?),
            "--gravity" => {
                gravity = match value(flag)?.as_str() {
                    "uniform" => GravityMode::Uniform,
                    "central" => GravityMode::Central,
                    "vacuum" => GravityMode::Vacuum,
                    other => {
                        return Err(format!(
                            "error: --gravity must be uniform, central or vacuum, not '{other}'"
                        ))
                    }
                }
            }
            "--mu" => mu = Some(parse_f64(&value(flag)?, flag)?),
            "--center" => center = parse_vec3(&value(flag)?, flag)?,
            "--space-above-m" => space_above_m = Some(parse_f64(&value(flag)?, flag)?),
            "--stage" => stages.push(parse_stage(&value(flag)?)?),
            "--runs" => {
                runs = value(flag)?
                    .parse()
                    .map_err(|_| "error: --runs needs a positive integer".to_string())?
            }
            "--seed" => {
                seed = value(flag)?
                    .parse()
                    .map_err(|_| "error: --seed needs an integer".to_string())?
            }
            "--cd-sigma" => cd_sigma = parse_f64(&value(flag)?, flag)?,
            "--wind-sigma" => wind_sigma = parse_f64(&value(flag)?, flag)?,
            "--speed-sigma" => speed_sigma = parse_f64(&value(flag)?, flag)?,
            "--high-angle" => high_angle = true,
            #[cfg(feature = "ballistics")]
            "--diameter-m" => diameter_m = Some(parse_f64(&value(flag)?, flag)?),
            #[cfg(feature = "ballistics")]
            "--cd" => cd = Some(parse_pairs(&value(flag)?, "cd")?),
            #[cfg(feature = "ballistics")]
            "--cd-scale" => cd_scale = parse_f64(&value(flag)?, flag)?,
            #[cfg(feature = "ballistics")]
            "--wind" => wind_mps = parse_vec3(&value(flag)?, flag)?,
            #[cfg(feature = "ballistics")]
            "--wind-zone" => wind_zones.push(parse_quad(&value(flag)?, "wind-zone")?),
            #[cfg(feature = "ballistics")]
            "--atmo-zone" => atmo_zones.push(parse_quad(&value(flag)?, "atmo-zone")?),
            #[cfg(feature = "ballistics")]
            "--twist-in" => twist_in = Some(parse_f64(&value(flag)?, flag)?),
            #[cfg(feature = "ballistics")]
            "--body-length-m" => body_length_m = Some(parse_f64(&value(flag)?, flag)?),
            #[cfg(feature = "ballistics")]
            "--twist-left" => twist_left = true,
            #[cfg(feature = "ballistics")]
            "--coriolis" => {
                let pairs = parse_pairs(&value(flag)?, "coriolis")?;
                if pairs.len() != 1 {
                    return Err("error: --coriolis needs exactly lat:az".to_string());
                }
                coriolis = Some(pairs[0]);
            }
            #[cfg(feature = "ballistics")]
            "--tipoff" => {
                let pairs = parse_pairs(&value(flag)?, "tipoff")?;
                if pairs.len() != 1 {
                    return Err("error: --tipoff needs exactly yaw:decay".to_string());
                }
                tipoff = Some(pairs[0]);
            }
            #[cfg(feature = "ballistics")]
            "--cna" => cna = Some(parse_pairs(&value(flag)?, "cna")?),
            #[cfg(feature = "ballistics")]
            "--lift-k" => lift_k = Some(parse_pairs(&value(flag)?, "lift-k")?),
            other if target_m.is_none() && !other.starts_with('-') => {
                target_m = Some(parse_f64(other, "target range")?)
            }
            other => return Err(format!("error: unknown option '{other}'")),
        }
        i += 1;
    }

    if !stages.is_empty() && (curve.is_some() || propellant.is_some()) {
        return Err("error: --stage replaces --curve/--propellant-mass".to_string());
    }
    let (curve, propellant) = if stages.is_empty() {
        let curve = curve.ok_or("error: --curve is required")?;
        let propellant = propellant.ok_or("error: --propellant-mass is required")?;
        (curve, propellant)
    } else {
        // Placeholder motor for staged mode; run_trajectory builds stages instead.
        (
            ThrustCurve::new(vec![
                ThrustPoint {
                    time_s: 0.0,
                    thrust_n: 0.0,
                },
                ThrustPoint {
                    time_s: 1.0,
                    thrust_n: 1.0,
                },
            ])
            .map_err(|e| format!("error: {e}"))?,
            1.0,
        )
    };
    let dry_mass = dry_mass.ok_or("error: --dry-mass is required")?;

    let opts = Options {
        duration_s,
        curve,
        dry_mass,
        propellant,
        ignition_s,
        dt_s,
        elevation_deg,
        azimuth_deg,
        speed_mps,
        position_m,
        format,
        plot,
        prograde,
        misalign,
        steering,
        steering_cap,
        steering_tau,
        rail_m,
        gravity,
        mu,
        center,
        space_above_m,
        stages,
        #[cfg(feature = "ballistics")]
        diameter_m,
        #[cfg(feature = "ballistics")]
        cd,
        #[cfg(feature = "ballistics")]
        cd_scale,
        #[cfg(feature = "ballistics")]
        wind_mps,
        #[cfg(feature = "ballistics")]
        wind_zones,
        #[cfg(feature = "ballistics")]
        atmo_zones,
        #[cfg(feature = "ballistics")]
        twist_in,
        #[cfg(feature = "ballistics")]
        body_length_m,
        #[cfg(feature = "ballistics")]
        twist_left,
        #[cfg(feature = "ballistics")]
        coriolis,
        #[cfg(feature = "ballistics")]
        tipoff,
        #[cfg(feature = "ballistics")]
        cna,
        #[cfg(feature = "ballistics")]
        lift_k,
    };
    Ok(Args {
        opts,
        runs,
        seed,
        cd_sigma,
        wind_sigma,
        speed_sigma,
        high_angle,
        target_m,
    })
}

fn simulate(args: &[String]) -> Result<String, String> {
    let parsed = parse(args)?;
    let (rows, substeps) = run_trajectory(&parsed.opts, false)?;
    Ok(render(&rows, &parsed.opts, substeps))
}

#[derive(Clone, Copy)]
pub(crate) struct Row {
    pub(crate) t: f64,
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) z: f64,
    pub(crate) speed: f64,
    pub(crate) mass: f64,
    pub(crate) thrust: f64,
    pub(crate) phase: &'static str,
}

fn phase_name(phase: MotorPhase) -> &'static str {
    match phase {
        MotorPhase::Unpowered => "unpowered",
        MotorPhase::AwaitingIgnition => "pre-ignition",
        MotorPhase::Burning => "burning",
        MotorPhase::BurnedOut => "burnout",
    }
}

enum Forces {
    Uniform(UniformGravity),
    Central(CentralGravity),
    #[cfg(feature = "ballistics")]
    Drag(Box<crate::ballistics::BallisticForces>),
    Switched(AltitudeSwitch<BoxedForces, BoxedForces>),
    Railed(RailForces<BoxedForces>),
}

impl ForceModel for Forces {
    fn acceleration(&self, state: &FlightState, mass_kg: f64) -> crate::Result<Vec3> {
        match self {
            Self::Uniform(f) => f.acceleration(state, mass_kg),
            Self::Central(f) => f.acceleration(state, mass_kg),
            #[cfg(feature = "ballistics")]
            Self::Drag(f) => f.acceleration(state, mass_kg),
            Self::Switched(f) => f.acceleration(state, mass_kg),
            Self::Railed(f) => f.acceleration(state, mass_kg),
        }
    }

    fn acceleration_with_axis(
        &self,
        state: &FlightState,
        mass_kg: f64,
        body_axis: Vec3,
    ) -> crate::Result<Vec3> {
        match self {
            Self::Uniform(f) => f.acceleration_with_axis(state, mass_kg, body_axis),
            Self::Central(f) => f.acceleration_with_axis(state, mass_kg, body_axis),
            #[cfg(feature = "ballistics")]
            Self::Drag(f) => f.acceleration_with_axis(state, mass_kg, body_axis),
            Self::Switched(f) => f.acceleration_with_axis(state, mass_kg, body_axis),
            Self::Railed(f) => f.acceleration_with_axis(state, mass_kg, body_axis),
        }
    }
}

struct WithSteering<G> {
    inner: G,
    steering: Vec3,
}

impl<G: Guidance> Guidance for WithSteering<G> {
    fn command(&mut self, context: GuidanceContext<'_>) -> crate::Result<ControlInput> {
        let mut command = self.inner.command(context)?;
        command.steering_acceleration_mps2 = self.steering;
        Ok(command)
    }
}

fn fmt_err(e: Error) -> String {
    format!("error: {e}")
}

/// Assemble the rocket, environment and guidance chain, then integrate.
/// Returns the sampled rows and the internal RK4 substep count.
/// `stop_at_impact` ends the run at the first ground crossing (y reaching 0).
pub(crate) fn run_trajectory(
    opts: &Options,
    stop_at_impact: bool,
) -> Result<(Vec<Row>, usize), String> {
    let (rocket, default_duration) = if opts.stages.is_empty() {
        let motor =
            Motor::new(opts.curve.clone(), opts.propellant, opts.ignition_s).map_err(fmt_err)?;
        let burnout = motor.burnout_time_s();
        let rocket = Rocket::new(opts.dry_mass, Some(motor)).map_err(fmt_err)?;
        (rocket, burnout + 10.0)
    } else {
        let mut stages = Vec::new();
        let mut last_event = 0.0_f64;
        for spec in &opts.stages {
            let motor = match (&spec.curve, spec.propellant) {
                (Some(curve), Some(prop)) => {
                    let m = Motor::new(curve.clone(), prop, spec.ignition_s).map_err(fmt_err)?;
                    last_event = last_event.max(m.burnout_time_s());
                    Some(m)
                }
                _ => None,
            };
            if let Some(sep) = spec.separation_s {
                last_event = last_event.max(sep);
            }
            stages.push(Stage::new(spec.dry_mass, motor, spec.separation_s).map_err(fmt_err)?);
        }
        let rocket = Rocket::staged(opts.dry_mass, stages).map_err(fmt_err)?;
        (rocket, last_event + 10.0)
    };

    let el = opts.elevation_deg.to_radians();
    let az = opts.azimuth_deg.to_radians();
    let axis = Vec3::new(el.cos() * az.cos(), el.sin(), el.cos() * az.sin());
    let duration = opts.duration_s.unwrap_or(default_duration);

    #[cfg(feature = "ballistics")]
    let use_drag = opts.diameter_m.is_some() || opts.cd.is_some();
    #[cfg(feature = "ballistics")]
    let wants_mpm = opts.twist_in.is_some()
        || opts.body_length_m.is_some()
        || opts.twist_left
        || opts.coriolis.is_some()
        || opts.tipoff.is_some()
        || opts.cna.is_some()
        || opts.lift_k.is_some();
    #[cfg(feature = "ballistics")]
    if wants_mpm && !use_drag {
        return Err(
            "error: spin, coriolis and tipoff act through the drag adapter; add --diameter-m and --cd"
                .to_string(),
        );
    }

    let mut base = match opts.gravity {
        GravityMode::Central => {
            #[cfg(feature = "ballistics")]
            if use_drag {
                return Err(
                    "error: central gravity and the drag adapter do not mix; use --space-above-m for the atmosphere/space split"
                        .to_string(),
                );
            }
            Forces::Central(match opts.mu {
                Some(mu) => CentralGravity::new(mu, opts.center).map_err(fmt_err)?,
                None => CentralGravity::earth(opts.center).map_err(fmt_err)?,
            })
        }
        GravityMode::Vacuum => Forces::Uniform(UniformGravity(Vec3::zeros())),
        GravityMode::Uniform => {
            #[cfg(feature = "ballistics")]
            if use_drag {
                let (diameter, cd) = match (opts.diameter_m, &opts.cd) {
                    (Some(d), Some(c)) => (d, c.clone()),
                    _ => return Err("error: drag needs both --diameter-m and --cd".to_string()),
                };
                let mut forces = crate::ballistics::BallisticForces::new(
                    diameter,
                    cd,
                    crate::ballistics::Atmosphere::default(),
                    opts.wind_mps,
                )
                .map_err(fmt_err)?;
                forces = forces.with_drag_scale(opts.cd_scale).map_err(fmt_err)?;
                if opts.twist_in.is_some() || opts.body_length_m.is_some() {
                    let (twist, length) = match (opts.twist_in, opts.body_length_m) {
                        (Some(t), Some(l)) => (t, l),
                        _ => {
                            return Err(
                                "error: spin needs both --twist-in and --body-length-m".to_string()
                            )
                        }
                    };
                    forces = forces
                        .with_spin(twist, length, !opts.twist_left)
                        .map_err(fmt_err)?;
                }
                if let Some((lat, az)) = opts.coriolis {
                    forces = forces.with_coriolis(lat, az).map_err(fmt_err)?;
                }
                if let Some((yaw, decay)) = opts.tipoff {
                    forces = forces.with_tipoff_yaw(yaw, decay).map_err(fmt_err)?;
                }
                if opts.cna.is_some() || opts.lift_k.is_some() {
                    let (cna, k) = match (opts.cna.clone(), opts.lift_k.clone()) {
                        (Some(c), Some(k)) => (c, k),
                        _ => {
                            return Err("error: body lift needs both --cna and --lift-k".to_string())
                        }
                    };
                    forces = forces.with_body_lift(cna, k).map_err(fmt_err)?;
                }
                if !opts.atmo_zones.is_empty() {
                    forces = forces
                        .with_atmosphere_zones(opts.atmo_zones.clone())
                        .map_err(fmt_err)?;
                }
                if !opts.wind_zones.is_empty() {
                    forces = forces
                        .with_wind_segments(opts.wind_zones.clone())
                        .map_err(fmt_err)?;
                }
                Forces::Drag(Box::new(forces))
            } else {
                Forces::Uniform(UniformGravity::default())
            }
            #[cfg(not(feature = "ballistics"))]
            {
                Forces::Uniform(UniformGravity::default())
            }
        }
    };

    if let Some(cutoff) = opts.space_above_m {
        let above: Forces = match opts.gravity {
            GravityMode::Central => match opts.mu {
                Some(mu) => Forces::Central(CentralGravity::new(mu, opts.center).map_err(fmt_err)?),
                None => Forces::Central(CentralGravity::earth(opts.center).map_err(fmt_err)?),
            },
            _ => Forces::Uniform(UniformGravity(Vec3::zeros())),
        };
        base = Forces::Switched(
            AltitudeSwitch::new(
                BoxedForces(Box::new(base)),
                BoxedForces(Box::new(above)),
                cutoff,
            )
            .map_err(fmt_err)?,
        );
    }

    if let Some(length) = opts.rail_m {
        base = Forces::Railed(
            RailForces::new(BoxedForces(Box::new(base)), opts.position_m, axis, length)
                .map_err(fmt_err)?,
        );
    }

    let mut guidance = BoxedGuidance(if opts.prograde {
        Box::new(ProgradeDirection::new(axis)) as Box<dyn Guidance>
    } else {
        Box::new(FixedDirection(axis)) as Box<dyn Guidance>
    });
    if opts.steering.norm() > 0.0 {
        guidance = BoxedGuidance(Box::new(WithSteering {
            inner: guidance,
            steering: opts.steering,
        }));
    }
    if let Some((tilt, clock)) = opts.misalign {
        guidance = BoxedGuidance(Box::new(
            ThrustMisalignment::new(guidance, tilt, clock).map_err(fmt_err)?,
        ));
    }
    if let Some(tau) = opts.steering_tau {
        guidance = BoxedGuidance(Box::new(SteeringLag::new(guidance, tau).map_err(fmt_err)?));
    }

    let settings = IntegratorSettings {
        max_steering_accel_mps2: opts.steering_cap.unwrap_or_else(|| opts.steering.norm()),
        ..IntegratorSettings::default()
    };
    let sim = Simulator::new(rocket, base, settings).map_err(fmt_err)?;

    let mut state = FlightState {
        time_s: 0.0,
        position_m: opts.position_m,
        velocity_mps: axis * opts.speed_mps,
    };
    let mut rows = Vec::new();
    let mut substeps = 0usize;
    let sample = |state: &FlightState| -> Result<Row, String> {
        let rocket = sim.rocket().sample(state.time_s).map_err(fmt_err)?;
        Ok(Row {
            t: state.time_s,
            x: state.position_m.x,
            y: state.position_m.y,
            z: state.position_m.z,
            speed: state.velocity_mps.norm(),
            mass: rocket.mass_kg,
            thrust: rocket.motor.thrust_n,
            phase: phase_name(rocket.motor.phase),
        })
    };
    rows.push(sample(&state)?);
    while state.time_s < duration - 1e-12 {
        let step = opts.dt_s.min(duration - state.time_s);
        let result = sim
            .step_guided(&state, step, &mut guidance)
            .map_err(fmt_err)?;
        substeps += result.substeps;
        state = result.state;
        rows.push(sample(&state)?);
        if stop_at_impact && state.time_s > 0.0 && state.position_m.y <= 0.0 {
            break;
        }
    }
    Ok((rows, substeps))
}

fn render(rows: &[Row], opts: &Options, substeps: usize) -> String {
    let mut out = match opts.format {
        Format::Table => render_table(rows),
        Format::Csv => render_csv(rows),
        Format::Json => render_json(rows),
    };
    if opts.format == Format::Table {
        out.push_str(&format!(
            "\n{} samples, {substeps} RK4 substeps\n",
            rows.len()
        ));
        if let Some((apogee, t)) = apogee(rows) {
            out.push_str(&format!("apogee {apogee:.2} m at t = {t:.3} s\n"));
        }
        if let Some(impact) = crate::analysis::impact_row(rows) {
            out.push_str(&format!(
                "impact x = {:.2} m, z = {:.2} m at t = {:.3} s, speed {:.2} m/s\n",
                impact.x, impact.z, impact.t, impact.speed
            ));
        }
    }
    if opts.plot {
        out.push('\n');
        out.push_str(&render_plot(rows));
    }
    out
}

pub(crate) fn apogee(rows: &[Row]) -> Option<(f64, f64)> {
    rows.iter()
        .map(|r| (r.y, r.t))
        .reduce(|a, b| if b.0 > a.0 { b } else { a })
}

fn render_table(rows: &[Row]) -> String {
    let mut out = String::from(
        "     t_s      x_m      y_m      z_m   speed_mps   mass_kg  thrust_n  phase\n",
    );
    for r in rows {
        out.push_str(&format!(
            "{:8.3} {:9.2} {:9.2} {:9.2} {:11.2} {:9.4} {:9.2}  {}\n",
            r.t, r.x, r.y, r.z, r.speed, r.mass, r.thrust, r.phase
        ));
    }
    out
}

fn render_csv(rows: &[Row]) -> String {
    let mut out = String::from("t_s,x_m,y_m,z_m,speed_mps,mass_kg,thrust_n,phase\n");
    for r in rows {
        out.push_str(&format!(
            "{},{},{},{},{},{},{},{}\n",
            r.t, r.x, r.y, r.z, r.speed, r.mass, r.thrust, r.phase
        ));
    }
    out
}

fn render_json(rows: &[Row]) -> String {
    let mut out = String::from("[\n");
    for (i, r) in rows.iter().enumerate() {
        out.push_str(&format!(
            "  {{\"t_s\":{},\"x_m\":{},\"y_m\":{},\"z_m\":{},\"speed_mps\":{},\"mass_kg\":{},\"thrust_n\":{},\"phase\":\"{}\"}}{}\n",
            r.t,
            r.x,
            r.y,
            r.z,
            r.speed,
            r.mass,
            r.thrust,
            r.phase,
            if i + 1 == rows.len() { "" } else { "," }
        ));
    }
    out.push(']');
    out
}

fn render_plot(rows: &[Row]) -> String {
    const HEIGHT: usize = 18;
    const WIDTH: usize = 72;
    let peak = rows.iter().map(|r| r.y).fold(0.0_f64, f64::max);
    if peak <= 0.0 {
        return "altitude never leaves the pad\n".to_string();
    }
    let mut grid = vec![vec![b' '; WIDTH]; HEIGHT];
    for (i, r) in rows.iter().enumerate() {
        let col = i * (WIDTH - 1) / rows.len().max(1);
        let row = ((r.y / peak) * (HEIGHT - 1) as f64).round() as usize;
        grid[HEIGHT - 1 - row][col] = b'*';
    }
    let mut out = format!("altitude_m (peak {peak:.1})\n");
    for line in &grid {
        out.push('|');
        out.push_str(std::str::from_utf8(line).unwrap());
        out.push('\n');
    }
    out.push('+');
    out.push_str(&"-".repeat(WIDTH));
    out.push_str(&format!(
        "\n0{:>width$.1} s\n",
        rows.last().map_or(0.0, |r| r.t),
        width = WIDTH - 3
    ));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(s: &str) -> Vec<String> {
        tokenize(s)
    }

    #[test]
    fn tokenize_honors_quotes() {
        assert_eq!(
            tokenize("simulate --curve '0:0, 1:5' --format json"),
            vec!["simulate", "--curve", "0:0, 1:5", "--format", "json"]
        );
    }

    #[test]
    fn help_lists_simulate() {
        let out = run(&argv("help")).unwrap();
        assert!(out.contains("simulate"));
        assert!(out.contains("--curve"));
    }

    #[test]
    fn missing_curve_is_an_error() {
        let err = run(&argv("simulate --dry-mass 0.18 --propellant-mass 0.04")).unwrap_err();
        assert!(err.contains("--curve is required"));
    }

    #[test]
    fn unknown_command_is_an_error() {
        let err = run(&argv("launch")).unwrap_err();
        assert!(err.contains("unknown command 'launch'"));
    }

    #[test]
    fn simulate_prints_a_table() {
        let out = run(&argv(
            "simulate --curve 0:0,0.1:20,0.9:15,1:0 --dry-mass 0.18 \
             --propellant-mass 0.04 --ignition-s 0.2 --duration-s 1.0 --dt-s 0.25",
        ))
        .unwrap();
        assert!(out.contains("phase"));
        assert!(out.contains("burning"));
        assert!(out.contains("substeps"));
    }

    #[test]
    fn simulate_json_is_machine_readable() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:10,1:0 --dry-mass 0.5 --propellant-mass 0.05 \
             --duration-s 0.4 --dt-s 0.2 --format json",
        ))
        .unwrap();
        assert!(out.starts_with('['));
        assert!(out.contains("\"phase\":\"burning\""));
    }

    #[test]
    fn simulate_csv_has_header() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:10,1:0 --dry-mass 0.5 --propellant-mass 0.05 \
             --duration-s 0.2 --dt-s 0.1 --format csv",
        ))
        .unwrap();
        assert!(out.starts_with("t_s,x_m,y_m,z_m,speed_mps,mass_kg,thrust_n,phase\n"));
    }

    #[test]
    fn plot_marks_peak() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:30,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 2.0 --dt-s 0.5 --plot",
        ))
        .unwrap();
        assert!(out.contains("altitude_m (peak"));
        assert!(out.contains('*'));
    }

    #[cfg(feature = "ballistics")]
    const DRAG: &str = "--diameter-m 0.05 --cd 0:0.45,0.8:0.5,1.2:0.7,3:0.4";

    #[cfg(feature = "ballistics")]
    #[test]
    fn spin_needs_body_length() {
        let err = run(&argv(&format!(
            "simulate --curve 0:0,0.5:30,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.2 {DRAG} --twist-in 10"
        )))
        .unwrap_err();
        assert!(err.contains("--twist-in and --body-length-m"));
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn mpm_options_need_the_drag_adapter() {
        let err = run(&argv(
            "simulate --curve 0:0,0.5:30,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.2 --coriolis 40:90",
        ))
        .unwrap_err();
        assert!(err.contains("--diameter-m and --cd"));
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn full_mpm_run_flies() {
        let out = run(&argv(&format!(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.25 --elevation-deg 60 --speed-mps 50 \
             {DRAG} --twist-in 10 --body-length-m 0.6 --coriolis 40:90 --tipoff 0.02:30"
        )))
        .unwrap();
        assert!(out.contains("burning"));
        assert!(out.contains("substeps"));
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn body_lift_needs_both_tables() {
        let err = run(&argv(&format!(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.2 {DRAG} --cna 0:2,1:3"
        )))
        .unwrap_err();
        assert!(err.contains("--cna and --lift-k"));
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn body_lift_run_flies() {
        let out = run(&argv(&format!(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.25 --elevation-deg 60 --speed-mps 50 \
             {DRAG} --cna 0:2,1:3,3:4 --lift-k 0:0,1:0.5,3:1"
        )))
        .unwrap();
        assert!(out.contains("burning"));
    }

    #[test]
    fn summary_reports_apogee_and_impact() {
        let out = run(&argv(
            "simulate --curve 0:0,0.05:50,0.1:0 --dry-mass 1.0 --propellant-mass 0.01 \
             --speed-mps 120 --elevation-deg 45 --duration-s 60 --dt-s 0.2",
        ))
        .unwrap();
        assert!(out.contains("apogee"), "got {out}");
        assert!(out.contains("impact x ="), "got {out}");
    }

    #[test]
    fn prograde_and_misalignment_fly() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.25 --elevation-deg 60 --speed-mps 50 \
             --prograde --misalign 0.01:0.5",
        ))
        .unwrap();
        assert!(out.contains("burning"));
    }

    #[test]
    fn steering_with_lag_flies() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.25 --elevation-deg 60 --speed-mps 50 \
             --steering 1,0,0 --steering-tau 0.2",
        ))
        .unwrap();
        assert!(out.contains("burning"));
    }

    #[test]
    fn rail_and_gravity_modes_fly() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.5 --dt-s 0.25 --rail 2.0",
        ))
        .unwrap();
        assert!(out.contains("burning"));
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.5 --dt-s 0.25 --gravity central --center 0,-6371000,0",
        ))
        .unwrap();
        assert!(out.contains("burning"));
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.5 --dt-s 0.25 --space-above-m 80000",
        ))
        .unwrap();
        assert!(out.contains("burning"));
    }

    #[test]
    fn staged_vehicle_flies_through_separations() {
        let out = run(&argv(
            "simulate --dry-mass 0.05 \
             --stage dry=0.05,prop=0.04,curve=0:24;0.6:24,ignition=0,sep=0.7 \
             --stage dry=0.04,prop=0.03,curve=0:16;0.5:16,ignition=0.8,sep=1.4 \
             --stage dry=0.03,prop=0.02,curve=0:12;0.4:12,ignition=1.5 \
             --duration-s 2.0 --dt-s 0.25",
        ))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
        assert!(out.contains("burnout"), "got {out}");
    }

    #[test]
    fn staging_rejects_mixed_motor_options() {
        let err = run(&argv(
            "simulate --dry-mass 0.05 --curve 0:0,1:5 --propellant-mass 0.02 \
             --stage dry=0.05,prop=0.04,curve=0:24;0.6:24",
        ))
        .unwrap_err();
        assert!(err.contains("--stage replaces"));
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn zone_flags_fly() {
        let out = run(&argv(&format!(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.25 --elevation-deg 60 --speed-mps 50 \
             {DRAG} --wind-zone 0:0:100:0 --wind-zone 5:0:10000:0 \
             --atmo-zone 15:1013.25:0:10000"
        )))
        .unwrap();
        assert!(out.contains("burning"));
    }
}
