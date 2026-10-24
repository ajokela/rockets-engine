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
     \x20 --dry-mass <kg>          vehicle mass without propellant (required\n\
     \x20                            unless --geometry-masses)\n\
     \x20 --propellant-mass <kg>   consumable propellant (required)\n\
     \x20 --ignition-s <t>         ignition delay [0]\n\
     \x20 --stage <spec>          repeat for staging; replaces the single-motor\n\
     \x20                            options. spec: dry=kg,prop=kg,curve=t:N;t:N,\n\
     \x20                            ignition=s,sep=s (curve/ignition/sep optional)\n\
     \x20 --motor-eng <text>      inline RASP .eng motor text (curve + propellant)\n\
     \x20 --motor-file <path>     .eng file on disk (native CLI only)\n\
     \x20 --recover <cdA>         parachute Cd·A in m²; recovery drag on descent\n\
     \x20 --recovery-after-s <t>  absolute-time gate for deployment [0=apogee]\n\
     \x20 --airframe <preset>     sport24 or hp54: diameter, Cd, Cnα, CP, CG,\n\
     \x20                            inertia and recovery Cd·A, illustrative only\n\
     \x20 --geometry <spec>       subsonic Barrowman modeler:\n\
     \x20                            nose=shape:L:D,tube=L:D,fins=N:cr:ct:s:m:t\n\
     \x20                            (m = leading-edge sweep; t = thickness, m)\n\
     \x20 --geometry-masses <n,t,f> structural masses (kg) for --geometry;\n\
     \x20                            replaces --dry-mass and estimates\n\
     \x20                            --cg-dry-m/--cg-prop-m/--pitch-inertia\n\
     \x20 --duration-s <t>         total sim time [burnout + 10 s]\n\
     \x20 --dt-s <t>               output cadence [0.05]\n\
     \n\
     LAUNCH & GUIDANCE\n\
     \x20 --elevation-deg <deg>    launch elevation above horizontal [90]\n\
     \x20 --azimuth-deg <deg>      launch azimuth from +X toward +Z [0]\n\
     \x20 --speed-mps <v>          initial speed along the launch axis [0]\n\
     \x20 --position-m <x,y,z>     initial position [0,0,0]\n\
     \x20 --prograde              thrust follows velocity (gravity turn)\n\
     \x20 --weathercock           second-order body-axis dynamics toward the\n\
     \x20                            relative wind (needs --cna, --cp, --cg-dry-m,\n\
     \x20                            --cg-prop-m, --pitch-inertia and drag)\n\
     \x20 --cp <mach:cp,...>       centre of pressure, m aft of nose, vs Mach\n\
     \x20 --cg-dry-m <m>           dry-structure CG, m aft of nose\n\
     \x20 --cg-prop-m <m>          propellant CG, m aft of nose\n\
     \x20 --pitch-inertia <kgm2>   scalar pitch inertia about the CG\n\
     \x20 --damping-ratio <z>      weathercock damping ratio [0.1]\n\
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
     \x20 --turbulence <sigma>     deterministic gust RMS per axis, m/s [0]\n\
     \x20 --turbulence-tau <s>     gust correlation period [2.0]\n\
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
    pub(crate) weathercock: bool,
    pub(crate) cp: Option<Vec<(f64, f64)>>,
    pub(crate) cg_dry_m: Option<f64>,
    pub(crate) cg_prop_m: Option<f64>,
    pub(crate) pitch_inertia: Option<f64>,
    pub(crate) damping_ratio: f64,
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
    pub(crate) recover: Option<f64>,
    pub(crate) recovery_after_s: f64,
    pub(crate) airframe: Option<String>,
    pub(crate) geometry: Option<String>,
    /// nose/tube/fins structural masses (kg) for --geometry.
    pub(crate) geometry_masses: Option<(f64, f64, f64)>,
    /// (casing mass kg, motor length m) retained from a motor file.
    pub(crate) motor_casing: Option<(f64, f64)>,
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
    pub(crate) turbulence: Option<(f64, f64)>,
    #[cfg(feature = "ballistics")]
    pub(crate) turbulence_seed: u64,
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

#[cfg(not(target_arch = "wasm32"))]
fn load_motor_file(path: &str) -> Result<crate::model_rocket::MotorFile, String> {
    crate::model_rocket::MotorFile::from_path(path).map_err(fmt_err)
}

#[cfg(target_arch = "wasm32")]
fn load_motor_file(_path: &str) -> Result<crate::model_rocket::MotorFile, String> {
    Err(
        "error: --motor-file needs the native CLI; use --motor-eng with inline RASP text"
            .to_string(),
    )
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
    let mut weathercock = false;
    let mut cp = None;
    let mut cg_dry_m = None;
    let mut cg_prop_m = None;
    let mut pitch_inertia = None;
    let mut damping_ratio = 0.1;
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
    let mut motor_eng = None;
    let mut motor_file = None;
    let mut recover = None;
    let mut recovery_after_s = 0.0;
    let mut airframe = None;
    let mut geometry = None;
    let mut geometry_masses = None;
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
    let mut turbulence = None;
    #[cfg(feature = "ballistics")]
    let mut turbulence_tau = None;
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
            "--weathercock" => weathercock = true,
            "--cp" => cp = Some(parse_pairs(&value(flag)?, "cp")?),
            "--cg-dry-m" => cg_dry_m = Some(parse_f64(&value(flag)?, flag)?),
            "--cg-prop-m" => cg_prop_m = Some(parse_f64(&value(flag)?, flag)?),
            "--pitch-inertia" => pitch_inertia = Some(parse_f64(&value(flag)?, flag)?),
            "--damping-ratio" => damping_ratio = parse_f64(&value(flag)?, flag)?,
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
            "--motor-eng" => motor_eng = Some(value(flag)?),
            "--motor-file" => motor_file = Some(value(flag)?),
            "--recover" => recover = Some(parse_f64(&value(flag)?, flag)?),
            "--recovery-after-s" => recovery_after_s = parse_f64(&value(flag)?, flag)?,
            "--airframe" => airframe = Some(value(flag)?),
            "--geometry" => geometry = Some(value(flag)?),
            "--geometry-masses" => {
                let raw = value(flag)?;
                let parts: Vec<&str> = raw.split(',').collect();
                if parts.len() != 3 {
                    return Err("error: --geometry-masses needs nose,tube,fins in kg".to_string());
                }
                let nose = parse_f64(parts[0], flag)?;
                let tube = parse_f64(parts[1], flag)?;
                let fins = parse_f64(parts[2], flag)?;
                if nose <= 0.0 || tube <= 0.0 || fins < 0.0 {
                    return Err(
                        "error: --geometry-masses needs positive nose/tube and non-negative fins"
                            .to_string(),
                    );
                }
                geometry_masses = Some((nose, tube, fins));
            }
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
            "--turbulence" => turbulence = Some(parse_f64(&value(flag)?, flag)?),
            #[cfg(feature = "ballistics")]
            "--turbulence-tau" => turbulence_tau = Some(parse_f64(&value(flag)?, flag)?),
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

    let motor_source = match (motor_eng, motor_file) {
        (Some(_), Some(_)) => {
            return Err("error: use only one of --motor-eng and --motor-file".to_string())
        }
        (Some(text), None) => {
            let text = text.replace("\\n", "\n");
            Some(crate::model_rocket::MotorFile::parse(&text).map_err(fmt_err)?)
        }
        (None, Some(path)) => Some(load_motor_file(&path)?),
        (None, None) => None,
    };
    if motor_source.is_some() && (curve.is_some() || propellant.is_some() || !stages.is_empty()) {
        return Err(
            "error: a motor file replaces --curve/--propellant-mass and --stage".to_string(),
        );
    }
    if !stages.is_empty() && (curve.is_some() || propellant.is_some()) {
        return Err("error: --stage replaces --curve/--propellant-mass".to_string());
    }
    let (curve, propellant, motor_casing) = if let Some(file) = motor_source {
        let casing_kg = file.total_mass_kg - file.propellant_mass_kg;
        (
            file.curve.clone(),
            file.propellant_mass_kg,
            Some((casing_kg, file.length_mm / 1000.0)),
        )
    } else if stages.is_empty() {
        let curve = curve.ok_or("error: --curve is required")?;
        let propellant = propellant.ok_or("error: --propellant-mass is required")?;
        (curve, propellant, None)
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
            None,
        )
    };
    let dry_mass = match (dry_mass, geometry_masses) {
        (Some(_), Some(_)) => {
            return Err("error: use either --dry-mass or --geometry-masses".to_string())
        }
        (Some(m), None) => m,
        (None, Some((nose, tube, fins))) => {
            nose + tube + fins + motor_casing.map(|c| c.0).unwrap_or(0.0)
        }
        (None, None) => return Err("error: --dry-mass is required".to_string()),
    };
    if geometry_masses.is_some() && geometry.is_none() {
        return Err("error: --geometry-masses needs --geometry".to_string());
    }
    #[cfg(feature = "ballistics")]
    let turbulence = match turbulence {
        Some(sigma) if sigma >= 0.0 => {
            let tau = turbulence_tau.unwrap_or(2.0);
            if tau <= 0.0 {
                return Err("error: --turbulence-tau must be positive".to_string());
            }
            Some((sigma, tau))
        }
        Some(_) => return Err("error: --turbulence needs a nonnegative sigma".to_string()),
        None => {
            if turbulence_tau.is_some() {
                return Err("error: --turbulence-tau needs --turbulence".to_string());
            }
            None
        }
    };

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
        weathercock,
        cp,
        cg_dry_m,
        cg_prop_m,
        pitch_inertia,
        damping_ratio,
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
        recover,
        recovery_after_s,
        airframe,
        geometry,
        geometry_masses,
        motor_casing,
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
        turbulence,
        #[cfg(feature = "ballistics")]
        turbulence_seed: seed,
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
    #[cfg(feature = "ballistics")]
    Recovered(crate::RecoveryForces<BoxedForces>),
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
            #[cfg(feature = "ballistics")]
            Self::Recovered(f) => f.acceleration(state, mass_kg),
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
            #[cfg(feature = "ballistics")]
            Self::Recovered(f) => f.acceleration_with_axis(state, mass_kg, body_axis),
        }
    }
}

/// Pin the commanded axis to the rail while the rocket is on it: the rail
/// constrains attitude as well as translation. The inner guidance does not
/// tick on the rail, so a weathercock keeps its pad axis instead of
/// relaxing toward a horizontal standing wind.
struct RailedGuidance {
    inner: Box<dyn Guidance>,
    origin_m: Vec3,
    direction: Vec3,
    length_m: f64,
}

impl Guidance for RailedGuidance {
    fn command(&mut self, context: GuidanceContext<'_>) -> crate::Result<ControlInput> {
        let along = (context.state.position_m - self.origin_m).dot(&self.direction);
        if along < self.length_m {
            Ok(ControlInput::along(self.direction))
        } else {
            self.inner.command(context)
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
    if opts.airframe.is_some() && opts.geometry.is_some() {
        return Err("error: use either --airframe or --geometry".to_string());
    }
    #[cfg(feature = "ballistics")]
    let geometry_model = opts
        .geometry
        .as_ref()
        .map(|spec| crate::geometry::parse_geometry(spec).map_err(fmt_err))
        .transpose()?;
    #[cfg(feature = "ballistics")]
    let geometry_analysis = geometry_model.as_ref().map(|g| g.analyze());
    #[cfg(feature = "ballistics")]
    let airframe = opts
        .airframe
        .as_ref()
        .map(|name| {
            crate::Airframe::by_name(name)
                .ok_or_else(|| format!("error: unknown airframe '{name}' (sport24 or hp54)"))
        })
        .transpose()?;
    #[cfg(feature = "ballistics")]
    let (diameter_m, cd, cna, lift_k, cp, cg_dry_m, cg_prop_m, pitch_inertia) = {
        let mut d = opts.diameter_m;
        let mut c = opts.cd.clone();
        let mut cn = opts.cna.clone();
        let mut k = opts.lift_k.clone();
        let mut cp_ = opts.cp.clone();
        let mut cgd = opts.cg_dry_m;
        let mut cgp = opts.cg_prop_m;
        let mut pi = opts.pitch_inertia;
        if let Some(af) = &airframe {
            if d.is_some() || c.is_some() {
                return Err("error: --airframe conflicts with --diameter-m/--cd".to_string());
            }
            d = Some(af.diameter_m);
            c = Some(af.cd_vs_mach.clone());
            if cn.is_none() {
                cn = Some(af.cna_vs_mach.clone());
            }
            if k.is_none() {
                k = Some(af.induced_k_vs_mach.clone());
            }
            if cp_.is_none() {
                cp_ = Some(af.cp_vs_mach.clone());
            }
            if cgd.is_none() {
                cgd = Some(af.cg_dry_m);
            }
            if cgp.is_none() {
                cgp = Some(af.cg_prop_m);
            }
            if pi.is_none() {
                pi = Some(af.pitch_inertia_kg_m2);
            }
        }
        if let Some(analysis) = &geometry_analysis {
            if d.is_some() || c.is_some() {
                return Err("error: --geometry conflicts with --diameter-m/--cd".to_string());
            }
            d = Some(analysis.max_diameter_m);
            c = Some(analysis.cd_vs_mach.clone());
            if cn.is_none() {
                cn = Some(analysis.cna_vs_mach.clone());
            }
            if k.is_none() {
                k = Some(analysis.induced_k_vs_mach.clone());
            }
            if cp_.is_none() {
                cp_ = Some(analysis.cp_vs_mach.clone());
            }
            // The motor sits at the aft end of the tube: default the
            // propellant CG from the retained motor length.
            if cgp.is_none() {
                if let Some((_, length_m)) = opts.motor_casing {
                    cgp = Some(analysis.total_length_m - length_m / 2.0);
                }
            }
            if let Some((nose_kg, tube_kg, fins_kg)) = opts.geometry_masses {
                let casing = opts
                    .motor_casing
                    .map(|(kg, length_m)| (kg, analysis.total_length_m - length_m / 2.0));
                let props = geometry_model
                    .as_ref()
                    .expect("geometry masses imply geometry")
                    .mass_properties(nose_kg, tube_kg, fins_kg, casing);
                if cgd.is_none() {
                    cgd = Some(props.dry_cg_m);
                }
                if pi.is_none() {
                    pi = Some(props.pitch_inertia_kg_m2);
                }
            }
        }
        (d, c, cn, k, cp_, cgd, cgp, pi)
    };
    #[cfg(feature = "ballistics")]
    let use_drag = diameter_m.is_some() || cd.is_some();
    #[cfg(feature = "ballistics")]
    let wants_mpm = opts.twist_in.is_some()
        || opts.body_length_m.is_some()
        || opts.twist_left
        || opts.coriolis.is_some()
        || opts.tipoff.is_some()
        || cna.is_some()
        || lift_k.is_some()
        || opts.weathercock;
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
                let (diameter, cd) = match (diameter_m, &cd) {
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
                if cna.is_some() || lift_k.is_some() {
                    let (cna, k) = match (cna.clone(), lift_k.clone()) {
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
                if let Some((sigma, tau)) = opts.turbulence {
                    forces = forces
                        .with_turbulence(sigma, tau, opts.turbulence_seed)
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

    #[cfg(feature = "ballistics")]
    if let Some(cd_area) = opts.recover {
        base = Forces::Recovered(
            crate::RecoveryForces::new(
                BoxedForces(Box::new(base)),
                cd_area,
                opts.recovery_after_s,
                crate::ballistics::Atmosphere::default(),
                opts.wind_mps,
            )
            .map_err(fmt_err)?,
        );
    }
    #[cfg(not(feature = "ballistics"))]
    if opts.recover.is_some() || opts.airframe.is_some() || opts.geometry.is_some() {
        return Err("error: --recover and --airframe need the 'ballistics' feature".to_string());
    }

    let mut guidance_base: Box<dyn Guidance> = if opts.prograde {
        Box::new(ProgradeDirection::new(axis)) as Box<dyn Guidance>
    } else {
        Box::new(FixedDirection(axis)) as Box<dyn Guidance>
    };
    #[cfg(feature = "ballistics")]
    if opts.weathercock {
        let cna = cna.ok_or("error: --weathercock needs --cna")?;
        let cp = cp.ok_or("error: --weathercock needs --cp")?;
        let params = crate::WeathercockParams::new(
            cna,
            cp,
            cg_dry_m.ok_or("error: --weathercock needs --cg-dry-m")?,
            cg_prop_m.ok_or("error: --weathercock needs --cg-prop-m")?,
            diameter_m.ok_or("error: --weathercock needs --diameter-m")?,
            pitch_inertia.ok_or("error: --weathercock needs --pitch-inertia")?,
            opts.damping_ratio,
        )
        .map_err(fmt_err)?;
        let mut weathercock = crate::Weathercock::new(
            params,
            axis,
            crate::ballistics::Atmosphere::default(),
            opts.wind_mps,
        )
        .map_err(fmt_err)?;
        if let Some((sigma, tau)) = opts.turbulence {
            weathercock = weathercock
                .with_turbulence(sigma, tau, opts.turbulence_seed)
                .map_err(fmt_err)?;
        }
        guidance_base = Box::new(weathercock);
    }
    if let Some(length) = opts.rail_m {
        guidance_base = Box::new(RailedGuidance {
            inner: guidance_base,
            origin_m: opts.position_m,
            direction: axis,
            length_m: length,
        });
    }
    #[cfg(not(feature = "ballistics"))]
    if opts.weathercock {
        return Err("error: --weathercock needs the 'ballistics' feature".to_string());
    }
    let mut guidance = BoxedGuidance(guidance_base);
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

    #[test]
    fn motor_eng_inline_rasp_flies() {
        let out = run(&argv(
            "simulate --motor-eng 'Estes D12 24 70 0 0.01 0.03\n0 0\n0.1 10\n1 0' \
             --dry-mass 0.2 --duration-s 1.0 --dt-s 0.25",
        ))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn openrocket_flight_parity_alpha_iii_a8() {
        // OpenRocket 24.12, its own Estes Alpha III fixture on its A8
        // triangle motor: apogee 409.63 m (rounded fins, smooth finish,
        // launch lug removed, zero wind — the configuration our geometry
        // modeler covers). Reproduce with scripts/openrocket-parity/.
        let out = run(&argv(
            "simulate --motor-eng 'Estes A8 18 70 0 0.0033 0.0164\\n0 0\\n1 9\\n2 0' \
             --geometry nose=ogive:0.07:0.024,tube=0.2:0.024,fins=3:0.05:0.03:0.05:0.02:0.0032 \
             --geometry-masses 0.00427,0.00741,0.01306 --duration-s 15 --dt-s 0.05",
        ))
        .unwrap();
        let apogee: f64 = out
            .split_whitespace()
            .skip_while(|t| *t != "apogee")
            .nth(1)
            .and_then(|s| s.parse().ok())
            .expect("apogee line");
        // CI runs scripts/openrocket-parity and injects the live apogee;
        // locally, fall back to the pinned OpenRocket 24.12 reference.
        let reference: f64 = std::env::var("ROCKETS_OPENROCKET_APOGEE_M")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(409.63);
        assert!(
            (apogee - reference).abs() / reference < 0.10,
            "apogee {apogee} vs OpenRocket {reference}"
        );
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn turbulence_is_deterministic_per_seed() {
        let cmd = "simulate --curve 0:0,0.5:10,1:0 --dry-mass 0.5 --propellant-mass 0.05 \
                   --diameter-m 0.05 --cd 0:0.4,3:0.4 --wind 3,0,0 --turbulence 2 \
                   --duration-s 5 --dt-s 0.5";
        let a = run(&argv(cmd)).unwrap();
        let b = run(&argv(cmd)).unwrap();
        assert_eq!(a, b);
        let c = run(&argv(&format!("{cmd} --seed 9"))).unwrap();
        assert_ne!(a, c);
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn weathercock_flies_in_wind_with_rail_and_turbulence() {
        // On the rail the axis is pinned (the rod constrains attitude), so
        // a standing wind no longer tips the rocket over on the pad.
        let cmd = "simulate --motor-eng 'Estes A8 18 70 0 0.0033 0.0164\\n0 0\\n1 9\\n2 0' \
             --geometry nose=ogive:0.07:0.024,tube=0.2:0.024,fins=3:0.05:0.03:0.05:0.02:0.0032 \
             --geometry-masses 0.00427,0.00741,0.01306 --weathercock --wind 4,0,0 \
             --turbulence 1.5 --rail 1 --recover 0.25 --duration-s 20 --dt-s 0.5";
        let out = run(&argv(cmd)).unwrap();
        let apogee: f64 = out
            .split_whitespace()
            .skip_while(|t| *t != "apogee")
            .nth(1)
            .and_then(|s| s.parse().ok())
            .expect("apogee line");
        assert!(apogee > 300.0, "apogee {apogee}");
        // Deterministic per seed; a different seed gusts differently.
        assert_eq!(out, run(&argv(cmd)).unwrap());
        assert_ne!(out, run(&argv(&format!("{cmd} --seed 9"))).unwrap());
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn turbulence_validates_its_numbers() {
        let base = "simulate --curve 0:0,1:10,1.5:0 --dry-mass 0.5 --propellant-mass 0.05 \
                    --diameter-m 0.05 --cd 0:0.4,3:0.4";
        let err = run(&argv(&format!("{base} --turbulence -1"))).unwrap_err();
        assert!(err.contains("--turbulence"), "got {err}");
        let err = run(&argv(&format!("{base} --turbulence 1 --turbulence-tau 0"))).unwrap_err();
        assert!(err.contains("--turbulence-tau"), "got {err}");
        let err = run(&argv(&format!("{base} --turbulence-tau 2"))).unwrap_err();
        assert!(
            err.contains("--turbulence-tau needs --turbulence"),
            "got {err}"
        );
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn geometry_masses_estimate_cg_and_inertia() {
        // No --dry-mass/--cg-dry-m/--cg-prop-m/--pitch-inertia: the mass
        // model fills them from the geometry and the motor file.
        let out = run(&argv(
            "simulate --motor-eng 'Estes D12 24 70 0 0.01 0.03\\n0 0\\n0.05 20\\n0.2 8\\n1.8 9\\n1.94 0' \
             --geometry nose=conical:0.1:0.024,tube=0.4:0.024,fins=3:0.05:0.03:0.04:0.02:0.002 \
             --geometry-masses 0.012,0.020,0.006 --weathercock --recover 0.25 \
             --duration-s 2.0 --dt-s 0.5",
        ))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
    }

    #[test]
    fn geometry_masses_need_geometry() {
        let err = run(&argv(
            "simulate --curve 0:0,1:10,1.5:0 --propellant-mass 0.01 \
             --geometry-masses 0.012,0.020,0.006",
        ))
        .unwrap_err();
        assert!(
            err.contains("--geometry-masses needs --geometry"),
            "got {err}"
        );
    }

    #[test]
    fn geometry_masses_conflict_with_dry_mass() {
        let err = run(&argv(
            "simulate --curve 0:0,1:10,1.5:0 --propellant-mass 0.01 --dry-mass 0.2 \
             --geometry nose=conical:0.1:0.024,tube=0.4:0.024,fins=3:0.05:0.03:0.04:0.02:0.002 \
             --geometry-masses 0.012,0.020,0.006",
        ))
        .unwrap_err();
        assert!(
            err.contains("use either --dry-mass or --geometry-masses"),
            "got {err}"
        );
    }

    #[test]
    fn motor_eng_escaped_newlines_parse() {
        let out = run(&argv(
            "simulate --motor-eng 'Estes D12 24 70 0 0.01 0.03\\n0 0\\n0.1 10\\n1 0' \
             --dry-mass 0.2 --duration-s 1.0 --dt-s 0.25",
        ))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
    }

    #[test]
    fn motor_eng_conflicts_with_explicit_curve() {
        let err = run(&argv(
            "simulate --motor-eng 'Estes D12 24 70 0 0.01 0.03\n0 0\n0.1 10\n1 0' \
             --dry-mass 0.2 --curve 0:0,1:5 --propellant-mass 0.02",
        ))
        .unwrap_err();
        assert!(err.contains("replaces"));
    }

    #[cfg(feature = "ballistics")]
    #[test]
    fn recovery_and_airframe_fly() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 2.0 --dt-s 0.5 --recover 0.5",
        ))
        .unwrap();
        assert!(out.contains("burnout"));
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.05 --elevation-deg 80 --speed-mps 50 \
             --airframe sport24 --weathercock",
        ))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
        let err = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --airframe sport24 --diameter-m 0.05",
        ))
        .unwrap_err();
        assert!(err.contains("conflicts"));
    }
    #[cfg(feature = "ballistics")]
    #[test]
    fn weathercock_flies_and_needs_its_parameters() {
        let err = run(&argv(&format!(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 0.2 {DRAG} --weathercock"
        )))
        .unwrap_err();
        assert!(err.contains("--cna"));
        let out = run(&argv(&format!(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.05 --elevation-deg 80 --speed-mps 50 \
             {DRAG} --cna 0:2,3:2 --lift-k 0:0,3:0.5 --cp 0:0.6,3:0.6 \
             --cg-dry-m 0.3 --cg-prop-m 0.35 --pitch-inertia 0.05 \
             --damping-ratio 0.3 --weathercock"
        )))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
    }
    #[cfg(feature = "ballistics")]
    #[test]
    fn geometry_modeler_flies() {
        let out = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --duration-s 1.0 --dt-s 0.05 --elevation-deg 80 --speed-mps 50 \
             --geometry nose=conical:0.1:0.024,tube=0.4:0.024,fins=3:0.05:0.03:0.04:0.02:0.002 \
             --weathercock --cg-dry-m 0.3 --cg-prop-m 0.35 --pitch-inertia 0.05",
        ))
        .unwrap();
        assert!(out.contains("burning"), "got {out}");
        let err = run(&argv(
            "simulate --curve 0:0,0.5:200,1:0 --dry-mass 0.3 --propellant-mass 0.05 \
             --geometry nose=conical:0.1:0.024,tube=0.4:0.024,fins=3:0.05:0.03:0.04:0.02 \
             --cd 0:0.45,1:0.5",
        ))
        .unwrap_err();
        assert!(err.contains("conflicts"));
    }
}
