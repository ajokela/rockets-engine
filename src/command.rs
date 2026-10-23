//! Shared command engine behind the `rockets` CLI and the browser terminal.
//!
//! One hand-rolled parser serves both front ends so the native binary and the
//! WASM build can never drift apart: the same argv produces the same text.
//! Native `main` passes `std::env::args`; the terminal passes the typed line.

use crate::{
    ControlInput, Error, FlightState, IntegratorSettings, Motor, Rocket, Simulator, ThrustCurve,
    ThrustPoint, UniformGravity, Vec3,
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
     \x20 simulate [OPTIONS]   integrate a flight and print the trajectory\n\
     \x20 version              print the engine version\n\
     \x20 help                 this text\n\
     \n\
     SIMULATE OPTIONS\n\
     \x20 --curve <t:N,...>        thrust points, seconds:newtons (required)\n\
     \x20 --dry-mass <kg>          vehicle mass without propellant (required)\n\
     \x20 --propellant-mass <kg>   consumable propellant (required)\n\
     \x20 --ignition-s <t>         ignition delay [0]\n\
     \x20 --duration-s <t>         total sim time [burnout + 10 s]\n\
     \x20 --dt-s <t>               output cadence [0.05]\n\
     \x20 --elevation-deg <deg>    launch elevation above horizontal [90]\n\
     \x20 --azimuth-deg <deg>      launch azimuth from +X toward +Z [0]\n\
     \x20 --speed-mps <v>          initial speed along the launch axis [0]\n\
     \x20 --position-m <x,y,z>     initial position [0,0,0]\n\
     \x20 --format table|csv|json  output shape [table]\n\
     \x20 --plot                   append an altitude chart\n\
     \x20                            (ASCII in the CLI, interactive in the browser)\n\
     \n\
     With feature 'ballistics' (default):\n\
     \x20 --diameter-m <d>         body diameter; enables drag with --cd\n\
     \x20 --cd <mach:cd,...>       drag coefficient curve\n\
     \x20 --wind <x,y,z>           air velocity; +X is a tailwind [0,0,0]\n\
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
     EXAMPLE\n\
     \x20 simulate --curve 0:0,0.1:20,0.9:15,1:0 --dry-mass 0.18 \\\n\
     \x20   --propellant-mass 0.04 --ignition-s 0.2 --plot"
        .to_string()
}

struct Options {
    curve: ThrustCurve,
    dry_mass: f64,
    propellant: f64,
    ignition_s: f64,
    duration_s: Option<f64>,
    dt_s: f64,
    elevation_deg: f64,
    azimuth_deg: f64,
    speed_mps: f64,
    position_m: Vec3,
    format: Format,
    plot: bool,
    #[cfg(feature = "ballistics")]
    diameter_m: Option<f64>,
    #[cfg(feature = "ballistics")]
    cd: Option<Vec<(f64, f64)>>,
    #[cfg(feature = "ballistics")]
    wind_mps: Vec3,
    #[cfg(feature = "ballistics")]
    twist_in: Option<f64>,
    #[cfg(feature = "ballistics")]
    body_length_m: Option<f64>,
    #[cfg(feature = "ballistics")]
    twist_left: bool,
    #[cfg(feature = "ballistics")]
    coriolis: Option<(f64, f64)>,
    #[cfg(feature = "ballistics")]
    tipoff: Option<(f64, f64)>,
    #[cfg(feature = "ballistics")]
    cna: Option<Vec<(f64, f64)>>,
    #[cfg(feature = "ballistics")]
    lift_k: Option<Vec<(f64, f64)>>,
}

#[derive(Clone, Copy, PartialEq)]
enum Format {
    Table,
    Csv,
    Json,
}

fn parse_pairs(spec: &str, what: &str) -> Result<Vec<(f64, f64)>, String> {
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

fn simulate(args: &[String]) -> Result<String, String> {
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
    #[cfg(feature = "ballistics")]
    let mut diameter_m = None;
    #[cfg(feature = "ballistics")]
    let mut cd = None;
    #[cfg(feature = "ballistics")]
    let mut wind_mps = Vec3::zeros();
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
            #[cfg(feature = "ballistics")]
            "--diameter-m" => diameter_m = Some(parse_f64(&value(flag)?, flag)?),
            #[cfg(feature = "ballistics")]
            "--cd" => cd = Some(parse_pairs(&value(flag)?, "cd")?),
            #[cfg(feature = "ballistics")]
            "--wind" => wind_mps = parse_vec3(&value(flag)?, flag)?,
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
            other => return Err(format!("error: unknown option '{other}'")),
        }
        i += 1;
    }

    let curve = curve.ok_or("error: --curve is required")?;
    let dry_mass = dry_mass.ok_or("error: --dry-mass is required")?;
    let propellant = propellant.ok_or("error: --propellant-mass is required")?;

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
        #[cfg(feature = "ballistics")]
        diameter_m,
        #[cfg(feature = "ballistics")]
        cd,
        #[cfg(feature = "ballistics")]
        wind_mps,
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
    fly(&opts)
}

struct Row {
    t: f64,
    x: f64,
    y: f64,
    z: f64,
    speed: f64,
    mass: f64,
    thrust: f64,
    phase: &'static str,
}

fn fly(opts: &Options) -> Result<String, String> {
    let motor = Motor::new(opts.curve.clone(), opts.propellant, opts.ignition_s)
        .map_err(|e| format!("error: {e}"))?;
    let burnout = motor.burnout_time_s();
    let rocket = Rocket::new(opts.dry_mass, Some(motor)).map_err(|e| format!("error: {e}"))?;

    let el = opts.elevation_deg.to_radians();
    let az = opts.azimuth_deg.to_radians();
    let axis = Vec3::new(el.cos() * az.cos(), el.sin(), el.cos() * az.sin());
    let control = ControlInput::along(axis);

    let duration = opts.duration_s.unwrap_or(burnout + 10.0);
    let settings = IntegratorSettings::default();

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

    let mut state = FlightState {
        time_s: 0.0,
        position_m: opts.position_m,
        velocity_mps: axis * opts.speed_mps,
    };

    let mut rows = Vec::new();
    let mut substeps = 0usize;

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
        .map_err(|e| format!("error: {e}"))?;
        if opts.twist_in.is_some() || opts.body_length_m.is_some() {
            let (twist, length) = match (opts.twist_in, opts.body_length_m) {
                (Some(t), Some(l)) => (t, l),
                _ => {
                    return Err("error: spin needs both --twist-in and --body-length-m".to_string())
                }
            };
            forces = forces
                .with_spin(twist, length, !opts.twist_left)
                .map_err(|e| format!("error: {e}"))?;
        }
        if let Some((lat, az)) = opts.coriolis {
            forces = forces
                .with_coriolis(lat, az)
                .map_err(|e| format!("error: {e}"))?;
        }
        if let Some((yaw, decay)) = opts.tipoff {
            forces = forces
                .with_tipoff_yaw(yaw, decay)
                .map_err(|e| format!("error: {e}"))?;
        }
        if opts.cna.is_some() || opts.lift_k.is_some() {
            let (cna, k) = match (opts.cna.clone(), opts.lift_k.clone()) {
                (Some(c), Some(k)) => (c, k),
                _ => return Err("error: body lift needs both --cna and --lift-k".to_string()),
            };
            forces = forces
                .with_body_lift(cna, k)
                .map_err(|e| format!("error: {e}"))?;
        }
        let sim = Simulator::new(rocket, forces, settings).map_err(|e| format!("error: {e}"))?;
        run_loop(
            &sim,
            &mut state,
            duration,
            opts.dt_s,
            control,
            &mut rows,
            &mut substeps,
        )?;
    } else {
        let sim = Simulator::new(rocket, UniformGravity::default(), settings)
            .map_err(|e| format!("error: {e}"))?;
        run_loop(
            &sim,
            &mut state,
            duration,
            opts.dt_s,
            control,
            &mut rows,
            &mut substeps,
        )?;
    }

    #[cfg(not(feature = "ballistics"))]
    {
        let sim = Simulator::new(rocket, UniformGravity::default(), settings)
            .map_err(|e| format!("error: {e}"))?;
        run_loop(
            &sim,
            &mut state,
            duration,
            opts.dt_s,
            control,
            &mut rows,
            &mut substeps,
        )?;
    }

    Ok(render(&rows, opts, substeps))
}

fn run_loop<E: crate::ForceModel>(
    sim: &Simulator<E>,
    state: &mut FlightState,
    duration: f64,
    dt: f64,
    control: ControlInput,
    rows: &mut Vec<Row>,
    substeps: &mut usize,
) -> Result<(), String> {
    let sample = |sim: &Simulator<E>, state: &FlightState| -> Result<Row, String> {
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
    rows.push(sample(sim, state)?);
    while state.time_s < duration - 1e-12 {
        let step = dt.min(duration - state.time_s);
        let result = sim.step(state, step, control).map_err(fmt_err)?;
        *substeps += result.substeps;
        *state = result.state;
        rows.push(sample(sim, state)?);
    }
    Ok(())
}

fn fmt_err(e: Error) -> String {
    format!("error: {e}")
}

fn phase_name(phase: crate::MotorPhase) -> &'static str {
    match phase {
        crate::MotorPhase::Unpowered => "unpowered",
        crate::MotorPhase::AwaitingIgnition => "pre-ignition",
        crate::MotorPhase::Burning => "burning",
        crate::MotorPhase::BurnedOut => "burnout",
    }
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
    }
    if opts.plot {
        out.push('\n');
        out.push_str(&render_plot(rows));
    }
    out
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
}
