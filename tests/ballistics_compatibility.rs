#![cfg(feature = "ballistics")]

use ballistics_engine::{
    atmosphere::calculate_air_density_cimp,
    constants::G_ACCEL_MPS2,
    drag::DragTable,
    trajectory_integration::{try_integrate_trajectory, TrajectoryParams},
    wind::{wind_vector, WindSegment},
    DragModel,
};
use rockets_engine::{
    ballistics::{Atmosphere, BallisticForces},
    *,
};

fn deck() -> Vec<(f64, f64)> {
    vec![
        (0.0, 0.2),
        (0.8, 0.24),
        (1.0, 0.38),
        (1.3, 0.30),
        (3.0, 0.22),
    ]
}

fn state() -> FlightState {
    FlightState {
        time_s: 0.0,
        position_m: Vec3::new(0.0, 20.0, 0.0),
        velocity_mps: Vec3::new(150.0, 90.0, 3.0),
    }
}

fn reference_params(
    mass: f64,
    diameter: f64,
    atmos_params: (f64, f64, f64, f64),
    wind_segments: Vec<WindSegment>,
) -> TrajectoryParams {
    let (mach, cd) = deck().into_iter().unzip();
    TrajectoryParams {
        mass_kg: mass,
        bc: 0.5,
        drag_model: DragModel::G1,
        wind_segments,
        atmos_params,
        omega_vector: None,
        enable_spin_drift: false,
        enable_magnus: false,
        enable_coriolis: false,
        target_distance_m: 1e6,
        enable_wind_shear: false,
        wind_shear_model: "none".into(),
        shooter_altitude_m: 0.0,
        is_twist_right: true,
        shooting_angle: 0.0,
        bullet_diameter: diameter,
        bullet_length: diameter,
        twist_rate: 12.0,
        custom_drag_table: Some(DragTable::try_new(mach, cd).unwrap()),
        cd_scale: 1.0,
        custom_drag_reference: false,
        bc_segments: None,
        use_bc_segments: false,
        bc_segments_data: None,
        ground_threshold: -1e6,
        atmo_sock: None,
    }
}

#[test]
fn unpowered_flight_matches_upstream_rk4_across_drag_wind_and_atmospheres() {
    let scenarios = [
        (0.2, 0.025, 0.0, 0.0, 0.0, 0.0),
        (0.5, 0.04, 30.0, 35.0, 2.0, 1000.0),
        (1.0, 0.05, 20.0, 180.0, -1.0, 0.0),
    ];
    for (mass, diameter, wind_kmh, angle_deg, updraft, station_altitude) in scenarios {
        for uniform in [true, false] {
            let (atmosphere, params) = if uniform {
                (
                    Atmosphere::Uniform {
                        density_kg_m3: 1.1,
                        speed_of_sound_mps: 340.0,
                    },
                    (1.1, 340.0, 0.0, 0.0),
                )
            } else {
                (
                    Atmosphere::Standard {
                        station_altitude_m: station_altitude,
                        temperature_c: 12.0,
                        pressure_hpa: 950.0,
                        humidity_percent: 60.0,
                    },
                    (
                        station_altitude,
                        12.0,
                        950.0,
                        calculate_air_density_cimp(12.0, 950.0, 60.0) / 1.225,
                    ),
                )
            };
            let segment = WindSegment {
                speed_kmh: wind_kmh,
                angle_deg,
                until_m: 1e6,
                vertical_mps: updraft,
            };
            // The upstream integration wrapper uses this conversion factor.
            let wind = wind_vector(wind_kmh * 0.2777778, angle_deg.to_radians(), updraft);
            let forces = BallisticForces::new(diameter, deck(), atmosphere, wind).unwrap();
            let sim = Simulator::new(
                Rocket::new(mass, None).unwrap(),
                forces,
                IntegratorSettings {
                    max_step_s: 0.001,
                    ..Default::default()
                },
            )
            .unwrap();
            let mut initial = state();
            // Upstream raw integration treats Y as relative to atmos_params.0;
            // our adapter accepts absolute altitude. Direct mode ignores altitude.
            let altitude_origin = if uniform { params.0 } else { station_altitude };
            initial.position_m.y += altitude_origin;
            let upstream = try_integrate_trajectory(
                [0.0, 20.0, 0.0, 150.0, 90.0, 3.0],
                (0.0, 2.0),
                reference_params(mass, diameter, params, vec![segment]),
                "RK4",
                1e-9,
                0.001,
            )
            .unwrap();
            let result = sim.step(&initial, 2.0, ControlInput::default()).unwrap();
            let (reference_time, reference_state) = upstream.last().unwrap();
            assert_eq!(*reference_time, result.state.time_s);
            let expected_pos = Vec3::new(
                reference_state[0],
                reference_state[1] + altitude_origin,
                reference_state[2],
            );
            let expected_vel =
                Vec3::new(reference_state[3], reference_state[4], reference_state[5]);
            assert!(
                (result.state.position_m - expected_pos).norm() < 2e-9,
                "position mismatch: {:?} vs {expected_pos:?}",
                result.state.position_m
            );
            assert!((result.state.velocity_mps - expected_vel).norm() < 2e-9);
        }
    }
}

#[test]
fn drag_responds_to_mass_wind_and_curve_scale_without_changing_gravity() {
    let forces = BallisticForces::new(0.03, deck(), Atmosphere::default(), Vec3::zeros()).unwrap();
    let gravity = Vec3::new(0.0, -G_ACCEL_MPS2, 0.0);
    let light_drag = forces.acceleration(&state(), 0.2).unwrap() - gravity;
    let heavy_drag = forces.acceleration(&state(), 0.4).unwrap() - gravity;
    assert!((light_drag - heavy_drag * 2.0).norm() < 1e-12);
    let scaled_drag = forces
        .clone()
        .with_drag_scale(2.0)
        .unwrap()
        .acceleration(&state(), 0.2)
        .unwrap()
        - gravity;
    assert!((scaled_drag - light_drag * 2.0).norm() < 1e-12);
    let zero_air_velocity = forces
        .acceleration_in_wind(&state(), 0.2, state().velocity_mps)
        .unwrap();
    assert_eq!(zero_air_velocity, gravity);
    let mut windy = forces;
    windy.set_wind(Vec3::new(0.0, 0.0, 10.0)).unwrap();
    assert!(windy.acceleration(&state(), 0.2).unwrap().z > 0.0);
}

#[test]
fn powered_atmospheric_flight_converges_and_tracks_mass_through_burnout() {
    let motor = Motor::new(
        ThrustCurve::new(vec![
            ThrustPoint {
                time_s: 0.0,
                thrust_n: 10.0,
            },
            ThrustPoint {
                time_s: 0.15,
                thrust_n: 20.0,
            },
            ThrustPoint {
                time_s: 0.7,
                thrust_n: 8.0,
            },
        ])
        .unwrap(),
        0.05,
        0.1,
    )
    .unwrap();
    let rocket = Rocket::new(0.15, Some(motor)).unwrap();
    let forces = BallisticForces::new(
        0.025,
        deck(),
        Atmosphere::default(),
        Vec3::new(-2.0, 0.0, 3.0),
    )
    .unwrap();
    let run = |dt| {
        Simulator::new(
            rocket.clone(),
            forces.clone(),
            IntegratorSettings {
                max_step_s: dt,
                ..Default::default()
            },
        )
        .unwrap()
        .step(&state(), 2.0, ControlInput::along(Vec3::new(1.0, 1.0, 0.0)))
        .unwrap()
    };
    let coarse = run(0.02);
    let fine = run(0.01);
    let reference = run(0.001);
    let coarse_error = (coarse.state.position_m - reference.state.position_m).norm();
    let fine_error = (fine.state.position_m - reference.state.position_m).norm();
    assert!(
        coarse_error > fine_error * 8.0,
        "coarse {coarse_error}, fine {fine_error}"
    );
    assert!(fine_error < 1e-6);
    assert_eq!(fine.rocket.mass_kg, 0.15);
    assert_eq!(fine.rocket.motor.phase, MotorPhase::BurnedOut);
}

#[test]
fn uniform_vacuum_adapter_retains_gravity() {
    let forces = BallisticForces::new(
        0.03,
        deck(),
        Atmosphere::Uniform {
            density_kg_m3: 0.0,
            speed_of_sound_mps: 340.0,
        },
        Vec3::zeros(),
    )
    .unwrap();
    assert_eq!(
        forces.acceleration(&state(), 0.2).unwrap(),
        Vec3::new(0.0, -G_ACCEL_MPS2, 0.0)
    );
}

#[test]
fn rejects_invalid_adapter_inputs_instead_of_triggering_upstream_fallbacks() {
    for atmosphere in [
        Atmosphere::Uniform {
            density_kg_m3: 2.0,
            speed_of_sound_mps: 340.0,
        },
        Atmosphere::Uniform {
            density_kg_m3: 1.0,
            speed_of_sound_mps: 200.0,
        },
        Atmosphere::Uniform {
            density_kg_m3: f64::NAN,
            speed_of_sound_mps: 340.0,
        },
        Atmosphere::Standard {
            station_altitude_m: 0.0,
            temperature_c: 15.0,
            pressure_hpa: 0.0,
            humidity_percent: 0.0,
        },
        Atmosphere::Standard {
            station_altitude_m: 0.0,
            temperature_c: -273.15,
            pressure_hpa: 1000.0,
            humidity_percent: 0.0,
        },
        Atmosphere::Standard {
            station_altitude_m: 0.0,
            temperature_c: 15.0,
            pressure_hpa: 1000.0,
            humidity_percent: 101.0,
        },
    ] {
        assert!(BallisticForces::new(0.03, deck(), atmosphere, Vec3::zeros()).is_err());
    }
    for diameter in [0.0, -0.03, f64::INFINITY] {
        assert!(
            BallisticForces::new(diameter, deck(), Atmosphere::default(), Vec3::zeros()).is_err()
        );
    }
    for bad_deck in [
        vec![],
        vec![(0.0, 0.2)],
        vec![(0.0, 0.2), (0.0, 0.3)],
        vec![(0.0, -0.2), (1.0, 0.3)],
    ] {
        assert!(
            BallisticForces::new(0.03, bad_deck, Atmosphere::default(), Vec3::zeros()).is_err()
        );
    }
    let forces = BallisticForces::new(0.03, deck(), Atmosphere::default(), Vec3::zeros()).unwrap();
    assert!(forces.clone().with_drag_scale(0.0).is_err());
    assert!(forces.acceleration(&state(), 0.0).is_err());
    assert!(forces
        .acceleration_in_wind(&state(), 0.2, Vec3::repeat(f64::NAN))
        .is_err());
}
