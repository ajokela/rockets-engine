# Changelog

## [0.21.1] - 2026-11-08

Moves to ballistics-engine 0.50.1 (`=0.50.1`). 0.50.1 changes no physics: it builds `ndarray-npy`
without npz support, so `zip` and its critical security advisories leave the dependency tree,
along with the crates only it pulled in (`flate2`, `zopfli`, `zlib-rs`, `crc32fast`,
`simd-adler32`). Every flight is 0.21.0's to the bit.

### Dependencies
- `ballistics-engine` `=0.50.0` becomes `=0.50.1`. `winds-engine` stays `=0.9.3` (it requires
  `^0.50`, so one engine copy).

### Tests
- The two weather_zones half-pressure tests assert the drag ratio the model computes, 0.4977097
  at the default 50 % humidity, to 1e-12, in place of 0.5 within 5e-3.

### Documentation
- The 0.21.0 entry below misexplained that ratio as the compressibility factor; it now gives
  the cause.

## [0.21.0] - 2026-11-03

Moves to ballistics-engine 0.50.0 (`=0.50.0`) and winds-engine 0.9.3 (`=0.9.3`). A minor bump,
because the default atmosphere changes and the engine's humid air moves humid flights.

### Changed (behaviour, breaking)
- **`Atmosphere::default()` is now 50 % relative humidity**, the engine's default
  (`ballistics::DEFAULT_HUMIDITY_PERCENT`, new). It was 0 %. A caller who wants the old dry
  default writes `Atmosphere::icao_standard(0.0)?`, which stays dry (0 %) at every altitude. The
  CLI's weather station is `icao_standard`, so the `rockets` binary and the browser terminal
  fly dry air exactly as before.
- **Humid air is humid in the sound speed too, and aloft.** The kernels read one per-step lookup,
  upstream's `air_at`: a station's (or a zone's) humidity sets its density and its sound speed, and
  the vapour fraction it implies is held at every altitude. 0.20.5 flew a dry sound speed
  everywhere and a dry lapse above the station. Nothing moves at 0 % humidity.
- Drag, spin, body lift, recovery drag and the weathercock still read one and the same air.
  The kernel measures a point's height from its station, so the adapter subtracts the station
  altitude from the state's Y (absolute altitude) before each kernel call; a dry flight's
  acceleration is 0.20.5's to the bit.
- Atmosphere zones are validated by upstream's `AtmoSock::validate` when attached.
- `Atmosphere::Uniform { density_kg_m3: 0.0, .. }` is the engine's exact vacuum
  (`KernelAtmosphere::uniform(0.0, c)`): gravity only, no drag, as before.

### Measured flight shifts
Measured with a throwaway driver (a boost-coast point-mass rocket, 2 m/s
tailwind, 0.02 s steps, ballistics 0.49.0 vs 0.50.0, 15 C and 1013.25 hPa unless noted).
Short: 24 N for 1 s, 0.04 kg propellant, 0.18 kg dry, 25 mm. Long: 2500 N for 15 s, 15 kg
propellant, 15 kg dry, 120 mm.

| Flight | 0.20.5 | 0.21.0 | Shift |
|---|---|---|---|
| short, dry (0 %) apogee | 436.0816 m | 436.0816 m | 0 (bitwise) |
| long, dry (0 %) apogee | 22070.9199 m | 22070.9199 m | 0 (bitwise) |
| short, `Atmosphere::default()` apogee (was dry 0 %, now 50 %) | 436.0816 m | 436.4437 m | +0.362 m |
| short, `default()` range / flight time | 464.648 m / 19.40 s | 465.395 m / 19.42 s | +0.747 m / +0.02 s |
| long, `default()` apogee | 22070.9199 m | 22054.3078 m | -16.6 m |
| long, `default()` range / flight time | 18506.851 m / 153.48 s | 18474.149 m / 153.40 s | -32.7 m / -0.08 s |
| short, explicit 50 % station, apogee | 436.4460 m | 436.4437 m | -0.002 m |
| long, explicit 50 % station, apogee | 22167.9872 m | 22054.3078 m | -113.7 m |
| short, 30 C / 80 % station, apogee | 443.3105 m | 443.3018 m | -0.009 m |
| long, 30 C / 80 % station, apogee | 22144.6634 m | 21720.9392 m | -423.7 m |

A short flight barely feels the new lookup (the humid sound speed and lapse act over under half a
kilometre); a 22 km flight loses 114 m (50 %) to 424 m (30 C, 80 %) of apogee against what 0.20.5
flew for the same explicit humid station, because the vapour fraction now thins and speeds the air
all the way up. By 10 km a 50 % column is slightly denser than a dry one, as upstream's held-mole
fraction model gives. Command: a throwaway `examples/` driver, run with
`cargo run --release --example <driver>` once on each version.

### Dependencies
- `ballistics-engine` `=0.49.0` becomes `=0.50.0`; `winds-engine` `=0.9.2` becomes `=0.9.3`
  (which requires ballistics-engine `^0.50`, so one engine copy).

### Internal
- The kernel call uses `KernelAtmosphere` (`StationAir` / `KernelAtmosphere::uniform`) in place
  of the 0.49 atmosphere tuple, `air_at` in place of `get_local_atmosphere` (deprecated upstream,
  an error under `-D warnings`).
- Tests: the default-humidity, dry-ICAO, humid-sound-speed and pad-altitude cases are new; the
  zone tests name the default's humidity for a zone meant to mirror the base, and their
  half-pressure tolerance widens from 2e-3 to 5e-3 because the drag ratio is 0.4977, not
  about 0.5. Halving the pressure at the same temperature and 50 % humidity doubles the
  water-vapour mole fraction (0.0084 to 0.0169). Vapour is lighter than dry air, so the density
  falls below half (ratio 0.4983222), and the humid air carries sound faster, so the Mach number
  drops and Cd falls (x0.9987709): 0.4977097 in all, against dry air's 0.4998983 (the
  compressibility factor alone). Corrected in 0.21.1; this note first blamed the compressibility
  factor.
