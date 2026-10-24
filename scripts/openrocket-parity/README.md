# OpenRocket flight parity

`Apogee.java` flies OpenRocket's own test fixture — the Estes Alpha III from
`TestRockets.makeEstesAlphaIII()` — on its A8 test motor and prints the
apogee, the drag breakdown and the component masses. The printed apogee is
pinned as a constant in `src/command.rs`
(`openrocket_flight_parity_alpha_iii_a8`), which flies the same configuration
through the `rockets` CLI and requires agreement within 10 %.

## Reproduce

```sh
curl -LO https://github.com/openrocket/openrocket/releases/download/release-24.12/OpenRocket-24.12.jar
javac -cp OpenRocket-24.12.jar Apogee.java
java -cp OpenRocket-24.12.jar:. Apogee
```

CI runs exactly this on every push (the `openrocket-parity` job in
`.github/workflows/ci.yml`): it downloads the pinned jar, runs the driver,
sanity-checks the reference within 5 % of 409.63 m, then injects the live
apogee into the parity test via `ROCKETS_OPENROCKET_APOGEE_M`. Local test
runs use the pinned constant when the variable is unset.

Reference result (OpenRocket 24.12, 2026-10-24): **apogee 409.63 m**, max
velocity 143.1 m/s. rockets-engine flies the same flight to 442.76 m (+8 %).

## Configuration

The driver boots the OpenRocket core headless (Guice `CoreModule`, like the
unit tests), then modifies the fixture so both sides model the same rocket:

- **Rounded fin cross-section** — the fixture's default SQUARE section adds
  ~(t/MAC)·A_fins/A_ref ≈ 1.0 of pressure drag (OpenRocket's
  `FinSetCalc.calculatePressureCD`); our modeler assumes streamlined fins.
- **SMOOTH finish** — ≈20 µm paint in OpenRocket's finish table, matching
  our modeler's default roughness.
- **Launch lug removed** — our geometry modeler has no lug concept.
- **Zero wind, ISA atmosphere, vertical rod** — a clean ascent; the
  parachute ejection delay is set to 1000 s so it never fires.

The motor is the fixture's A8 (9 N·s triangle, 3.3 g propellant, 13.1 g
casing). The A8 keeps the flight below Mach 0.45; the fixture's D21 reaches
Mach 0.8, where OpenRocket's pre-transonic pressure rise — deliberately
absent from our Mach-1-clamped modeler — dominates the difference (with the
D21, OpenRocket flies to 655.83 m against our ~900 m).

Masses for the Rust side come from the printed flight mass: structure
24.74 g (nose 4.27 g, fins 13.06 g, everything else 7.41 g), casing 13.1 g.

## Known remaining differences

- Fin thickness pressure drag and fin-body interference (~0.15 of Cd in
  OpenRocket) are not in our modeler; our skin friction runs slightly
  higher than theirs. The two effects roughly cancel at these speeds.
- OpenRocket's launch-rod guidance vs our free start at 90° is negligible
  in zero wind.
