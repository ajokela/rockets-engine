# OpenRocket flight parity

`Apogee.java` flies OpenRocket's own test fixture — the Estes Alpha III from
`TestRockets.makeEstesAlphaIII()` — on its A8 and D21 test motors with zero
wind and prints both apogees plus the component masses. The apogees are
pinned as constants in `src/command.rs`
(`openrocket_flight_parity_alpha_iii_{a8,d21}`), which fly the same
configuration through the `rockets` CLI and require agreement within 10 %.

## Reproduce

```sh
curl -LO https://github.com/openrocket/openrocket/releases/download/release-24.12/OpenRocket-24.12.jar
javac -cp OpenRocket-24.12.jar Apogee.java
java -cp OpenRocket-24.12.jar:. Apogee
```

CI runs exactly this on every push (the `openrocket-parity` job in
`.github/workflows/ci.yml`): it downloads the pinned jar, runs the driver,
sanity-checks both references within 5 % of the recorded values, then
injects the live apogees into the parity tests via
`ROCKETS_OPENROCKET_APOGEE_M` (A8) and `ROCKETS_OPENROCKET_D21_APOGEE_M`.
Local test runs use the pinned constants when the variables are unset.

Reference results (OpenRocket 24.12, 2026-10-25, rounded fins / SMOOTH
finish / no launch lug):

| Motor | Regime | OpenRocket | rockets-engine | Δ |
| --- | --- | --- | --- | --- |
| A8 (9 N·s) | subsonic, max Mach 0.42 | 409.64 m | 411.96 m | +0.6 % |
| D21 (32 N·s) | transonic, max Mach 0.85 | 697.02 m | 754.66 m | +8.3 % |

## Configuration

The driver boots the OpenRocket core headless (Guice `CoreModule`, like the
unit tests), then modifies the fixture so both sides model the same rocket:

- **Rounded fin cross-section** — the fixture's default SQUARE section adds
  ~(t/MAC)·A_fins/A_ref ≈ 1.0 of pressure drag (OpenRocket's
  `FinSetCalc.calculatePressureCD`); our modeler assumes streamlined fins
  (the rounded-section form of that same formula is what we implement).
- **SMOOTH finish** — ≈20 µm paint in OpenRocket's finish table, matching
  our modeler's default roughness.
- **Launch lug removed** — our geometry modeler has no lug concept.
- **Zero wind, ISA atmosphere, vertical rod** — a clean ascent; the
  parachute ejection delay is set to 1000 s so it never fires.

Masses for the Rust side come from the printed flight mass: structure
24.74 g (nose 4.27 g, fins 13.06 g, everything else 7.41 g), plus the
motor's casing (A8 13.1 g, D21 15.4 g).

## Known remaining differences

- Fin-body interference drag (~0.1 of Cd in OpenRocket) is not modelled.
- Elliptical and parabolic nose wave drag (NASA TR-R-100 tables) is not
  modelled past Mach 1; conical and ogive noses are.
- The residual D21 gap is mostly the transonic drag-rise details above
  Mach 0.85 and skin-friction compressibility corrections.

