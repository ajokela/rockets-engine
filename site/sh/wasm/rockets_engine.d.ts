/* tslint:disable */
/* eslint-disable */

export class Calculator {
  free(): void;
  [Symbol.dispose](): void;
  setAltitude(altitude_ft: number): Calculator;
  setDiameter(diameter_inches: number): Calculator;
  setHumidity(humidity: number): Calculator;
  setPressure(pressure_inhg: number): Calculator;
  setVelocity(velocity_fps: number): Calculator;
  setMaxRange(range_yards: number): Calculator;
  setDragModel(model: string): Calculator;
  setZeroRange(range_yards: number): Calculator;
  /**
   * Height of the bore above the ground, in inches (default 60 = 5 ft). The solve stops
   * when the projectile falls this far below the muzzle, so raising it pushes the
   * ground-impact cutoff further downrange. Use [`Self::ignore_ground_impact`] to remove
   * the cutoff entirely rather than setting an implausible height.
   */
  setBoreHeight(height_inches: number): Calculator;
  setTemperature(temp_f: number): Calculator;
  /**
   * Add a downrange wind segment: `speed_mph` from `direction_deg`
   * (wind-FROM: 0 = headwind, 90 = from the right) out to `until_yards`.
   * Each segment applies from the previous boundary to `until_yards`; wind is
   * zero beyond the last segment. When any segment is added it overrides the
   * scalar `setWind` value. Repeatable.
   */
  addWindSegment(speed_mph: number, direction_deg: number, until_yards: number): Calculator;
  setSightHeight(height_inches: number): Calculator;
  /**
   * MBA-1368: compass bearing of the shot (degrees, 0 = north, 90 = east) —
   * required by `setWindReference("compass")`; also feeds Coriolis when enabled.
   */
  setShotDirection(bearing_deg: number): Calculator;
  /**
   * MBA-1368: choose the wind entry frame — "shooter" (default; wind-FROM angles
   * relative to the line of fire) or "compass" (earth-fixed bearings, 0 = north,
   * covering `setWind` AND every `addWindSegment` direction, re-referenced against
   * the shot azimuth at solve time). Compass mode requires `setShotDirection`;
   * invalid values surface when the command runs. Additive method — no existing
   * signature changed.
   */
  setWindReference(mode: string): Calculator;
  /**
   * Remove all downrange wind segments (reverts to the scalar `setWind`).
   */
  clearWindSegments(): Calculator;
  enableCoriolis(enabled: boolean, latitude?: number | null): Calculator;
  /**
   * Get full trajectory table as array of points
   * Returns array of: [{ range_yards, drop_inches, windage_inches, velocity_fps, energy_ftlb, time_sec }, ...]
   *
   * ⚠️ DIFFERENT KEYS FROM [`Self::calculate_trajectory`], which answers `drift_inches`
   * and `time_seconds`. The two have never agreed and the difference is load-bearing for
   * existing embedders, so it is documented here rather than quietly unified (MBA-1535).
   */
  getFullTrajectory(): any;
  /**
   * Calculate trajectory and return result as JavaScript object
   * Returns: { range_yards, drop_inches, drift_inches, velocity_fps, energy_ftlb, time_seconds }
   * (`drift_inches`, not `windage_inches` -- the wrong name here is what a wasm test was
   * written against, and it failed on a field that has never existed. MBA-1535.)
   */
  calculateTrajectory(range_yards: number): any;
  /**
   * Solve all the way to the requested range instead of stopping at ground impact.
   *
   * Without this, `calculateTrajectory(range)` returns the closest point it actually
   * solved, which for a typical .308 is around 516 yd — short of a 1000 yd request, and
   * reported only by the `range_yards` field of the returned object.
   */
  ignoreGroundImpact(ignore: boolean): Calculator;
  enableSpinDrift(enabled: boolean, twist_rate?: number | null): Calculator;
  /**
   * Create a new calculator with default values
   * Defaults: .308 Winchester 168gr at 2700 fps, standard atmosphere
   */
  constructor();
  setBC(bc: number): Calculator;
  setMass(mass_grains: number): Calculator;
  setWind(speed_mph: number, direction_deg: number): Calculator;
}

export class WasmBallistics {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Run a command and return the output
   */
  runCommand(command: string): string;
  /**
   * Report whether a BC5D table is currently loaded.
   */
  hasBc5dTable(): boolean;
  /**
   * Report whether a custom drag table is currently loaded.
   */
  hasDragTable(): boolean;
  /**
   * Load a BC5D correction table from the raw bytes of a `bc5d_<caliber>.bin`
   * file. The host (browser `fetch()` or Node `fs`/`fetch`) is responsible
   * for retrieving the file — WASM has no filesystem or network — and passes
   * the bytes here.
   *
   * Once loaded, any `trajectory` run that includes `--use-bc-segments` will
   * apply velocity-dependent BC segments synthesized from this table. Load a
   * table matching the bullet's caliber (e.g. `bc5d_308.bin` for a .308).
   *
   * Returns a short human-readable summary of the loaded table. Replaces any
   * previously loaded table.
   */
  loadBc5dTable(bytes: Uint8Array): string;
  /**
   * Load a custom Mach:Cd drag table (MBA-1328) — a measured or manufacturer-published
   * drag curve (Hornady CDM data, a Lapua/Doppler-radar-derived deck, or your own) — from
   * the raw bytes of a CSV file. The host (browser `fetch()` or Node `fs`/`fetch`) is
   * responsible for retrieving the file — WASM has no filesystem or network — and passes
   * the bytes here, mirroring [`Self::load_bc5d_table`].
   *
   * The bytes must be valid UTF-8 CSV text; parsing is delegated to
   * [`crate::drag::DragTable::from_csv_str`] — the SAME parser native `--drag-table`
   * uses — so the accepted format is identical: two columns `mach,cd` per line, blank
   * lines and `#` comments ignored, a single leading textual header row tolerated, Mach
   * strictly ascending with at least 2 points, Cd finite and > 0.
   *
   * Once loaded, EVERY subsequent `trajectory`, `zero`, `lead`, and `monte-carlo` run
   * applies the table automatically — no `--use-*` gate flag is needed (unlike a loaded
   * BC5D table, which only takes effect with `--use-bc-segments`). A loaded table is a
   * full physical substitute for the G-model + BC (see `calculate_drag_coefficient`):
   * `-b`/`--bc` may still be supplied but is ignored for drag once a table is active
   * (matching the native `--drag-table` CLI semantics documented in CLI_USAGE.md).
   *
   * Returns a short human-readable summary of the loaded table (point count + Mach
   * range). Replaces any previously loaded table.
   *
   * MBA-1409: also accepts `.drg` vendor drag-curve text (the same format the native
   * `--drag-table` CLI accepts by `.drg` file extension) as a fallback. WASM has no
   * filesystem and thus no extension to dispatch on, so the bytes are tried as CSV first
   * (exactly as before this change); only on CSV failure, if the text
   * [`crate::drag_file::looks_like_drg`], it is retried through
   * [`crate::drag_file::parse_drg`]. If both fail, the returned error names both formats.
   */
  loadDragTable(bytes: Uint8Array): string;
  /**
   * Unload any custom drag table previously installed via [`Self::load_drag_table`],
   * reverting every subsequent `trajectory`, `zero`, `lead`, and `monte-carlo` run to the
   * standard G-model + BC drag (the `-b`/`--bc` value with the selected G1/G7 curve).
   *
   * The inverse of `loadDragTable`. It lets a single engine instance alternate between a
   * measured-curve (CDM) solve and a plain G7-BC solve without constructing a second
   * instance: `load → solve CDM → clear → solve G7`, with the G7 half uncontaminated —
   * the same load/clear pattern `clearWindSegments` provides for segmented wind.
   *
   * Returns `true` if a table was loaded (and is now cleared), `false` if none was loaded
   * (a harmless no-op). Idempotent.
   */
  clearDragTable(): boolean;
  constructor();
}

export class WasmRockets {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Run one typed command line and return the text to print.
   */
  runCommand(line: string): string;
  /**
   * Create the terminal handle.
   */
  constructor();
  /**
   * Engine version, for the terminal banner.
   */
  version(): string;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
  readonly memory: WebAssembly.Memory;
  readonly __wbg_wasmrockets_free: (a: number, b: number) => void;
  readonly wasmrockets_runCommand: (a: number, b: number, c: number) => [number, number, number, number];
  readonly wasmrockets_version: (a: number) => [number, number];
  readonly wasmrockets_new: () => number;
  readonly __wbg_calculator_free: (a: number, b: number) => void;
  readonly __wbg_wasmballistics_free: (a: number, b: number) => void;
  readonly calculator_addWindSegment: (a: number, b: number, c: number, d: number) => number;
  readonly calculator_calculateTrajectory: (a: number, b: number) => [number, number, number];
  readonly calculator_clearWindSegments: (a: number) => number;
  readonly calculator_enableCoriolis: (a: number, b: number, c: number, d: number) => number;
  readonly calculator_enableSpinDrift: (a: number, b: number, c: number, d: number) => number;
  readonly calculator_getFullTrajectory: (a: number) => [number, number, number];
  readonly calculator_ignoreGroundImpact: (a: number, b: number) => number;
  readonly calculator_new: () => number;
  readonly calculator_setAltitude: (a: number, b: number) => number;
  readonly calculator_setBC: (a: number, b: number) => number;
  readonly calculator_setBoreHeight: (a: number, b: number) => number;
  readonly calculator_setDiameter: (a: number, b: number) => number;
  readonly calculator_setDragModel: (a: number, b: number, c: number) => number;
  readonly calculator_setHumidity: (a: number, b: number) => number;
  readonly calculator_setMass: (a: number, b: number) => number;
  readonly calculator_setMaxRange: (a: number, b: number) => number;
  readonly calculator_setPressure: (a: number, b: number) => number;
  readonly calculator_setShotDirection: (a: number, b: number) => number;
  readonly calculator_setSightHeight: (a: number, b: number) => number;
  readonly calculator_setTemperature: (a: number, b: number) => number;
  readonly calculator_setVelocity: (a: number, b: number) => number;
  readonly calculator_setWind: (a: number, b: number, c: number) => number;
  readonly calculator_setWindReference: (a: number, b: number, c: number) => number;
  readonly calculator_setZeroRange: (a: number, b: number) => number;
  readonly wasmballistics_clearDragTable: (a: number) => number;
  readonly wasmballistics_hasBc5dTable: (a: number) => number;
  readonly wasmballistics_hasDragTable: (a: number) => number;
  readonly wasmballistics_loadBc5dTable: (a: number, b: number, c: number) => [number, number, number, number];
  readonly wasmballistics_loadDragTable: (a: number, b: number, c: number) => [number, number, number, number];
  readonly wasmballistics_new: () => number;
  readonly wasmballistics_runCommand: (a: number, b: number, c: number) => [number, number, number, number];
  readonly __wbindgen_exn_store: (a: number) => void;
  readonly __externref_table_alloc: () => number;
  readonly __wbindgen_externrefs: WebAssembly.Table;
  readonly __wbindgen_malloc: (a: number, b: number) => number;
  readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
  readonly __externref_table_dealloc: (a: number) => void;
  readonly __wbindgen_free: (a: number, b: number, c: number) => void;
  readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
* Instantiates the given `module`, which can either be bytes or
* a precompiled `WebAssembly.Module`.
*
* @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
*
* @returns {InitOutput}
*/
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
* If `module_or_path` is {RequestInfo} or {URL}, makes a request and
* for everything else, calls `WebAssembly.instantiate` directly.
*
* @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
*
* @returns {Promise<InitOutput>}
*/
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
