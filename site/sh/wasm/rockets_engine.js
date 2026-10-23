let wasm;

function addToExternrefTable0(obj) {
    const idx = wasm.__externref_table_alloc();
    wasm.__wbindgen_externrefs.set(idx, obj);
    return idx;
}

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return decodeText(ptr, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function handleError(f, args) {
    try {
        return f.apply(this, args);
    } catch (e) {
        const idx = addToExternrefTable0(e);
        wasm.__wbindgen_exn_store(idx);
    }
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function takeFromExternrefTable0(idx) {
    const value = wasm.__wbindgen_externrefs.get(idx);
    wasm.__externref_table_dealloc(idx);
    return value;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    }
}

let WASM_VECTOR_LEN = 0;

const CalculatorFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_calculator_free(ptr >>> 0, 1));

const WasmBallisticsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_wasmballistics_free(ptr >>> 0, 1));

const WasmRocketsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_wasmrockets_free(ptr >>> 0, 1));

/**
 * Object-oriented calculator for programmatic use
 * Provides a type-safe, fluent API alternative to the CLI interface
 */
export class Calculator {
    static __wrap(ptr) {
        ptr = ptr >>> 0;
        const obj = Object.create(Calculator.prototype);
        obj.__wbg_ptr = ptr;
        CalculatorFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        CalculatorFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_calculator_free(ptr, 0);
    }
    /**
     * @param {number} altitude_ft
     * @returns {Calculator}
     */
    setAltitude(altitude_ft) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setAltitude(ptr, altitude_ft);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} diameter_inches
     * @returns {Calculator}
     */
    setDiameter(diameter_inches) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setDiameter(ptr, diameter_inches);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} humidity
     * @returns {Calculator}
     */
    setHumidity(humidity) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setHumidity(ptr, humidity);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} pressure_inhg
     * @returns {Calculator}
     */
    setPressure(pressure_inhg) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setPressure(ptr, pressure_inhg);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} velocity_fps
     * @returns {Calculator}
     */
    setVelocity(velocity_fps) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setVelocity(ptr, velocity_fps);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} range_yards
     * @returns {Calculator}
     */
    setMaxRange(range_yards) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setMaxRange(ptr, range_yards);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {string} model
     * @returns {Calculator}
     */
    setDragModel(model) {
        const ptr = this.__destroy_into_raw();
        const ptr0 = passStringToWasm0(model, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.calculator_setDragModel(ptr, ptr0, len0);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} range_yards
     * @returns {Calculator}
     */
    setZeroRange(range_yards) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setZeroRange(ptr, range_yards);
        return Calculator.__wrap(ret);
    }
    /**
     * Height of the bore above the ground, in inches (default 60 = 5 ft). The solve stops
     * when the projectile falls this far below the muzzle, so raising it pushes the
     * ground-impact cutoff further downrange. Use [`Self::ignore_ground_impact`] to remove
     * the cutoff entirely rather than setting an implausible height.
     * @param {number} height_inches
     * @returns {Calculator}
     */
    setBoreHeight(height_inches) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setBoreHeight(ptr, height_inches);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} temp_f
     * @returns {Calculator}
     */
    setTemperature(temp_f) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setTemperature(ptr, temp_f);
        return Calculator.__wrap(ret);
    }
    /**
     * Add a downrange wind segment: `speed_mph` from `direction_deg`
     * (wind-FROM: 0 = headwind, 90 = from the right) out to `until_yards`.
     * Each segment applies from the previous boundary to `until_yards`; wind is
     * zero beyond the last segment. When any segment is added it overrides the
     * scalar `setWind` value. Repeatable.
     * @param {number} speed_mph
     * @param {number} direction_deg
     * @param {number} until_yards
     * @returns {Calculator}
     */
    addWindSegment(speed_mph, direction_deg, until_yards) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_addWindSegment(ptr, speed_mph, direction_deg, until_yards);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} height_inches
     * @returns {Calculator}
     */
    setSightHeight(height_inches) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setSightHeight(ptr, height_inches);
        return Calculator.__wrap(ret);
    }
    /**
     * MBA-1368: compass bearing of the shot (degrees, 0 = north, 90 = east) —
     * required by `setWindReference("compass")`; also feeds Coriolis when enabled.
     * @param {number} bearing_deg
     * @returns {Calculator}
     */
    setShotDirection(bearing_deg) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setShotDirection(ptr, bearing_deg);
        return Calculator.__wrap(ret);
    }
    /**
     * MBA-1368: choose the wind entry frame — "shooter" (default; wind-FROM angles
     * relative to the line of fire) or "compass" (earth-fixed bearings, 0 = north,
     * covering `setWind` AND every `addWindSegment` direction, re-referenced against
     * the shot azimuth at solve time). Compass mode requires `setShotDirection`;
     * invalid values surface when the command runs. Additive method — no existing
     * signature changed.
     * @param {string} mode
     * @returns {Calculator}
     */
    setWindReference(mode) {
        const ptr = this.__destroy_into_raw();
        const ptr0 = passStringToWasm0(mode, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.calculator_setWindReference(ptr, ptr0, len0);
        return Calculator.__wrap(ret);
    }
    /**
     * Remove all downrange wind segments (reverts to the scalar `setWind`).
     * @returns {Calculator}
     */
    clearWindSegments() {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_clearWindSegments(ptr);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {boolean} enabled
     * @param {number | null} [latitude]
     * @returns {Calculator}
     */
    enableCoriolis(enabled, latitude) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_enableCoriolis(ptr, enabled, !isLikeNone(latitude), isLikeNone(latitude) ? 0 : latitude);
        return Calculator.__wrap(ret);
    }
    /**
     * Get full trajectory table as array of points
     * Returns array of: [{ range_yards, drop_inches, windage_inches, velocity_fps, energy_ftlb, time_sec }, ...]
     *
     * ⚠️ DIFFERENT KEYS FROM [`Self::calculate_trajectory`], which answers `drift_inches`
     * and `time_seconds`. The two have never agreed and the difference is load-bearing for
     * existing embedders, so it is documented here rather than quietly unified (MBA-1535).
     * @returns {any}
     */
    getFullTrajectory() {
        const ret = wasm.calculator_getFullTrajectory(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Calculate trajectory and return result as JavaScript object
     * Returns: { range_yards, drop_inches, drift_inches, velocity_fps, energy_ftlb, time_seconds }
     * (`drift_inches`, not `windage_inches` -- the wrong name here is what a wasm test was
     * written against, and it failed on a field that has never existed. MBA-1535.)
     * @param {number} range_yards
     * @returns {any}
     */
    calculateTrajectory(range_yards) {
        const ret = wasm.calculator_calculateTrajectory(this.__wbg_ptr, range_yards);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Solve all the way to the requested range instead of stopping at ground impact.
     *
     * Without this, `calculateTrajectory(range)` returns the closest point it actually
     * solved, which for a typical .308 is around 516 yd — short of a 1000 yd request, and
     * reported only by the `range_yards` field of the returned object.
     * @param {boolean} ignore
     * @returns {Calculator}
     */
    ignoreGroundImpact(ignore) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_ignoreGroundImpact(ptr, ignore);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {boolean} enabled
     * @param {number | null} [twist_rate]
     * @returns {Calculator}
     */
    enableSpinDrift(enabled, twist_rate) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_enableSpinDrift(ptr, enabled, !isLikeNone(twist_rate), isLikeNone(twist_rate) ? 0 : twist_rate);
        return Calculator.__wrap(ret);
    }
    /**
     * Create a new calculator with default values
     * Defaults: .308 Winchester 168gr at 2700 fps, standard atmosphere
     */
    constructor() {
        const ret = wasm.calculator_new();
        this.__wbg_ptr = ret >>> 0;
        CalculatorFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} bc
     * @returns {Calculator}
     */
    setBC(bc) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setBC(ptr, bc);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} mass_grains
     * @returns {Calculator}
     */
    setMass(mass_grains) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setMass(ptr, mass_grains);
        return Calculator.__wrap(ret);
    }
    /**
     * @param {number} speed_mph
     * @param {number} direction_deg
     * @returns {Calculator}
     */
    setWind(speed_mph, direction_deg) {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.calculator_setWind(ptr, speed_mph, direction_deg);
        return Calculator.__wrap(ret);
    }
}
if (Symbol.dispose) Calculator.prototype[Symbol.dispose] = Calculator.prototype.free;

export class WasmBallistics {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        WasmBallisticsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_wasmballistics_free(ptr, 0);
    }
    /**
     * Run a command and return the output
     * @param {string} command
     * @returns {string}
     */
    runCommand(command) {
        let deferred3_0;
        let deferred3_1;
        try {
            const ptr0 = passStringToWasm0(command, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.wasmballistics_runCommand(this.__wbg_ptr, ptr0, len0);
            var ptr2 = ret[0];
            var len2 = ret[1];
            if (ret[3]) {
                ptr2 = 0; len2 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred3_0 = ptr2;
            deferred3_1 = len2;
            return getStringFromWasm0(ptr2, len2);
        } finally {
            wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
        }
    }
    /**
     * Report whether a BC5D table is currently loaded.
     * @returns {boolean}
     */
    hasBc5dTable() {
        const ret = wasm.wasmballistics_hasBc5dTable(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Report whether a custom drag table is currently loaded.
     * @returns {boolean}
     */
    hasDragTable() {
        const ret = wasm.wasmballistics_hasDragTable(this.__wbg_ptr);
        return ret !== 0;
    }
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
     * @param {Uint8Array} bytes
     * @returns {string}
     */
    loadBc5dTable(bytes) {
        let deferred3_0;
        let deferred3_1;
        try {
            const ptr0 = passArray8ToWasm0(bytes, wasm.__wbindgen_malloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.wasmballistics_loadBc5dTable(this.__wbg_ptr, ptr0, len0);
            var ptr2 = ret[0];
            var len2 = ret[1];
            if (ret[3]) {
                ptr2 = 0; len2 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred3_0 = ptr2;
            deferred3_1 = len2;
            return getStringFromWasm0(ptr2, len2);
        } finally {
            wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
        }
    }
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
     * @param {Uint8Array} bytes
     * @returns {string}
     */
    loadDragTable(bytes) {
        let deferred3_0;
        let deferred3_1;
        try {
            const ptr0 = passArray8ToWasm0(bytes, wasm.__wbindgen_malloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.wasmballistics_loadDragTable(this.__wbg_ptr, ptr0, len0);
            var ptr2 = ret[0];
            var len2 = ret[1];
            if (ret[3]) {
                ptr2 = 0; len2 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred3_0 = ptr2;
            deferred3_1 = len2;
            return getStringFromWasm0(ptr2, len2);
        } finally {
            wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
        }
    }
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
     * @returns {boolean}
     */
    clearDragTable() {
        const ret = wasm.wasmballistics_clearDragTable(this.__wbg_ptr);
        return ret !== 0;
    }
    constructor() {
        const ret = wasm.wasmballistics_new();
        this.__wbg_ptr = ret >>> 0;
        WasmBallisticsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
}
if (Symbol.dispose) WasmBallistics.prototype[Symbol.dispose] = WasmBallistics.prototype.free;

/**
 * Stateless handle to the engine's command surface.
 */
export class WasmRockets {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        WasmRocketsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_wasmrockets_free(ptr, 0);
    }
    /**
     * Run one typed command line and return the text to print.
     * @param {string} line
     * @returns {string}
     */
    runCommand(line) {
        let deferred3_0;
        let deferred3_1;
        try {
            const ptr0 = passStringToWasm0(line, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.wasmrockets_runCommand(this.__wbg_ptr, ptr0, len0);
            var ptr2 = ret[0];
            var len2 = ret[1];
            if (ret[3]) {
                ptr2 = 0; len2 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred3_0 = ptr2;
            deferred3_1 = len2;
            return getStringFromWasm0(ptr2, len2);
        } finally {
            wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
        }
    }
    /**
     * Create the terminal handle.
     */
    constructor() {
        const ret = wasm.wasmrockets_new();
        this.__wbg_ptr = ret >>> 0;
        WasmRocketsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Engine version, for the terminal banner.
     * @returns {string}
     */
    version() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmrockets_version(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
}
if (Symbol.dispose) WasmRockets.prototype[Symbol.dispose] = WasmRockets.prototype.free;

const EXPECTED_RESPONSE_TYPES = new Set(['basic', 'cors', 'default']);

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && EXPECTED_RESPONSE_TYPES.has(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else {
                    throw e;
                }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }
}

function __wbg_get_imports() {
    const imports = {};
    imports.wbg = {};
    imports.wbg.__wbg___wbindgen_throw_dd24417ed36fc46e = function(arg0, arg1) {
        throw new Error(getStringFromWasm0(arg0, arg1));
    };
    imports.wbg.__wbg_new_1ba21ce319a06297 = function() {
        const ret = new Object();
        return ret;
    };
    imports.wbg.__wbg_new_25f239778d6112b9 = function() {
        const ret = new Array();
        return ret;
    };
    imports.wbg.__wbg_push_7d9be8f38fc13975 = function(arg0, arg1) {
        const ret = arg0.push(arg1);
        return ret;
    };
    imports.wbg.__wbg_set_781438a03c0c3c81 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = Reflect.set(arg0, arg1, arg2);
        return ret;
    }, arguments) };
    imports.wbg.__wbindgen_cast_2241b6af4c4b2941 = function(arg0, arg1) {
        // Cast intrinsic for `Ref(String) -> Externref`.
        const ret = getStringFromWasm0(arg0, arg1);
        return ret;
    };
    imports.wbg.__wbindgen_cast_d6cd19b81560fd6e = function(arg0) {
        // Cast intrinsic for `F64 -> Externref`.
        const ret = arg0;
        return ret;
    };
    imports.wbg.__wbindgen_init_externref_table = function() {
        const table = wasm.__wbindgen_externrefs;
        const offset = table.grow(4);
        table.set(0, undefined);
        table.set(offset + 0, undefined);
        table.set(offset + 1, null);
        table.set(offset + 2, true);
        table.set(offset + 3, false);
    };

    return imports;
}

function __wbg_finalize_init(instance, module) {
    wasm = instance.exports;
    __wbg_init.__wbindgen_wasm_module = module;
    cachedUint8ArrayMemory0 = null;


    wasm.__wbindgen_start();
    return wasm;
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (typeof module !== 'undefined') {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (typeof module_or_path !== 'undefined') {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (typeof module_or_path === 'undefined') {
        module_or_path = new URL('rockets_engine_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync };
export default __wbg_init;
