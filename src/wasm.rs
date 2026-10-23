//! Browser-terminal bindings: one struct, one method.
//!
//! The terminal page (site/sh/index.html) instantiates `WasmRockets`, passes
//! each typed line to `runCommand` and prints the returned text. All parsing
//! and formatting lives in [`crate::command`], shared with the native CLI.

use wasm_bindgen::prelude::*;

/// Stateless handle to the engine's command surface.
#[wasm_bindgen]
pub struct WasmRockets;

#[wasm_bindgen]
impl WasmRockets {
    /// Create the terminal handle.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self
    }

    /// Run one typed command line and return the text to print.
    #[wasm_bindgen(js_name = runCommand)]
    pub fn run_command(&self, line: &str) -> Result<String, JsValue> {
        let args = crate::command::tokenize(line);
        crate::command::run(&args).map_err(|e| JsValue::from_str(&e))
    }

    /// Engine version, for the terminal banner.
    #[wasm_bindgen(js_name = version)]
    pub fn version(&self) -> String {
        env!("CARGO_PKG_VERSION").to_string()
    }
}

impl Default for WasmRockets {
    fn default() -> Self {
        Self::new()
    }
}
