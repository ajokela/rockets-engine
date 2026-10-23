#!/usr/bin/env bash
# Build the browser terminal's WASM module (site/sh/wasm/).
#
# --no-default-features then an explicit list: the terminal needs `wasm` (the
# bindings) and `ballistics` (the drag/atmosphere adapter). serde is left off;
# the terminal's JSON output is formatted by hand in src/command.rs.
#
# wasm-pack is deliberately NOT used: its 0.15.0 release passes --out-dir to
# cargo, which current cargo rejects. The two steps it wraps are short, so the
# script runs them directly. The wasm-bindgen CLI version must equal the
# wasm-bindgen crate version pinned in Cargo.toml.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
cd "$repo_root"

out_dir="${1:-site/sh/wasm}"
want_cli="$(grep -m1 '^wasm-bindgen = ' Cargo.toml | sed -E 's/.*"=([0-9.]+)".*/\1/')"

command -v wasm-bindgen >/dev/null 2>&1 || {
  echo "error: wasm-bindgen CLI not found (cargo install wasm-bindgen-cli --version $want_cli)" >&2
  exit 1
}
have_cli="$(wasm-bindgen --version | awk '{print $2}')"
[ "$have_cli" = "$want_cli" ] || {
  echo "error: wasm-bindgen CLI $have_cli != crate $want_cli (cargo install wasm-bindgen-cli --version $want_cli)" >&2
  exit 1
}

cargo build --release --target wasm32-unknown-unknown \
  --no-default-features --features wasm,ballistics

wasm-bindgen --target web \
  --out-dir "$out_dir" --out-name rockets_engine \
  target/wasm32-unknown-unknown/release/rockets_engine.wasm

echo "built: $out_dir/rockets_engine.js + $out_dir/rockets_engine_bg.wasm"
