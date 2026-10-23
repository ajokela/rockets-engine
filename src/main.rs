//! The `rockets` command line tool: a thin shell over [`rockets_engine::command`].
//!
//! The parser and formatter live in the library so the browser terminal runs
//! the identical command surface. This shim only supplies argv and exit codes.

use rockets_engine::command;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match command::run(&args) {
        Ok(text) => {
            println!("{text}");
        }
        Err(text) => {
            eprintln!("{text}");
            std::process::exit(2);
        }
    }
}
