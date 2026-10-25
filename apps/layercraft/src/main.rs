//! layercraft: compile a solid down to a print on a Bambu Lab X2D, with every stage open.
//!
//!   layercraft machine import [--name x2d]      read the printer, process and filament once
//!   layercraft compile part.3mf --out part.gcode.3mf [--emit paths=p.json] [--from paths=p.json]
//!   layercraft view part.3mf --out view.json   a SolidCraft script drawing the toolpaths
//!   layercraft compare ours.gcode.3mf theirs.gcode   side by side, layer by layer
//!
//! Nothing here sends a file to a printer or starts a print.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod commands;
mod compare;
mod pipeline;
mod settings;

use clap::Parser;

fn main() -> std::process::ExitCode {
    let cli = commands::Cli::parse();
    // A bug must not take the user's files down with it: report it as an error.
    match std::panic::catch_unwind(|| commands::run(cli)) {
        Ok(Ok(())) => std::process::ExitCode::SUCCESS,
        Ok(Err(e)) => {
            eprintln!("error: {e}");
            std::process::ExitCode::FAILURE
        }
        Err(_) => {
            eprintln!("error: internal error (a bug in layercraft); nothing was written to the printer");
            std::process::ExitCode::FAILURE
        }
    }
}
