//! The command line: arguments and what each command does.

use crate::deliver::{Delivery, Setup, deliver};
use crate::pipeline::{self, Stage, stage_file};
use crate::settings;
use clap::{Parser, Subcommand};
use layercraft_machine::Machine;
use layercraft_machine::import::{BAMBU_STUDIO_PROFILES, Selection, import};
use std::path::{Path, PathBuf};

#[derive(Parser)]
#[command(name = "layercraft", version, about = "Compile a solid down to a print on a Bambu Lab X2D")]
pub struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// The machine description: printer, process and filament.
    #[command(subcommand)]
    Machine(MachineCommand),
    /// Compile a 3MF (or a stage file) into a .gcode.3mf.
    Compile {
        input: Option<PathBuf>,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value = "x2d")]
        machine: String,
        /// Write a stage: mesh, sections, regions, paths or program, as STAGE=FILE (repeatable);
        /// as JSON, or as a PLY picture when FILE ends in .ply.
        #[arg(long, value_parser = stage_file)]
        emit: Vec<(Stage, PathBuf)>,
        /// Start from a stage file instead of the 3MF, as STAGE=FILE.
        #[arg(long, value_parser = stage_file)]
        from: Option<(Stage, PathBuf)>,
        /// Where the part's centre goes on the plate, as X,Y in mm (default: the plate's centre).
        #[arg(long)]
        at: Option<String>,
        /// The filament's colour for the printer's screen.
        #[arg(long, default_value = "#808080")]
        colour: String,
        /// The plate type, such as "Textured PEI Plate" (default: the printer's).
        #[arg(long)]
        bed: Option<String>,
        /// Also write the G-code as plain text.
        #[arg(long)]
        gcode: Option<PathBuf>,
        /// Vary settings by place: a JSON list of overrides, each `{"within": {"z": [0, 5]},
        /// "set": {"motion": {"lift_radius": 0.6}}}`; boxes are in plate millimetres.
        #[arg(long)]
        tune: Option<PathBuf>,
        /// Keep every layer flat, as a planar slicer does, rather than bend layers to follow
        /// gently sloped tops.
        #[arg(long)]
        flat: bool,
    },
    /// Print several programs together on one plate, layer by layer under one frame: program
    /// stages (.json) and G-code from any slicer (.gcode, .gcode.3mf), each already placed.
    Merge {
        inputs: Vec<PathBuf>,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value = "x2d")]
        machine: String,
        /// The filament's colour for the printer's screen.
        #[arg(long, default_value = "#808080")]
        colour: String,
        /// The plate type, such as "Textured PEI Plate" (default: the printer's).
        #[arg(long)]
        bed: Option<String>,
        /// Also write the G-code as plain text.
        #[arg(long)]
        gcode: Option<PathBuf>,
        /// Vary the motion settings by place, as for `compile`: cooling, here.
        #[arg(long)]
        tune: Option<PathBuf>,
        /// Write the merged program: program=FILE, as JSON or as a PLY picture (repeatable).
        #[arg(long, value_parser = stage_file)]
        emit: Vec<(Stage, PathBuf)>,
    },
    /// Compare two G-code files (.gcode or .gcode.3mf), layer by layer and feature by feature.
    Compare { ours: PathBuf, theirs: PathBuf },
    /// Write a SolidCraft script that draws the toolpaths, one sketch per layer and role.
    View {
        /// A 3MF, or a paths stage file.
        input: PathBuf,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value = "x2d")]
        machine: String,
        /// Only layers FROM..TO (end excluded).
        #[arg(long)]
        layers: Option<String>,
        /// Vary settings by place, as for `compile`.
        #[arg(long)]
        tune: Option<PathBuf>,
        /// Keep every layer flat, as for `compile`.
        #[arg(long)]
        flat: bool,
    },
}

#[derive(Subcommand)]
enum MachineCommand {
    /// Read the profiles of an installed Bambu Studio, once, into ~/.config/layercraft.
    Import {
        #[arg(long, default_value = "x2d")]
        name: String,
        #[arg(long, default_value = BAMBU_STUDIO_PROFILES)]
        profiles: PathBuf,
        #[arg(long, default_value = "Bambu Lab X2D 0.4 nozzle")]
        printer: String,
        #[arg(long, default_value = "0.20mm Standard @BBL X2D")]
        process: String,
        #[arg(long, default_value = "Bambu PLA Basic @BBL X2D 0.4 nozzle")]
        filament: String,
        #[arg(long, default_value = "Direct Drive Standard")]
        variant: String,
    },
    /// Show where a machine description came from.
    Show {
        #[arg(long, default_value = "x2d")]
        name: String,
    },
}

fn machine_file(name: &str) -> Result<PathBuf, String> {
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_') {
        return Err(format!("a machine name is letters, digits, - and _: `{name}`"));
    }
    let base = match std::env::var_os("XDG_CONFIG_HOME") {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(std::env::var_os("HOME").ok_or("HOME is not set")?).join(".config"),
    };
    Ok(base.join("layercraft").join("machines").join(format!("{name}.json")))
}

fn load_machine(name: &str) -> Result<Machine, String> {
    let path = machine_file(name)?;
    let text = std::fs::read_to_string(&path).map_err(|e| {
        format!("{}: {e} (run `layercraft machine import --name {name}` first)", path.display())
    })?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// The machine, its settings and the tuning; with `flat`, layers that never bend.
fn setup(machine: &str, bed: Option<&str>, tune: Option<&Path>, flat: bool) -> Result<Setup, String> {
    let tuning = tune.map(pipeline::read_tuning).transpose()?.unwrap_or_default();
    let machine = load_machine(machine)?;
    let mut settings = settings::read(&machine, bed).map_err(|e| e.to_string())?;
    if flat {
        settings.layering.foliation = None;
    }
    Ok(Setup { machine, settings, tuning })
}

/// The middle of the plate's printable area.
fn plate_centre(s: &Setup) -> [f64; 2] {
    let area = s.settings.envelope.area;
    [(area[0][0] + area[1][0]) / 2.0, (area[0][1] + area[1][1]) / 2.0]
}

fn parse_pair(text: &str) -> Result<[f64; 2], String> {
    let mut parts = text.split(',').map(|v| v.trim().parse::<f64>());
    match (parts.next(), parts.next(), parts.next()) {
        (Some(Ok(x)), Some(Ok(y)), None) if x.is_finite() && y.is_finite() => Ok([x, y]),
        _ => Err(format!("expected X,Y, got `{text}`")),
    }
}

fn write_file(path: &Path, bytes: &[u8]) -> Result<(), String> {
    std::fs::write(path, bytes).map_err(|e| format!("{}: {e}", path.display()))
}

pub fn run(cli: Cli) -> Result<(), String> {
    match cli.command {
        Command::Machine(MachineCommand::Import { name, profiles, printer, process, filament, variant }) => {
            let selection = Selection { machine: printer, process, filament, variant };
            let machine = import(&profiles, &selection).map_err(|e| e.to_string())?;
            let path = machine_file(&name)?;
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
            }
            let json = serde_json::to_string_pretty(&machine).map_err(|e| e.to_string())?;
            write_file(&path, json.as_bytes())?;
            println!("imported {} into {}", machine.source, path.display());
            Ok(())
        }
        Command::Machine(MachineCommand::Show { name }) => {
            let machine = load_machine(&name)?;
            println!("{}\nvariant: {}", machine.source, machine.variant);
            Ok(())
        }
        Command::Compile { input, out, machine, emit, from, at, colour, bed, gcode, tune, flat } => {
            let s = setup(&machine, bed.as_deref(), tune.as_deref(), flat)?;
            let at = match at {
                Some(text) => parse_pair(&text)?,
                None => plate_centre(&s),
            };
            let output = pipeline::run(input.as_deref(), at, &s.settings, &s.tuning, from.as_ref(), &emit)?;
            let object_name = input
                .as_ref()
                .and_then(|p| p.file_stem())
                .map(|s| s.to_string_lossy().trim_end_matches(".gcode").to_string())
                .unwrap_or_else(|| "part".into());
            let delivery = Delivery { setup: &s, colour, object_name, gcode: gcode.as_deref(), out: &out };
            println!("{}", deliver(output.program, &delivery)?);
            Ok(())
        }
        Command::Merge { inputs, out, machine, colour, bed, gcode, tune, emit } => {
            if let Some((stage, _)) = emit.iter().find(|(s, _)| *s != Stage::Program) {
                return Err(format!("merge writes the program stage only, not {stage:?}"));
            }
            let s = setup(&machine, bed.as_deref(), tune.as_deref(), false)?;
            let programs = inputs.iter().map(|p| pipeline::read_program(p)).collect::<Result<Vec<_>, _>>()?;
            let merged = layercraft_motion::merge(&programs, s.settings.motion.deretract_speed);
            let limits = &s.settings.motion.limits;
            pipeline::emit(Stage::Program, &merged, || layercraft_view::program(&merged, limits), &emit)?;
            let object_name = "plate".into();
            let delivery = Delivery { setup: &s, colour, object_name, gcode: gcode.as_deref(), out: &out };
            println!("{}", deliver(merged, &delivery)?);
            Ok(())
        }
        Command::Compare { ours, theirs } => {
            let report = crate::compare::report(
                &crate::compare::read_gcode(&ours)?,
                &crate::compare::read_gcode(&theirs)?,
            );
            print!("{report}");
            Ok(())
        }
        Command::View { input, out, machine, layers, tune, flat } => {
            let paths: layercraft_path::Paths = if input.extension().is_some_and(|e| e == "json") {
                pipeline::read_json(&input)?
            } else {
                let s = setup(&machine, None, tune.as_deref(), flat)?;
                let output =
                    pipeline::run(Some(&input), plate_centre(&s), &s.settings, &s.tuning, None, &[])?;
                output.paths.ok_or("no toolpaths")?
            };
            let range = match layers {
                Some(text) => {
                    let (a, b) = text.split_once("..").ok_or("layers are FROM..TO")?;
                    a.trim().parse::<usize>().map_err(|e| e.to_string())?
                        ..b.trim().parse::<usize>().map_err(|e| e.to_string())?
                }
                None => 0..usize::MAX,
            };
            let script = layercraft_view::solidcraft_script(&paths, range);
            write_file(&out, script.to_string().as_bytes())?;
            println!("wrote {}", out.display());
            Ok(())
        }
    }
}
