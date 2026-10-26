//! The command line: arguments and what each command does.

use crate::pipeline::{self, Stage, stage_file};
use crate::settings::{self, plate_code};
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
        Command::Compile { input, out, machine, emit, from, at, colour, bed, gcode, tune } => {
            let tuning = tune.as_deref().map(pipeline::read_tuning).transpose()?.unwrap_or_default();
            let machine = load_machine(&machine)?;
            let settings = settings::read(&machine, bed.as_deref()).map_err(|e| e.to_string())?;
            let area = settings.envelope.area;
            let at = match at {
                Some(text) => parse_pair(&text)?,
                None => [(area[0][0] + area[1][0]) / 2.0, (area[0][1] + area[1][1]) / 2.0],
            };
            let output = pipeline::run(input.as_deref(), at, &settings, &tuning, from.as_ref(), &emit)?;
            let violations = layercraft_verify::verify(&output.program, &settings.envelope);
            if let Some(first) = violations.first() {
                for v in &violations {
                    eprintln!("unsafe: op {}: {}", v.op, v.reason);
                }
                return Err(format!(
                    "{} unsafe instructions, the first: {}; nothing written",
                    violations.len(),
                    first.reason
                ));
            }
            let mut keys: Vec<(String, String)> =
                settings.scope.iter().map(|(k, v)| (k.clone(), v.render())).collect();
            keys.sort();
            let header = layercraft_gcode::Header {
                generator: format!("layercraft {}", env!("CARGO_PKG_VERSION")),
                filament_diameter: settings.motion.filament_diameter,
                filament_density: settings.filament_density,
                settings: keys,
            };
            let text = layercraft_gcode::write(&output.program, &header);
            if let Some(path) = &gcode {
                write_file(path, text.as_bytes())?;
            }
            let summary = &output.program.summary;
            let filament_area = std::f64::consts::PI * (settings.motion.filament_diameter / 2.0).powi(2);
            let grams = summary.filament_mm * filament_area * settings.filament_density / 1000.0;
            let empty = layercraft_path::Paths { layers: Vec::new() };
            let paths = output.paths.as_ref().unwrap_or(&empty);
            let settings_json: serde_json::Map<String, serde_json::Value> = settings
                .scope
                .iter()
                .map(|(k, v)| {
                    let json = match v {
                        layercraft_machine::Value::List(items) => {
                            serde_json::Value::Array(items.iter().map(|i| i.render().into()).collect())
                        }
                        other => other.render().into(),
                    };
                    (k.clone(), json)
                })
                .collect();
            let plate = layercraft_package::Plate {
                gcode: text,
                printer_model: settings.printer_model.clone(),
                nozzle_diameter: settings.nozzle_diameter,
                filament: layercraft_package::Filament {
                    kind: settings.filament_kind.clone(),
                    colour,
                    code: settings.filament_code.clone(),
                    used_m: summary.filament_mm / 1000.0,
                    used_g: grams,
                },
                seconds: summary.motion_seconds,
                bed_type: plate_code(&settings.bed_type).to_string(),
                settings_json: serde_json::Value::Object(settings_json).to_string(),
                thumbnail: layercraft_view::thumbnail(paths, 512),
                small_thumbnail: layercraft_view::thumbnail(paths, 128),
                generator: header.generator.clone(),
                client_version: if machine.slicer_version.is_empty() {
                    return Err("the machine description predates layercraft 0.1.3: run `layercraft machine import` again".into());
                } else {
                    machine.slicer_version.clone()
                },
                object_name: input
                    .as_ref()
                    .and_then(|p| p.file_stem())
                    .map(|s| s.to_string_lossy().trim_end_matches(".gcode").to_string())
                    .unwrap_or_else(|| "part".into()),
                layers: summary.layers,
                layer_height: settings.layering.layer_height,
                first_layer_seconds: summary.first_layer_seconds,
                bounds: summary.first_layer_bounds,
            };
            let file = std::fs::File::create(&out).map_err(|e| format!("{}: {e}", out.display()))?;
            layercraft_package::write_package(&plate, std::io::BufWriter::new(file))
                .map_err(|e| e.to_string())?;
            let minutes = summary.motion_seconds / 60.0;
            println!(
                "{}: {} layers, {:.0} mm of filament ({grams:.2} g), {minutes:.1} min of motion (the printer's own start and end not counted), up to z = {:.2} mm",
                out.display(),
                summary.layers,
                summary.filament_mm,
                summary.max_z
            );
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
        Command::View { input, out, machine, layers, tune } => {
            let tuning = tune.as_deref().map(pipeline::read_tuning).transpose()?.unwrap_or_default();
            let paths: layercraft_path::Paths = if input.extension().is_some_and(|e| e == "json") {
                let text =
                    std::fs::read_to_string(&input).map_err(|e| format!("{}: {e}", input.display()))?;
                serde_json::from_str(&text).map_err(|e| format!("{}: {e}", input.display()))?
            } else {
                let machine = load_machine(&machine)?;
                let settings = settings::read(&machine, None).map_err(|e| e.to_string())?;
                let area = settings.envelope.area;
                let at = [(area[0][0] + area[1][0]) / 2.0, (area[0][1] + area[1][1]) / 2.0];
                let output = pipeline::run(Some(&input), at, &settings, &tuning, None, &[])?;
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
