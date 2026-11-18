//! The command line: arguments and what each command does.

use crate::deliver::{Delivery, Setup, deliver};
use crate::pipeline::{self, Stage, stage_file};
use crate::settings;
use clap::{Parser, Subcommand};
use layercraft_path::SparsePattern;
use std::path::{Path, PathBuf};

/// The colour of a filament the design gives none.
const GREY: &str = "#808080";

/// Colours given on the command line, comma separated, each `#RRGGBB`.
fn colours(text: &str) -> Result<Vec<String>, String> {
    text.split(',')
        .map(|c| {
            let c = c.trim();
            let hex = c.strip_prefix('#').unwrap_or_default();
            if hex.len() == 6 && hex.chars().all(|h| h.is_ascii_hexdigit()) {
                Ok(format!("#{}", hex.to_ascii_uppercase()))
            } else {
                Err(format!("`{c}` is not a colour #RRGGBB"))
            }
        })
        .collect()
}

#[derive(Parser)]
#[command(name = "layercraft", version, about = "Compile a solid down to a print on a Bambu Lab X2D")]
pub struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Compile a 3MF (or a stage file) into a .gcode.3mf.
    Compile {
        input: Option<PathBuf>,
        #[arg(long)]
        out: PathBuf,
        /// The print quality: 0.20 or 0.12, its layer height in mm.
        #[arg(long, default_value = "0.20")]
        quality: String,
        /// The filament: pla (Bambu PLA Basic) or petg-tr (Bambu PETG Translucent).
        #[arg(long, default_value = "pla")]
        material: String,
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
        /// The filaments' colours for the printer's screen, one per material, comma separated
        /// (default: the 3MF's colours, grey without).
        #[arg(long)]
        colour: Option<String>,
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
        /// What sparse areas are filled with: convection rolls from wall to wall, stacked,
        /// tilted the other way every five layers, or straight lines crossing from layer to
        /// layer (default: rolls).
        #[arg(long, value_enum)]
        infill: Option<Infill>,
    },
    /// Print several programs together on one plate, layer by layer under one frame: program
    /// stages (.json) and G-code from any slicer (.gcode, .gcode.3mf), each already placed.
    Merge {
        inputs: Vec<PathBuf>,
        #[arg(long)]
        out: PathBuf,
        /// The print quality: 0.20 or 0.12, its layer height in mm.
        #[arg(long, default_value = "0.20")]
        quality: String,
        /// The filament: pla (Bambu PLA Basic) or petg-tr (Bambu PETG Translucent).
        #[arg(long, default_value = "pla")]
        material: String,
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
        /// The print quality: 0.20 or 0.12, its layer height in mm.
        #[arg(long, default_value = "0.20")]
        quality: String,
        /// The filament: pla (Bambu PLA Basic) or petg-tr (Bambu PETG Translucent).
        #[arg(long, default_value = "pla")]
        material: String,
        /// Only layers FROM..TO (end excluded).
        #[arg(long)]
        layers: Option<String>,
        /// Vary settings by place, as for `compile`.
        #[arg(long)]
        tune: Option<PathBuf>,
        /// Keep every layer flat, as for `compile`.
        #[arg(long)]
        flat: bool,
        /// What sparse areas are filled with, as for `compile`.
        #[arg(long, value_enum)]
        infill: Option<Infill>,
    },
}

/// What sparse areas are filled with.
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum Infill {
    Lines,
    Rolls,
}

/// How a compile is made: layers kept `flat` or not, the sparse areas filled with `infill` or
/// with the settings' rolls. Supports are no choice: whatever hangs stands on them.
#[derive(Clone, Copy)]
struct Choices {
    flat: bool,
    infill: Option<Infill>,
}

/// The printer in `quality` and `material`, its settings and the tuning, as `choices` make them.
fn setup(
    quality: &str,
    material: &str,
    bed: Option<&str>,
    tune: Option<&Path>,
    choices: Choices,
) -> Result<Setup, String> {
    let Choices { flat, infill } = choices;
    let tuning = tune.map(pipeline::read_tuning).transpose()?.unwrap_or_default();
    let machine = crate::printers::machine(quality, material).map_err(|e| e.to_string())?;
    let mut settings = settings::read(&machine, bed).map_err(|e| e.to_string())?;
    if flat {
        settings.layering.foliation = None;
    }
    match infill {
        Some(Infill::Lines) => settings.path.sparse_pattern = SparsePattern::Lines,
        Some(Infill::Rolls) => settings.path.sparse_pattern = settings::rolls(settings.layering.layer_height),
        None => {}
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
        Command::Compile {
            input,
            out,
            quality,
            material,
            emit,
            from,
            at,
            colour,
            bed,
            gcode,
            tune,
            flat,
            infill,
        } => {
            let s = setup(&quality, &material, bed.as_deref(), tune.as_deref(), Choices { flat, infill })?;
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
            let colours = match colour {
                Some(text) => colours(&text)?,
                None if !output.materials.is_empty() => {
                    output.materials.iter().map(|m| m.colour.clone().unwrap_or_else(|| GREY.into())).collect()
                }
                None => vec![GREY.into()],
            };
            let delivery = Delivery { setup: &s, colours, object_name, gcode: gcode.as_deref(), out: &out };
            println!("{}", deliver(output.program, &delivery)?);
            Ok(())
        }
        Command::Merge { inputs, out, quality, material, colour, bed, gcode, tune, emit } => {
            if let Some((stage, _)) = emit.iter().find(|(s, _)| *s != Stage::Program) {
                return Err(format!("merge writes the program stage only, not {stage:?}"));
            }
            let choices = Choices { flat: false, infill: None };
            let s = setup(&quality, &material, bed.as_deref(), tune.as_deref(), choices)?;
            let programs = inputs.iter().map(|p| pipeline::read_program(p)).collect::<Result<Vec<_>, _>>()?;
            if programs.iter().flat_map(|p| &p.layers).any(|l| l.material != 0) {
                return Err("not supported yet: merging programs printed in several materials".into());
            }
            let merged = layercraft_motion::merge(&programs, s.settings.motion.deretract_speed);
            let limits = &s.settings.motion.limits;
            pipeline::emit(Stage::Program, &merged, || layercraft_view::program(&merged, limits), &emit)?;
            let object_name = "plate".into();
            let delivery = Delivery {
                setup: &s,
                colours: colours(&colour)?,
                object_name,
                gcode: gcode.as_deref(),
                out: &out,
            };
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
        Command::View { input, out, quality, material, layers, tune, flat, infill } => {
            let paths: layercraft_path::Paths = if input.extension().is_some_and(|e| e == "json") {
                pipeline::read_json(&input)?
            } else {
                let s = setup(&quality, &material, None, tune.as_deref(), Choices { flat, infill })?;
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
