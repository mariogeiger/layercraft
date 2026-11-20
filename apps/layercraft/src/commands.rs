//! The command line: arguments and what each command does.

use crate::defaults::{self, COLOUR, X2D};
use crate::deliver::{Delivery, Setup, deliver};
use crate::pipeline::{self, Stage, stage_file};
use crate::settings;
use clap::{Parser, Subcommand};
use layercraft_path::SparsePattern;
use std::path::{Path, PathBuf};

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
        #[command(flatten)]
        printing: Printing,
        /// Also write the G-code as plain text.
        #[arg(long)]
        gcode: Option<PathBuf>,
        /// The filament every support column is laid in, numbered from 1 as the materials are
        /// (default: each column in the one the part prints on the most of its layers).
        #[arg(long)]
        support: Option<usize>,
        /// The filament the supports' interface is laid in, numbered from 1 as the materials
        /// are (default: its column's): one that does not weld to the part, as PLA under PETG,
        /// lets the supports come off it. Give it a colour and a material.
        #[arg(long)]
        support_interface: Option<usize>,
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
        #[command(flatten)]
        printing: Printing,
        /// Also write the G-code as plain text.
        #[arg(long)]
        gcode: Option<PathBuf>,
        /// The filaments' colours for the printer's screen, one per material, comma separated.
        #[arg(long, default_value = COLOUR)]
        colour: String,
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
        #[command(flatten)]
        printing: Printing,
        /// Only layers FROM..TO (end excluded).
        #[arg(long)]
        layers: Option<String>,
        /// Keep every layer flat, as for `compile`.
        #[arg(long)]
        flat: bool,
        /// What sparse areas are filled with, as for `compile`.
        #[arg(long, value_enum)]
        infill: Option<Infill>,
    },
}

/// What every command takes that compiles or prints: where to write, the printer's quality and
/// filaments, the plate and the tuning.
#[derive(clap::Args)]
struct Printing {
    #[arg(long)]
    out: PathBuf,
    /// The print quality: 0.20 or 0.12, its layer height in mm.
    #[arg(long, default_value = defaults::QUALITY)]
    quality: String,
    /// The filaments, one per material, comma separated, the last standing for those past
    /// them: pla (Bambu PLA Basic) or petg-tr (Bambu PETG Translucent).
    #[arg(long, default_value = defaults::MATERIAL)]
    material: String,
    /// The plate type, such as "Textured PEI Plate" (default: the printer's).
    #[arg(long)]
    bed: Option<String>,
    /// Vary settings by place: a JSON list of overrides, each `{"within": {"z": [0, 5]},
    /// "set": {"motion": {"lift_radius": 0.6}}}`; boxes are in plate millimetres.
    #[arg(long)]
    tune: Option<PathBuf>,
}

/// What sparse areas are filled with.
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum Infill {
    Lines,
    Rolls,
}

/// How a compile is made: layers kept `flat` or not, the sparse areas filled with `infill` or
/// with the settings' rolls, the supports' columns in the material `column` or each in its own,
/// their interface in the material `interface` or its column's. Supports are no choice: whatever
/// hangs stands on them.
#[derive(Clone, Copy)]
struct Choices {
    flat: bool,
    infill: Option<Infill>,
    column: Option<usize>,
    interface: Option<usize>,
}

/// The printer in `quality` and `materials` (one per material, comma separated) on the plate
/// `bed`, its settings for each filament and the tuning, as `choices` make them.
fn setup(
    quality: &str,
    materials: &str,
    bed: Option<&str>,
    tune: Option<&Path>,
    choices: Choices,
) -> Result<Setup, String> {
    let Choices { flat, infill, column, interface } = choices;
    let tuning = tune.map(pipeline::read_tuning).transpose()?.unwrap_or_default();
    let quality = settings::quality(quality)?;
    let plate = bed.map(settings::plate).transpose()?.unwrap_or(X2D.plate);
    let interface = match interface {
        Some(k) => Some(k.checked_sub(1).ok_or("the supports' interface filament is numbered from 1")?),
        None => None,
    };
    let column = match column {
        Some(k) => Some(k.checked_sub(1).ok_or("the supports' filament is numbered from 1")?),
        None => None,
    };
    let read = |name: &str| -> Result<settings::Settings, String> {
        let mut settings = settings::read(&X2D, quality, settings::material(name.trim())?, plate)?;
        if flat {
            settings.layering.foliation = None;
        }
        match infill {
            Some(Infill::Lines) => settings.path.sparse_pattern = SparsePattern::Lines,
            Some(Infill::Rolls) => {
                settings.path.sparse_pattern = settings::rolls(settings.layering.layer_height)
            }
            None => {}
        }
        settings.region.supports.interface_material = interface;
        settings.region.supports.column_material = column;
        Ok(settings)
    };
    let mut each = materials.split(',').map(read);
    let settings = each.next().ok_or("no material given")??;
    let more = each.collect::<Result<Vec<_>, String>>()?;
    Ok(Setup { settings, more, tuning })
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
            printing: p,
            gcode,
            support,
            support_interface,
            emit,
            from,
            at,
            colour,
            flat,
            infill,
        } => {
            let choices = Choices { flat, infill, column: support, interface: support_interface };
            let s = setup(&p.quality, &p.material, p.bed.as_deref(), p.tune.as_deref(), choices)?;
            let at = match at {
                Some(text) => parse_pair(&text)?,
                None => plate_centre(&s),
            };
            let output = pipeline::run(input.as_deref(), at, &s, from.as_ref(), &emit)?;
            let object_name = input
                .as_ref()
                .and_then(|p| p.file_stem())
                .map(|s| s.to_string_lossy().trim_end_matches(".gcode").to_string())
                .unwrap_or_else(|| "part".into());
            let colours = match colour {
                Some(text) => colours(&text)?,
                None if !output.materials.is_empty() => output
                    .materials
                    .iter()
                    .map(|m| m.colour.clone().unwrap_or_else(|| COLOUR.into()))
                    .collect(),
                None => vec![COLOUR.into()],
            };
            let delivery = Delivery { setup: &s, colours, object_name, gcode: gcode.as_deref(), out: &p.out };
            println!("{}", deliver(output.program, &delivery)?);
            Ok(())
        }
        Command::Merge { inputs, printing: p, gcode, colour, emit } => {
            if let Some((stage, _)) = emit.iter().find(|(s, _)| *s != Stage::Program) {
                return Err(format!("merge writes the program stage only, not {stage:?}"));
            }
            let choices = Choices { flat: false, infill: None, column: None, interface: None };
            let s = setup(&p.quality, &p.material, p.bed.as_deref(), p.tune.as_deref(), choices)?;
            let programs = inputs.iter().map(|p| pipeline::read_program(p)).collect::<Result<Vec<_>, _>>()?;
            let merged = layercraft_motion::merge(&programs, s.settings.motion.deretract_speed)
                .map_err(|e| e.to_string())?;
            let limits = &s.settings.motion.limits;
            pipeline::emit(Stage::Program, &merged, || layercraft_view::program(&merged, limits), &emit)?;
            let delivery = Delivery {
                setup: &s,
                colours: colours(&colour)?,
                object_name: "plate".into(),
                gcode: gcode.as_deref(),
                out: &p.out,
            };
            println!("{}", deliver(merged, &delivery)?);
            Ok(())
        }
        Command::Compare { ours, theirs } => {
            let report =
                crate::compare::report(&pipeline::read_gcode(&ours)?, &pipeline::read_gcode(&theirs)?);
            print!("{report}");
            Ok(())
        }
        Command::View { input, printing: p, layers, flat, infill } => {
            let paths: layercraft_path::Paths = if pipeline::has_extension(&input, "json") {
                pipeline::read_json(&input)?
            } else {
                let choices = Choices { flat, infill, column: None, interface: None };
                let s = setup(&p.quality, &p.material, p.bed.as_deref(), p.tune.as_deref(), choices)?;
                let output = pipeline::run(Some(&input), plate_centre(&s), &s, None, &[])?;
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
            write_file(&p.out, script.to_string().as_bytes())?;
            println!("wrote {}", p.out.display());
            Ok(())
        }
    }
}
