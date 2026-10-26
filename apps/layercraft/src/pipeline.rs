//! The chain of stages, with any stage written out (`--emit`) or read back in (`--from`).
//!
//! A stage is written as JSON, which reads back, or as PLY when the file name ends in `.ply`:
//! a picture of the stage to look at (`layercraft_view`), which does not. A tuning varies the
//! region, path and motion parameters by place (`--tune`). The program stage holds the moves
//! at full speed, without the machine's own G-code: cooling and the frame come after it.

use crate::settings::Settings;
use layercraft_mesh::{Mesh, read_3mf};
use layercraft_motion::Program;
use layercraft_path::Paths;
use layercraft_region::Regions;
use layercraft_section::Sections;
use layercraft_tune::{Field, TuneError, Tuning};
use layercraft_view::{Complex, ViewError};
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::path::{Path, PathBuf};

/// A stage whose output can be written and read back.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stage {
    Mesh,
    Sections,
    Regions,
    Paths,
    Program,
}

impl std::str::FromStr for Stage {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(match s {
            "mesh" => Stage::Mesh,
            "sections" => Stage::Sections,
            "regions" => Stage::Regions,
            "paths" => Stage::Paths,
            "program" => Stage::Program,
            other => {
                return Err(format!("unknown stage `{other}` (mesh, sections, regions, paths, program)"));
            }
        })
    }
}

/// `stage=file`, as given to `--emit` and `--from`.
pub fn stage_file(text: &str) -> Result<(Stage, PathBuf), String> {
    let (stage, file) = text.split_once('=').ok_or_else(|| format!("expected STAGE=FILE, got `{text}`"))?;
    Ok((stage.parse()?, PathBuf::from(file)))
}

/// The stages whose parameters a tuning may vary by place; the layering is the whole plate's.
pub const TUNED: [&str; 3] = ["region", "path", "motion"];

/// The largest tuning file read, in bytes.
const MAX_TUNING_BYTES: u64 = 16 << 20;

/// Reads a tuning: a JSON list of overrides, each `{"within": {"x": [from, to], ...}, "set":
/// {"motion": {...}, ...}}`.
pub fn read_tuning(path: &Path) -> Result<Tuning, String> {
    let len = std::fs::metadata(path).map_err(|e| format!("{}: {e}", path.display()))?.len();
    if len > MAX_TUNING_BYTES {
        return Err(format!("{} is larger than {MAX_TUNING_BYTES} bytes", path.display()));
    }
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let tuning: Tuning = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    tuning.check().map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(tuning)
}

/// What the chain produced.
pub struct Output {
    pub paths: Option<Paths>,
    pub program: Program,
}

/// The largest stage file read back, in bytes.
const MAX_STAGE_BYTES: u64 = 4 << 30;

fn read_json<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    if is_ply(path) {
        return Err(format!("{} is a picture of a stage: resume from its JSON", path.display()));
    }
    let len = std::fs::metadata(path).map_err(|e| format!("{}: {e}", path.display()))?.len();
    if len > MAX_STAGE_BYTES {
        return Err(format!("{} is larger than {MAX_STAGE_BYTES} bytes", path.display()));
    }
    let file = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_reader(std::io::BufReader::new(file)).map_err(|e| format!("{}: {e}", path.display()))
}

/// A program from a file: a program stage (JSON), or the moves of G-code from any slicer.
pub fn read_program(path: &Path) -> Result<Program, String> {
    if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("json")) {
        return read_json(path);
    }
    let text = crate::compare::read_gcode(path)?;
    layercraft_gcode::read(&text).map_err(|e| format!("{}: {e}", path.display()))
}

fn is_ply(path: &Path) -> bool {
    path.extension().is_some_and(|e| e.eq_ignore_ascii_case("ply"))
}

/// Writes `stage` to each file `emits` names for it: `value` as JSON, or `view` as PLY.
pub fn emit<T: Serialize, const V: usize, const E: usize, const F: usize>(
    stage: Stage,
    value: &T,
    view: impl Fn() -> Result<Complex<V, E, F>, ViewError>,
    emits: &[(Stage, PathBuf)],
) -> Result<(), String> {
    for (s, path) in emits.iter().filter(|(s, _)| *s == stage) {
        let file = std::io::BufWriter::new(
            std::fs::File::create(path).map_err(|e| format!("{}: {e}", path.display()))?,
        );
        if is_ply(path) {
            view().and_then(|c| c.write(file)).map_err(|e| format!("{s:?}: {e}"))?;
        } else {
            serde_json::to_writer(file, value).map_err(|e| format!("{s:?}: {e}"))?;
        }
        eprintln!("wrote the {s:?} stage to {}", path.display());
    }
    Ok(())
}

/// Reads the 3MF and moves it onto the plate: standing on z = 0, centred on `at`.
pub fn load(input: &Path, at: [f64; 2]) -> Result<Mesh, String> {
    let file = std::fs::File::open(input).map_err(|e| format!("{}: {e}", input.display()))?;
    let mut mesh = read_3mf(std::io::BufReader::new(file)).map_err(|e| e.to_string())?;
    let [lo, hi] = mesh.bounds().ok_or("the 3MF holds no geometry")?;
    mesh.translate([at[0] - (lo[0] + hi[0]) / 2.0, at[1] - (lo[1] + hi[1]) / 2.0, -lo[2]]);
    Ok(mesh)
}

/// Runs the chain from the input (a 3MF, or a stage file given by `from`) to the program.
pub fn run(
    input: Option<&Path>,
    at: [f64; 2],
    settings: &Settings,
    tuning: &Tuning,
    from: Option<&(Stage, PathBuf)>,
    emits: &[(Stage, PathBuf)],
) -> Result<Output, String> {
    tuning.check_stages(&TUNED).map_err(|e| e.to_string())?;
    let field = |e: TuneError| e.to_string();
    let region = Field::new(settings.region.clone(), tuning, "region").map_err(field)?;
    let path = Field::new(settings.path.clone(), tuning, "path").map_err(field)?;
    let motion = Field::new(settings.motion.clone(), tuning, "motion").map_err(field)?;
    let start = from.map(|(s, _)| *s);
    let mesh: Option<Mesh> = match start {
        Some(Stage::Mesh) => from.map(|(_, p)| read_json(p)).transpose()?,
        None => Some(load(input.ok_or("give a 3MF, or a stage with --from")?, at)?),
        _ => None,
    };
    if let Some(mesh) = &mesh {
        emit(Stage::Mesh, mesh, || layercraft_view::mesh(mesh), emits)?;
    }
    let sections: Option<Sections> = match (start, &mesh) {
        (Some(Stage::Sections), _) => from.map(|(_, p)| read_json(p)).transpose()?,
        (_, Some(mesh)) => {
            Some(layercraft_section::section(mesh, &settings.layering).map_err(|e| e.to_string())?)
        }
        _ => None,
    };
    if let Some(s) = &sections {
        if s.open_chains > 0 {
            eprintln!("warning: {} section chains did not close (the mesh has holes)", s.open_chains);
        }
        emit(Stage::Sections, s, || layercraft_view::sections(s), emits)?;
    }
    let regions: Option<Regions> = match (start, &sections) {
        (Some(Stage::Regions), _) => from.map(|(_, p)| read_json(p)).transpose()?,
        (_, Some(s)) => Some(layercraft_region::split(s, &region).map_err(|e| e.to_string())?),
        _ => None,
    };
    if let Some(r) = &regions {
        emit(Stage::Regions, r, || layercraft_view::regions(r), emits)?;
    }
    let paths: Option<Paths> = match (start, &regions) {
        (Some(Stage::Paths), _) => from.map(|(_, p)| read_json(p)).transpose()?,
        (_, Some(r)) => Some(layercraft_path::trace(r, &path).map_err(|e| e.to_string())?),
        _ => None,
    };
    if let Some(p) = &paths {
        emit(Stage::Paths, p, || layercraft_view::paths(p), emits)?;
    }
    let program: Program = match (start, &paths) {
        (Some(Stage::Program), _) => from.map(|(_, p)| read_json(p)).transpose()?.ok_or("no program")?,
        (_, Some(p)) => layercraft_motion::compile(p, &motion).map_err(|e| e.to_string())?,
        _ => return Err("nothing to compile".into()),
    };
    emit(Stage::Program, &program, || layercraft_view::program(&program, &settings.motion.limits), emits)?;
    Ok(Output { paths, program })
}
