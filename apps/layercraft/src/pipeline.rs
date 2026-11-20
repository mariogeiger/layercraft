//! The chain of stages, with any stage written out (`--emit`) or read back in (`--from`).
//!
//! A stage is written as JSON, which reads back, or as PLY when the file name ends in `.ply`:
//! a picture of the stage to look at (`layercraft_view`), which does not. A tuning varies the
//! region, path and motion parameters by place (`--tune`). The program stage holds the moves
//! at full speed, without the machine's own G-code: cooling and the frame come after it. A
//! program is also read from the G-code of a `.gcode` file or of a `.gcode.3mf`'s plate.

use crate::deliver::Setup;
use layercraft_mesh::{Mesh, read_3mf};
use layercraft_motion::Program;
use layercraft_path::Paths;
use layercraft_region::Regions;
use layercraft_section::Sections;
use layercraft_tune::{Field, TuneError, Tuning};
use layercraft_view::{Complex, ViewError};
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::io::Read;
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
    /// The materials of the mesh, when the chain began with one.
    pub materials: Vec<layercraft_mesh::Material>,
}

/// The largest stage file read back, in bytes.
const MAX_STAGE_BYTES: u64 = 4 << 30;

/// The largest G-code read, in bytes.
const MAX_GCODE_BYTES: u64 = 2 << 30;

/// Whether `path`'s name ends in `.ext`, in any case.
pub fn has_extension(path: &Path, ext: &str) -> bool {
    path.extension().is_some_and(|e| e.eq_ignore_ascii_case(ext))
}

/// The G-code of a `.gcode` file, or of the first plate of a `.gcode.3mf`.
pub fn read_gcode(path: &Path) -> Result<String, String> {
    let name = path.display();
    let file = std::fs::File::open(path).map_err(|e| format!("{name}: {e}"))?;
    let mut text = String::new();
    if has_extension(path, "3mf") {
        let mut zip =
            zip::ZipArchive::new(std::io::BufReader::new(file)).map_err(|e| format!("{name}: {e}"))?;
        let entry = zip.by_name("Metadata/plate_1.gcode").map_err(|e| format!("{name}: {e}"))?;
        entry.take(MAX_GCODE_BYTES).read_to_string(&mut text).map_err(|e| format!("{name}: {e}"))?;
    } else {
        file.take(MAX_GCODE_BYTES).read_to_string(&mut text).map_err(|e| format!("{name}: {e}"))?;
    }
    Ok(text)
}

pub fn read_json<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    if has_extension(path, "ply") {
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
    if has_extension(path, "json") {
        return read_json(path);
    }
    let text = read_gcode(path)?;
    layercraft_gcode::read(&text).map_err(|e| format!("{}: {e}", path.display()))
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
        if has_extension(path, "ply") {
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
    setup: &Setup,
    from: Option<&(Stage, PathBuf)>,
    emits: &[(Stage, PathBuf)],
) -> Result<Output, String> {
    let (settings, tuning) = (&setup.settings, &setup.tuning);
    tuning.check_stages(&TUNED).map_err(|e| e.to_string())?;
    let tune_error = |e: TuneError| e.to_string();
    let region = Field::new(settings.region.clone(), tuning, "region").map_err(tune_error)?;
    let path = Field::new(settings.path.clone(), tuning, "path").map_err(tune_error)?;
    // Each filament moves by its own: the first's, then those after it.
    let motion = std::iter::once(settings)
        .chain(&setup.more)
        .map(|s| Field::new(s.motion.clone(), tuning, "motion").map_err(tune_error))
        .collect::<Result<Vec<_>, String>>()?;
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
            eprintln!(
                "warning: {} section chains stopped at cracks in the mesh, up to {:.3} mm wide; each was joined to the nearest chain",
                s.open_chains, s.widest_crack
            );
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
    let materials = mesh.map(|m| m.materials).unwrap_or_default();
    Ok(Output { paths, program, materials })
}
