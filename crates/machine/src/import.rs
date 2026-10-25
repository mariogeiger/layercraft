//! Importing a machine description from an installed Bambu Studio's profiles (read as data).
//!
//! A profile is a JSON object that may `inherits` another profile of its kind and `include`
//! template files; the flat profile is the inherited chain applied from the root down, each
//! profile's includes before its own settings.

use crate::{Machine, MachineError, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Where Bambu Studio keeps its Bambu Lab profiles on macOS.
pub const BAMBU_STUDIO_PROFILES: &str = "/Applications/BambuStudio.app/Contents/Resources/profiles/BBL";
/// The longest inheritance chain followed.
const MAX_CHAIN: usize = 32;
/// The largest profile file read, in bytes.
const MAX_PROFILE_BYTES: u64 = 16 << 20;

/// The profiles to import.
#[derive(Debug, Clone)]
pub struct Selection {
    pub machine: String,
    pub process: String,
    pub filament: String,
    pub variant: String,
}

fn failure(e: impl std::fmt::Display) -> MachineError {
    MachineError::Import(e.to_string())
}

fn read_object(path: &Path) -> Result<serde_json::Map<String, serde_json::Value>, MachineError> {
    let len = std::fs::metadata(path).map_err(|e| failure(format!("{}: {e}", path.display())))?.len();
    if len > MAX_PROFILE_BYTES {
        return Err(failure(format!("{} is larger than {MAX_PROFILE_BYTES} bytes", path.display())));
    }
    let text = std::fs::read_to_string(path).map_err(|e| failure(format!("{}: {e}", path.display())))?;
    match serde_json::from_str(&text).map_err(|e| failure(format!("{}: {e}", path.display())))? {
        serde_json::Value::Object(map) => Ok(map),
        _ => Err(failure(format!("{} is not a JSON object", path.display()))),
    }
}

/// The flat settings of one profile, with its inherited chain and includes resolved.
pub fn flatten(dir: &Path, name: &str) -> Result<BTreeMap<String, Value>, MachineError> {
    let file = |n: &str| -> PathBuf { dir.join(format!("{n}.json")) };
    let mut chain = Vec::new();
    let mut next = Some(name.to_string());
    while let Some(n) = next {
        if chain.len() >= MAX_CHAIN {
            return Err(failure(format!("the profile {name} inherits more than {MAX_CHAIN} levels")));
        }
        let object = read_object(&file(&n))?;
        next = object.get("inherits").and_then(|v| v.as_str()).filter(|s| !s.is_empty()).map(str::to_string);
        chain.push(object);
    }
    let mut flat = BTreeMap::new();
    for object in chain.iter().rev() {
        if let Some(serde_json::Value::Array(includes)) = object.get("include") {
            for include in includes.iter().filter_map(|v| v.as_str()) {
                for (k, v) in read_object(&file(include))? {
                    flat.insert(k, Value::from_json(&v));
                }
            }
        }
        for (k, v) in object {
            flat.insert(k.clone(), Value::from_json(v));
        }
    }
    Ok(flat)
}

/// Builds a machine description from a profile folder holding `machine/`, `process/` and
/// `filament/`.
pub fn import(dir: &Path, selection: &Selection) -> Result<Machine, MachineError> {
    let version = std::fs::read_to_string(dir.join("../../../Info.plist")).ok().and_then(|plist| {
        let after = plist.split("CFBundleShortVersionString").nth(1)?;
        Some(after.split("<string>").nth(1)?.split('<').next()?.to_string())
    });
    let mut machine = flatten(&dir.join("machine"), &selection.machine)?;
    // The printer model's own file (not in the inheritance chain) names its model code and plate.
    if let Some(model) = machine.get("printer_model").map(Value::render) {
        let path = dir.join("machine").join(format!("{model}.json"));
        if let Ok(object) = read_object(&path) {
            for key in ["model_id", "default_bed_type"] {
                if let Some(v) = object.get(key) {
                    machine.insert(key.to_string(), Value::from_json(v));
                }
            }
        }
    }
    Ok(Machine {
        source: format!(
            "Bambu Studio {} profiles: {} / {} / {}",
            version.as_deref().unwrap_or("(unknown version)"),
            selection.machine,
            selection.process,
            selection.filament
        ),
        variant: selection.variant.clone(),
        slicer_version: version.clone().unwrap_or_default(),
        machine,
        process: flatten(&dir.join("process"), &selection.process)?,
        filament: flatten(&dir.join("filament"), &selection.filament)?,
    })
}
