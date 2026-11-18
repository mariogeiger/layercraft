//! The printers layercraft knows, each from its folder in `printers/`: its settings, the
//! printer's own, each quality's and each material's, and what they do not tell, measured on the
//! printer itself (the folder says how). The settings are built into layercraft, so every
//! computer prints with the same; until layercraft writes its own G-code, they stand in the
//! maker's profiles, read from an installed Bambu Studio, for the variant printed with. A
//! printer's measurements are found by the model its settings name.

use layercraft_machine::import::{BAMBU_STUDIO_PROFILES, Selection, import};
use layercraft_machine::{Kind, Machine, MachineError, Value};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

/// What layercraft knows of one printer beyond its machine description, in mm and degrees.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Printer {
    /// The model its machine description names (`printer_model`).
    pub model: String,
    pub nozzle: Nozzle,
    /// The room kept under the head's underside, for beads that curl up and a plate not quite
    /// flat.
    pub head_margin: f64,
    /// The radius of the helix a spiral lift turns: the smallest that breaks the string.
    pub spiral_lift_radius: f64,
    /// G-code the printer expects after the filament's start, before the first layer.
    pub prologue: String,
}

/// The nozzle: a flat face whose flank rises to a column, which reaches `protrusion` below the
/// underside of the rest of the head.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Nozzle {
    pub face_diameter: f64,
    /// The flank's angle above the plate.
    pub flank_degrees: f64,
    pub column_diameter: f64,
    pub protrusion: f64,
}

/// Each known printer, as its folder describes it.
const KNOWN: [&str; 1] = [include_str!("../../../printers/bambu-lab-x2d/printer.json")];

/// A printer's settings as its folder holds them, each a JSON object of settings: the
/// printer's, then by name each quality's and each material's, and the maker's profiles they
/// stand in.
struct Folder {
    maker: &'static str,
    printer: &'static str,
    qualities: &'static [(&'static str, &'static str)],
    materials: &'static [(&'static str, &'static str)],
}

/// The Bambu Lab X2D's settings.
const X2D: Folder = Folder {
    maker: include_str!("../../../printers/bambu-lab-x2d/bambu-studio.json"),
    printer: include_str!("../../../printers/bambu-lab-x2d/machine.json"),
    qualities: &[
        ("0.20", include_str!("../../../printers/bambu-lab-x2d/quality/0.20.json")),
        ("0.12", include_str!("../../../printers/bambu-lab-x2d/quality/0.12.json")),
    ],
    materials: &[
        ("pla", include_str!("../../../printers/bambu-lab-x2d/material/pla.json")),
        ("petg-tr", include_str!("../../../printers/bambu-lab-x2d/material/petg-tr.json")),
    ],
};

/// The maker's profiles a printer's settings stand in, as an installed Bambu Studio names them:
/// the printer's and variant's, and each quality's and material's by our name.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Maker {
    printer: String,
    variant: String,
    quality: BTreeMap<String, String>,
    material: BTreeMap<String, String>,
}

fn unreadable(what: &str, e: impl std::fmt::Display) -> MachineError {
    MachineError::Import(format!("printers/: {what}: {e}"))
}

/// The entry named `name` among `entries`, or why there is none.
fn named<'a>(entries: &[(&str, &'a str)], name: &str, what: &str) -> Result<&'a str, MachineError> {
    entries.iter().find(|(n, _)| *n == name).map(|(_, text)| *text).ok_or_else(|| {
        let known: Vec<&str> = entries.iter().map(|(n, _)| *n).collect();
        MachineError::Unsupported(format!("the {what} `{name}`: the printer has {}", known.join(", ")))
    })
}

/// The X2D printing `quality` in `material`: the maker's profiles from the installed Bambu Studio,
/// each of layercraft's settings given for the variant printed with.
pub fn machine(quality: &str, material: &str) -> Result<Machine, MachineError> {
    let quality_text = named(X2D.qualities, quality, "quality")?;
    let material_text = named(X2D.materials, material, "material")?;
    let maker: Maker = serde_json::from_str(X2D.maker).map_err(|e| unreadable("bambu-studio.json", e))?;
    let profile = |names: &BTreeMap<String, String>, name: &str| {
        names
            .get(name)
            .cloned()
            .ok_or_else(|| unreadable("bambu-studio.json", format!("no profile for `{name}`")))
    };
    let selection = Selection {
        machine: maker.printer.clone(),
        process: profile(&maker.quality, quality)?,
        filament: profile(&maker.material, material)?,
        variant: maker.variant.clone(),
    };
    let mut machine = import(Path::new(BAMBU_STUDIO_PROFILES), &selection)?;
    for (kind, file, text) in [
        (Kind::Machine, "machine.json", X2D.printer),
        (Kind::Process, quality, quality_text),
        (Kind::Filament, material, material_text),
    ] {
        let settings: serde_json::Map<String, serde_json::Value> =
            serde_json::from_str(text).map_err(|e| unreadable(file, e))?;
        for (key, value) in &settings {
            machine.set(kind, key, Value::from_json(value));
        }
    }
    Ok(machine)
}

/// The printer whose machine description names the model `model`.
pub fn printer(model: &str) -> Result<Printer, MachineError> {
    for text in KNOWN {
        let p: Printer = serde_json::from_str(text).map_err(|e| MachineError::Import(e.to_string()))?;
        if p.model == model {
            return check(p);
        }
    }
    Err(MachineError::Unsupported(format!("the printer `{model}`, which has no folder in printers/")))
}

/// The printer, if its measurements make a nozzle: finite, the face no wider than the column, the
/// flank rising, the margin within the nozzle's reach.
fn check(p: Printer) -> Result<Printer, MachineError> {
    let n = &p.nozzle;
    let sizes = [n.face_diameter, n.column_diameter, n.protrusion, p.head_margin, p.spiral_lift_radius];
    let sound = sizes.iter().all(|v| v.is_finite() && *v >= 0.0)
        && n.face_diameter <= n.column_diameter
        && n.flank_degrees > 0.0
        && n.flank_degrees <= 90.0
        && p.head_margin <= n.protrusion;
    if sound {
        Ok(p)
    } else {
        Err(MachineError::Import(format!("the measurements of the printer `{}` make no nozzle", p.model)))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn every_known_printer_reads_and_makes_a_nozzle() {
        for text in KNOWN {
            let p: Printer = serde_json::from_str(text).unwrap();
            assert_eq!(printer(&p.model).unwrap(), p);
        }
        assert!(printer("Some Other Printer").is_err());
    }

    #[test]
    fn every_quality_and_material_reads_and_names_its_profile() {
        let maker: Maker = serde_json::from_str(X2D.maker).unwrap();
        let object =
            |text: &str| serde_json::from_str::<serde_json::Map<String, serde_json::Value>>(text).unwrap();
        assert!(!object(X2D.printer).is_empty());
        for (entries, profiles) in [(X2D.qualities, &maker.quality), (X2D.materials, &maker.material)] {
            for (name, text) in entries {
                assert!(!object(text).is_empty(), "{name}");
                assert!(profiles.contains_key(*name), "{name} has no profile in bambu-studio.json");
            }
            assert_eq!(entries.len(), profiles.len());
        }
        let refused = |r: Result<Machine, MachineError>| matches!(r, Err(MachineError::Unsupported(_)));
        assert!(refused(machine("0.16", "pla")));
        assert!(refused(machine("0.20", "wood")));
    }
}
