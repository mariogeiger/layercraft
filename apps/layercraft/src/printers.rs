//! The printers layercraft knows, each from its folder in `printers/`: what a printer's machine
//! description does not tell, measured on the printer itself (the folder says how). A printer is
//! found by the model its machine description names.

use layercraft_machine::MachineError;
use serde::Deserialize;

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
}
