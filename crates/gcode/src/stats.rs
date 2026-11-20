//! Reading G-code back: filament and path length per layer and feature, from any slicer's
//! output that marks layers (`; CHANGE_LAYER`, `; Z_HEIGHT:`) and features (`; FEATURE:`).

use crate::read::{Extrusion, word};
use std::collections::BTreeMap;

/// What one layer of a G-code file does.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LayerStats {
    pub z: f64,
    /// Filament pushed per feature, in mm (retractions and their recoveries excluded).
    pub filament: BTreeMap<String, f64>,
    /// Length extruded along per feature, in mm (straight moves only).
    pub length: BTreeMap<String, f64>,
    pub travel: f64,
}

/// What a G-code file does, layer by layer.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Stats {
    pub layers: Vec<LayerStats>,
    /// The slicer's own estimate, from the header, as written.
    pub estimate: Option<String>,
}

/// Reads the statistics of a G-code text; lines it does not understand are skipped.
pub fn stats(text: &str) -> Stats {
    let mut out = Stats::default();
    let mut feature = String::from("(none)");
    let mut at = [0.0f64; 2];
    let mut extrusion = Extrusion::default();
    for raw in text.lines() {
        let line = raw.trim();
        if out.estimate.is_none()
            && let Some((_, rest)) = line.split_once("total estimated time:")
        {
            out.estimate = Some(rest.trim().to_string());
        }
        if line == "; CHANGE_LAYER" {
            out.layers.push(LayerStats::default());
            continue;
        }
        if let Some(z) = line.strip_prefix("; Z_HEIGHT:").and_then(|v| v.trim().parse().ok()) {
            if let Some(layer) = out.layers.last_mut() {
                layer.z = z;
            }
            continue;
        }
        if let Some(name) = line.strip_prefix("; FEATURE:") {
            feature = name.trim().to_string();
            continue;
        }
        let code = line.split(';').next().unwrap_or("").trim();
        let head = code.split_whitespace().next().unwrap_or("");
        if extrusion.follow(head, code) {
            continue;
        }
        match head {
            "G0" | "G1" | "G2" | "G3" => {
                let to = [word(code, 'X').unwrap_or(at[0]), word(code, 'Y').unwrap_or(at[1])];
                let moved = (to[0] - at[0]).hypot(to[1] - at[1]);
                let e = extrusion.pushed(code);
                let has_xy = word(code, 'X').is_some() || word(code, 'Y').is_some();
                if let Some(layer) = out.layers.last_mut() {
                    if e > 0.0 && has_xy {
                        *layer.filament.entry(feature.clone()).or_default() += e;
                        if code.starts_with("G1") || code.starts_with("G0") {
                            *layer.length.entry(feature.clone()).or_default() += moved;
                        }
                    } else if e == 0.0 && has_xy {
                        layer.travel += moved;
                    }
                }
                at = to;
            }
            _ => {}
        }
    }
    out
}
