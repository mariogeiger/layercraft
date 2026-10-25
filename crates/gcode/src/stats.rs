//! Reading G-code back: filament and path length per layer and feature, from any slicer's
//! output that marks layers (`; CHANGE_LAYER`, `; Z_HEIGHT:`) and features (`; FEATURE:`).

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

fn word(line: &str, letter: char) -> Option<f64> {
    line.split_whitespace().skip(1).find_map(|w| w.strip_prefix(letter).and_then(|v| v.parse::<f64>().ok()))
}

/// Reads the statistics of a G-code text; lines it does not understand are skipped.
pub fn stats(text: &str) -> Stats {
    let mut out = Stats::default();
    let mut feature = String::from("(none)");
    let mut at = [0.0f64; 2];
    let mut absolute_e = false;
    let mut last_e = 0.0;
    for raw in text.lines() {
        let line = raw.trim();
        if let Some(rest) = line.strip_prefix("; total estimated time:") {
            out.estimate = Some(rest.trim().to_string());
        } else if line.contains("total estimated time:") && out.estimate.is_none() {
            out.estimate = line.split("total estimated time:").nth(1).map(|s| s.trim().to_string());
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
        match code.split_whitespace().next() {
            Some("M82") => absolute_e = true,
            Some("M83") => absolute_e = false,
            Some("G92") => last_e = word(code, 'E').unwrap_or(last_e),
            Some("G0" | "G1" | "G2" | "G3") => {
                let to = [word(code, 'X').unwrap_or(at[0]), word(code, 'Y').unwrap_or(at[1])];
                let moved = (to[0] - at[0]).hypot(to[1] - at[1]);
                let e = match word(code, 'E') {
                    Some(v) if absolute_e => {
                        let d = v - last_e;
                        last_e = v;
                        d
                    }
                    Some(v) => v,
                    None => 0.0,
                };
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
