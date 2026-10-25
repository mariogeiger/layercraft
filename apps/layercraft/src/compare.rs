//! Comparing two G-code files, layer by layer and feature by feature: ours against another
//! slicer's, as a reference rather than an oracle.

use layercraft_gcode::stats::{Stats, stats};
use std::collections::BTreeMap;
use std::io::Read;
use std::path::Path;

/// The largest G-code read, in bytes.
const MAX_GCODE_BYTES: u64 = 2 << 30;

/// The G-code of a `.gcode` file, or of the first plate of a `.gcode.3mf`.
pub fn read_gcode(path: &Path) -> Result<String, String> {
    let file = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let name = path.to_string_lossy();
    let mut text = String::new();
    if name.ends_with(".3mf") {
        let mut zip =
            zip::ZipArchive::new(std::io::BufReader::new(file)).map_err(|e| format!("{name}: {e}"))?;
        let entry = zip.by_name("Metadata/plate_1.gcode").map_err(|e| format!("{name}: {e}"))?;
        entry.take(MAX_GCODE_BYTES).read_to_string(&mut text).map_err(|e| format!("{name}: {e}"))?;
    } else {
        file.take(MAX_GCODE_BYTES).read_to_string(&mut text).map_err(|e| format!("{name}: {e}"))?;
    }
    Ok(text)
}

fn totals(s: &Stats) -> (BTreeMap<String, f64>, BTreeMap<String, f64>, f64) {
    let mut filament = BTreeMap::new();
    let mut length = BTreeMap::new();
    let mut travel = 0.0;
    for layer in &s.layers {
        for (k, v) in &layer.filament {
            *filament.entry(k.clone()).or_insert(0.0) += v;
        }
        for (k, v) in &layer.length {
            *length.entry(k.clone()).or_insert(0.0) += v;
        }
        travel += layer.travel;
    }
    (filament, length, travel)
}

/// A table of the two files side by side.
pub fn report(ours: &str, theirs: &str) -> String {
    let (a, b) = (stats(ours), stats(theirs));
    let (fa, la, ta) = totals(&a);
    let (fb, lb, tb) = totals(&b);
    let mut out = String::new();
    let sum = |m: &BTreeMap<String, f64>| m.values().sum::<f64>();
    out.push_str(&format!("{:<28} {:>12} {:>12}\n", "", "ours", "theirs"));
    out.push_str(&format!("{:<28} {:>12} {:>12}\n", "layers", a.layers.len(), b.layers.len()));
    let est = |s: &Stats| s.estimate.clone().unwrap_or_else(|| "-".into());
    out.push_str(&format!("{:<28} {:>12} {:>12}\n", "estimated time", est(&a), est(&b)));
    out.push_str(&format!("{:<28} {:>12.1} {:>12.1}\n", "filament, mm", sum(&fa), sum(&fb)));
    out.push_str(&format!("{:<28} {:>12.0} {:>12.0}\n", "travel, mm", ta, tb));
    out.push_str("\nfilament by feature, mm (straight extruded length, mm)\n");
    let mut keys: Vec<&String> = fa.keys().chain(fb.keys()).collect();
    keys.sort();
    keys.dedup();
    for k in keys {
        let cell = |f: &BTreeMap<String, f64>, l: &BTreeMap<String, f64>| {
            format!("{:.1} ({:.0})", f.get(k).copied().unwrap_or(0.0), l.get(k).copied().unwrap_or(0.0))
        };
        out.push_str(&format!("  {:<26} {:>12} {:>12}\n", k, cell(&fa, &la), cell(&fb, &lb)));
    }
    out.push_str("\nlayers whose filament differs most\n");
    let mut diffs: Vec<(usize, f64, f64, f64)> = a
        .layers
        .iter()
        .zip(&b.layers)
        .enumerate()
        .map(|(i, (x, y))| (i, x.z, sum(&x.filament), sum(&y.filament)))
        .collect();
    diffs.sort_by(|p, q| (q.2 - q.3).abs().total_cmp(&(p.2 - p.3).abs()));
    for (i, z, x, y) in diffs.into_iter().take(6) {
        out.push_str(&format!("  layer {i:>4} z {z:>7.2}: {x:>8.2} vs {y:>8.2} mm\n"));
    }
    out
}
