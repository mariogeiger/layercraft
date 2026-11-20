//! From a program to the file the printer runs: its beads written as the fewest lines and arcs
//! within the resolution, each layer cooled, each move held to what the hot end melts of its
//! filament, the machine's own G-code around the passes, every move verified, then written as
//! G-code and packaged with the plate's summary and thumbnails. Compiled, read or merged, every
//! program leaves this way.

use crate::settings::{Filaments, Settings};
use layercraft_motion::{Filament, Program, disc_area};
use layercraft_tune::{Field, Tuning};
use std::path::Path;

/// The stages' settings for the first filament and for those after it (`more`, one per
/// material, the last standing for those past them), and the tuning given.
pub struct Setup {
    pub settings: Settings,
    pub more: Vec<Settings>,
    pub tuning: Tuning,
}

impl Setup {
    /// The filaments, by material.
    pub fn filaments(&self) -> Filaments<'_> {
        Filaments { first: &self.settings, more: &self.more }
    }
}

/// What goes with a program into its file.
pub struct Delivery<'a> {
    pub setup: &'a Setup,
    /// Each material's filament colour, `#RRGGBB`, by material.
    pub colours: Vec<String>,
    pub object_name: String,
    /// Also write the G-code as plain text here.
    pub gcode: Option<&'a Path>,
    pub out: &'a Path,
}

/// Fits, cools, holds to the machine's limits, frames, verifies, writes and packages `program`;
/// a line saying what was written.
pub fn deliver(mut program: Program, d: &Delivery) -> Result<String, String> {
    let settings = &d.setup.settings;
    let motion = &settings.motion;
    let materials = program.materials();
    if d.colours.len() < materials {
        return Err(format!("the program prints {materials} materials; give a colour for each"));
    }
    let colours = d.colours.get(..materials).unwrap_or_default();
    let filaments = d.setup.filaments();
    let scope = filaments.scope(colours);
    let fields = (0..materials)
        .map(|k| Field::new(filaments.of(k).motion.clone(), &d.setup.tuning, "motion"))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| e.to_string())?;
    // Each material's filament, which its passes' moves push and are held to.
    let fed: Vec<Filament> = (0..materials).map(|k| filaments.of(k).motion.filament()).collect();
    layercraft_motion::fit(&mut program, motion.resolution, motion.arc_fitting);
    layercraft_motion::cool(&mut program, &fields, settings.envelope.area).map_err(|e| e.to_string())?;
    // Held to the machine's limits last: slowing only lengthens the layers cooled.
    let slowed = layercraft_motion::limit(&mut program, &fed, motion.limits.max_speed_z);
    let times = layercraft_motion::estimate(&program, &motion.limits);
    let summary = layercraft_motion::summarize(&program, &times, &fed);
    let heat = filaments.heat(materials);
    let frame =
        layercraft_motion::frame(&program, &motion.templates, &heat, &scope).map_err(|e| e.to_string())?;
    let violations = layercraft_verify::verify(&program, &settings.envelope, &fed);
    if let Some(first) = violations.first() {
        for v in &violations {
            eprintln!("unsafe: layer {} (pass {}) move {}: {}", v.layer, v.pass, v.index, v.reason);
        }
        return Err(format!(
            "{} unsafe moves, the first: {}; nothing written",
            violations.len(),
            first.reason
        ));
    }
    let mut keys: Vec<(String, String)> = scope.iter().map(|(k, v)| (k.clone(), v.render())).collect();
    keys.sort();
    let header = layercraft_gcode::Header {
        generator: format!("layercraft {}", env!("CARGO_PKG_VERSION")),
        filament_diameters: fed.iter().map(|f| f.diameter).collect(),
        filament_densities: (0..materials).map(|k| filaments.of(k).material.density).collect(),
        settings: keys,
    };
    let text = layercraft_gcode::write(&program, &frame, &summary, &times, &header);
    if let Some(path) = d.gcode {
        std::fs::write(path, text.as_bytes()).map_err(|e| format!("{}: {e}", path.display()))?;
    }
    let grams_of = |k: usize, mm: f64| {
        let area = fed.get(k).map_or(0.0, |f| disc_area(f.diameter));
        mm * area * filaments.of(k).material.density / 1000.0
    };
    let grams: f64 = summary.filament_mm_by_material.iter().enumerate().map(|(k, &mm)| grams_of(k, mm)).sum();
    let settings_json: serde_json::Map<String, serde_json::Value> = scope
        .iter()
        .map(|(k, v)| {
            let json = match v {
                layercraft_template::Value::List(items) => {
                    serde_json::Value::Array(items.iter().map(|i| i.render().into()).collect())
                }
                other => other.render().into(),
            };
            (k.clone(), json)
        })
        .collect();
    let plate = layercraft_package::Plate {
        gcode: text,
        printer_model: settings.printer.model_id.to_string(),
        nozzle_diameter: settings.printer.nozzle_diameter,
        filaments: colours
            .iter()
            .enumerate()
            .map(|(k, colour)| {
                let mm = summary.filament_mm_by_material.get(k).copied().unwrap_or(0.0);
                let own = filaments.of(k);
                layercraft_package::Filament {
                    kind: own.material.kind.to_string(),
                    colour: colour.clone(),
                    code: own.material.code.to_string(),
                    used_m: mm / 1000.0,
                    used_g: grams_of(k, mm),
                }
            })
            .collect(),
        seconds: summary.motion_seconds,
        bed_type: settings.plate.code().to_string(),
        settings_json: serde_json::Value::Object(settings_json).to_string(),
        thumbnail: layercraft_view::thumbnail(&program, 512),
        small_thumbnail: layercraft_view::thumbnail(&program, 128),
        generator: header.generator.clone(),
        client_version: settings.printer.slicer_version.to_string(),
        object_name: d.object_name.clone(),
        layers: summary.layers,
        layer_height: settings.layering.layer_height,
        first_layer_seconds: summary.first_layer_seconds,
        bounds: summary.first_layer_bounds,
    };
    let file = std::fs::File::create(d.out).map_err(|e| format!("{}: {e}", d.out.display()))?;
    layercraft_package::write_package(&plate, std::io::BufWriter::new(file)).map_err(|e| e.to_string())?;
    let melted = if slowed > 0 {
        format!("; {slowed} beads slowed to the hot end's melt limit")
    } else {
        String::new()
    };
    Ok(format!(
        "{}: {} layers, {:.0} mm of filament ({grams:.2} g), {:.1} min of motion (the printer's own start and end not counted), up to z = {:.2} mm{melted}",
        d.out.display(),
        summary.layers,
        summary.filament_mm,
        summary.motion_seconds / 60.0,
        summary.max_z
    ))
}
