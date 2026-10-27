//! From a program to the file the printer runs: each layer cooled, each bead held to what the
//! hot end melts and written as the fewest lines and arcs within the resolution, the machine's
//! own G-code around the layers, every move verified, then written as G-code and packaged with
//! the plate's summary and thumbnails. Compiled, read or merged, every program leaves this way.

use crate::settings::{Settings, plate_code};
use layercraft_machine::Machine;
use layercraft_motion::Program;
use layercraft_tune::{Field, Tuning};
use std::path::Path;

/// What goes with a program into its file.
pub struct Delivery<'a> {
    pub settings: &'a Settings,
    pub tuning: &'a Tuning,
    pub machine: &'a Machine,
    /// The filament's colour for the printer's screen.
    pub colour: String,
    pub object_name: String,
    /// Also write the G-code as plain text here.
    pub gcode: Option<&'a Path>,
    pub out: &'a Path,
}

/// Cools, frames, verifies, writes and packages `program`; a line saying what was written.
pub fn deliver(mut program: Program, d: &Delivery) -> Result<String, String> {
    let settings = d.settings;
    let motion = &settings.motion;
    let field = Field::new(motion.clone(), d.tuning, "motion").map_err(|e| e.to_string())?;
    layercraft_motion::cool(&mut program, &field).map_err(|e| e.to_string())?;
    let slowed = layercraft_motion::melt(&mut program, motion.max_volumetric_speed, motion.filament_diameter);
    layercraft_motion::fit(&mut program, motion.resolution, motion.arc_fitting);
    let frame =
        layercraft_motion::frame(&program, &motion.templates, &settings.scope).map_err(|e| e.to_string())?;
    let violations = layercraft_verify::verify(&program, &settings.envelope);
    if let Some(first) = violations.first() {
        for v in &violations {
            eprintln!("unsafe: layer {} move {}: {}", v.layer, v.index, v.reason);
        }
        return Err(format!(
            "{} unsafe moves, the first: {}; nothing written",
            violations.len(),
            first.reason
        ));
    }
    let mut keys: Vec<(String, String)> =
        settings.scope.iter().map(|(k, v)| (k.clone(), v.render())).collect();
    keys.sort();
    let header = layercraft_gcode::Header {
        generator: format!("layercraft {}", env!("CARGO_PKG_VERSION")),
        filament_diameter: motion.filament_diameter,
        filament_density: settings.filament_density,
        settings: keys,
    };
    let text = layercraft_gcode::write(&program, &frame, &motion.limits, &header);
    if let Some(path) = d.gcode {
        std::fs::write(path, text.as_bytes()).map_err(|e| format!("{}: {e}", path.display()))?;
    }
    let summary = layercraft_motion::finish(&program, &motion.limits, motion.filament_diameter);
    let filament_area = std::f64::consts::PI * (motion.filament_diameter / 2.0).powi(2);
    let grams = summary.filament_mm * filament_area * settings.filament_density / 1000.0;
    let settings_json: serde_json::Map<String, serde_json::Value> = settings
        .scope
        .iter()
        .map(|(k, v)| {
            let json = match v {
                layercraft_machine::Value::List(items) => {
                    serde_json::Value::Array(items.iter().map(|i| i.render().into()).collect())
                }
                other => other.render().into(),
            };
            (k.clone(), json)
        })
        .collect();
    if d.machine.slicer_version.is_empty() {
        return Err(
            "the machine description predates layercraft 0.1.3: run `layercraft machine import` again".into(),
        );
    }
    let plate = layercraft_package::Plate {
        gcode: text,
        printer_model: settings.printer_model.clone(),
        nozzle_diameter: settings.nozzle_diameter,
        filament: layercraft_package::Filament {
            kind: settings.filament_kind.clone(),
            colour: d.colour.clone(),
            code: settings.filament_code.clone(),
            used_m: summary.filament_mm / 1000.0,
            used_g: grams,
        },
        seconds: summary.motion_seconds,
        bed_type: plate_code(&settings.bed_type).to_string(),
        settings_json: serde_json::Value::Object(settings_json).to_string(),
        thumbnail: layercraft_view::thumbnail(&program, 512),
        small_thumbnail: layercraft_view::thumbnail(&program, 128),
        generator: header.generator.clone(),
        client_version: d.machine.slicer_version.clone(),
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
