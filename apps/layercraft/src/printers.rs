//! The printers layercraft knows, each from its folder in `printers/`: its settings, the
//! printer's own, each quality's and each material's, and what they do not tell, measured on the
//! printer itself (the folder says how), and its own G-code: start, layer change, filament change
//! and end. The settings are built into layercraft, so every computer prints with the same; they
//! still stand in the maker's profiles, read from an installed Bambu Studio, for the variant
//! printed with. A printer's measurements are found by the model its settings name.

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
    /// The printer's start, layercraft's own G-code template (`start.gcode`).
    #[serde(skip)]
    pub start: String,
    /// The printer's layer change, layercraft's own G-code template (`layer-change.gcode`).
    #[serde(skip)]
    pub layer_change: String,
    /// The printer's filament change, layercraft's own G-code template
    /// (`filament-change.gcode`).
    #[serde(skip)]
    pub filament_change: String,
    /// The printer's end, layercraft's own G-code template (`end.gcode`).
    #[serde(skip)]
    pub end: String,
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

/// A printer's folder: its measurements and its own G-code, then its settings, each a JSON
/// object: the printer's, by name each quality's and each material's, and the maker's profiles
/// they stand in.
struct Folder {
    measured: &'static str,
    start: &'static str,
    layer_change: &'static str,
    filament_change: &'static str,
    end: &'static str,
    maker: &'static str,
    printer: &'static str,
    qualities: &'static [(&'static str, &'static str)],
    materials: &'static [(&'static str, &'static str)],
}

/// The Bambu Lab X2D.
const X2D: Folder = Folder {
    measured: include_str!("../../../printers/bambu-lab-x2d/printer.json"),
    start: include_str!("../../../printers/bambu-lab-x2d/start.gcode"),
    layer_change: include_str!("../../../printers/bambu-lab-x2d/layer-change.gcode"),
    filament_change: include_str!("../../../printers/bambu-lab-x2d/filament-change.gcode"),
    end: include_str!("../../../printers/bambu-lab-x2d/end.gcode"),
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

/// Each known printer's folder.
const FOLDERS: [&Folder; 1] = [&X2D];

/// The printer whose machine description names the model `model`.
pub fn printer(model: &str) -> Result<Printer, MachineError> {
    for folder in FOLDERS {
        let mut p: Printer =
            serde_json::from_str(folder.measured).map_err(|e| MachineError::Import(e.to_string()))?;
        if p.model == model {
            p.start = folder.start.to_string();
            p.layer_change = folder.layer_change.to_string();
            p.filament_change = folder.filament_change.to_string();
            p.end = folder.end.to_string();
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
        for folder in FOLDERS {
            let p: Printer = serde_json::from_str(folder.measured).unwrap();
            let read = printer(&p.model).unwrap();
            let own = Printer {
                start: folder.start.to_string(),
                layer_change: folder.layer_change.to_string(),
                filament_change: folder.filament_change.to_string(),
                end: folder.end.to_string(),
                ..p
            };
            assert_eq!(read, own);
            for sequence in [&read.start, &read.layer_change, &read.filament_change, &read.end] {
                assert!(!sequence.trim().is_empty());
            }
        }
    }

    #[test]
    fn the_x2d_end_lowers_the_plate_to_80_mm_or_clear_of_a_taller_part() {
        let end = printer("Bambu Lab X2D").unwrap().end;
        for (top, plate) in
            [(5.0, "G1 Z80 F600"), (69.0, "G1 Z80 F600"), (150.0, "G1 Z160 F600"), (255.0, "G1 Z261 F600")]
        {
            let scope = [("max_layer_z", top), ("printable_height", 261.0)]
                .into_iter()
                .map(|(k, v)| (k.to_string(), Value::Num(v)))
                .collect();
            let text = layercraft_machine::expand(&end, &scope).unwrap();
            assert!(text.lines().any(|l| l == plate), "top {top}: {text}");
        }
        assert!(printer("Some Other Printer").is_err());
    }

    /// The X2D's start for a part `top` mm high on `plate` at `bed` °C, in `filament`.
    fn start(top: f64, plate: &str, bed: f64, filament: &str) -> Vec<String> {
        let num = |v: f64| Value::Num(v);
        let list = |v: Vec<Value>| Value::List(v);
        let scope = [
            ("max_print_z", num(top)),
            ("curr_bed_type", Value::Str(plate.into())),
            ("bed_temperature_initial_layer_single", num(bed)),
            ("filament_type", list(vec![Value::Str(filament.into())])),
            ("initial_filament_id", num(0.0)),
            ("initial_nozzle_id", num(0.0)),
            ("initial_no_support_hotend", num(-1.0)),
            ("filament_map", list(vec![num(1.0)])),
            ("nozzle_diameter_at_nozzle_id", list(vec![num(0.4)])),
            ("nozzle_temperature_initial_layer", list(vec![num(220.0)])),
            ("nozzle_temperature", list(vec![num(220.0)])),
            ("flush_volumetric_speeds", list(vec![num(21.0)])),
            ("flush_temperatures", list(vec![num(240.0)])),
            ("filament_max_volumetric_speed", list(vec![num(21.0)])),
            ("first_layer_print_min", list(vec![num(124.15), num(124.15)])),
            ("first_layer_print_size", list(vec![num(7.7), num(7.7)])),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect();
        let text = layercraft_machine::expand(&printer("Bambu Lab X2D").unwrap().start, &scope).unwrap();
        text.lines()
            .map(|l| l.split(';').next().unwrap_or("").trim().to_string())
            .filter(|c| !c.is_empty())
            .collect()
    }

    /// As the maker's own start gave them, measured by compiling with it.
    #[test]
    fn the_x2d_start_follows_the_part_the_plate_and_the_filament() {
        let has = |commands: &[String], c: &str| commands.iter().any(|x| x == c);
        let pla = start(5.0, "Textured PEI Plate", 55.0, "PLA");
        assert_eq!(pla.len(), 264);
        for c in ["M140 S55", "G29 A1 X124.15 Y124.15 I7.7 J7.7 R", "M620.10 A0 F523.843 H0.4 T240 P220 S1"] {
            assert!(has(&pla, c), "{c}");
        }
        for (c, there) in [("G1 E-3 F1800", true), ("M109 S140 A", true), ("M1015.4 S1 K1 H0.4", true)] {
            assert_eq!(has(&pla, c), there, "{c}");
        }
        // Obstacles under the plate are looked for from a 145 mm part up.
        assert!(!start(144.8, "Textured PEI Plate", 55.0, "PLA").iter().any(|c| c.starts_with("G3811")));
        assert!(has(&start(145.0, "Textured PEI Plate", 55.0, "PLA"), "G3811 Z145"));
        // The Z trim, by the plate and whether it is above 70 °C.
        for (plate, bed, trim) in [
            ("Textured PEI Plate", 70.0, "G29.1 Z0.002"),
            ("Textured PEI Plate", 71.0, "G29.1 Z-0.003"),
            ("Cool Plate", 70.0, "G29.1 Z0.022"),
            ("Cool Plate", 71.0, "G29.1 Z0.017"),
        ] {
            assert!(has(&start(5.0, plate, bed, "PLA"), trim), "{plate} {bed}");
        }
        let tpu = start(5.0, "Textured PEI Plate", 55.0, "TPU");
        assert!(
            !has(&tpu, "G1 E-3 F1800") && has(&tpu, "M1015.3 S1 H0.4") && has(&tpu, "M1015.4 S0 K0 H0.4")
        );
        let pc = start(5.0, "Textured PEI Plate", 55.0, "PC");
        assert!(has(&pc, "M109 S170 A") && has(&pc, "M1015.4 S0 K0 H0.4"));
        assert!(has(&start(5.0, "Textured PEI Plate", 55.0, "PETG-CF"), "M1015.4 S1 K1 H0.4"));
    }

    /// The X2D's filament change from PETG to PETG at layer `layer` (from 0), top `z`, with the
    /// settings `with` over the support's (Bambu PETG Translucent, 191.006 mm flushed).
    fn change(layer: f64, z: f64, with: &[(&str, Value)]) -> Vec<String> {
        let num = |v: f64| Value::Num(v);
        let pair = |v: f64| Value::List(vec![num(v), num(v)]);
        let mut scope: layercraft_machine::Scope = [
            ("current_filament_id", num(0.0)),
            ("next_filament_id", num(1.0)),
            ("current_nozzle_id", num(0.0)),
            ("next_nozzle_id", num(0.0)),
            ("current_hotend", num(-1.0)),
            ("next_hotend", num(-1.0)),
            ("flush_length", num(191.006)),
            ("old_filament_temp", num(245.0)),
            ("new_filament_temp", num(245.0)),
            ("long_retraction_when_cut", Value::Bool(false)),
            ("retraction_distance_when_cut", num(10.0)),
            ("long_retraction_when_ec", Value::Bool(true)),
            ("retraction_distance_when_ec", num(3.0)),
            ("max_layer_z", num(z)),
            ("layer_z", num(z)),
            ("layer_num", num(layer)),
            ("flush_volumetric_speeds", pair(6.0)),
            ("flush_temperatures", pair(270.0)),
            ("nozzle_diameter_at_nozzle_id", Value::List(vec![num(0.4)])),
            ("filament_cooling_before_tower", pair(10.0)),
            ("filament_type", Value::List(vec![Value::Str("PETG".into()), Value::Str("PETG".into())])),
            ("retract_length_toolchange", pair(2.0)),
            ("filament_map", pair(1.0)),
            ("filament_max_volumetric_speed", pair(6.0)),
            ("initial_layer_print_height", num(0.2)),
            ("initial_layer_acceleration", pair(500.0)),
            ("travel_acceleration", pair(10000.0)),
            ("close_additional_fan_first_x_layers", pair(3.0)),
            ("additional_fan_full_speed_layer", pair(0.0)),
            ("first_x_layer_fan_speed", pair(40.0)),
            ("additional_cooling_fan_speed", pair(10.0)),
            ("min_vitrification_temperature", num(70.0)),
            ("max_additional_fan", num(10.0)),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect();
        scope.extend(with.iter().map(|(k, v)| (k.to_string(), v.clone())));
        let text =
            layercraft_machine::expand(&printer("Bambu Lab X2D").unwrap().filament_change, &scope).unwrap();
        text.lines()
            .map(|l| l.split(';').next().unwrap_or("").trim().to_string())
            .filter(|c| !c.is_empty())
            .collect()
    }

    /// As the maker's own change gave them, measured by compiling two-colour plates with it.
    #[test]
    fn the_x2d_filament_change_flushes_loads_and_sets_the_new_filament() {
        let has = |commands: &[String], c: &str| commands.iter().any(|x| x == c);
        // The support's change, at its layer 26.
        let support = change(25.0, 5.2, &[]);
        for c in [
            "M620 S1A B H-1",
            "G1 Z8.2 F1200",
            "M620.10 A0 F149.669 L191.006 H0.4 T270 P245 S1",
            "M620.10 A1 F149.669 L191.006 H0.4 T270 P245 S1",
            "M620.15 C235",
            "M620.11 P0 L0 I0 B-1 E0",
            "M620.11 K1 I0 B-1 R3 F200",
            "T1 H-1",
            "SYNC T10",
            "M983.3 F2.5 A0.4 R2",
            "M1015.4 S1 K1 H0.4",
            "M204 S10000",
            "M142 P1 R50 S55 U0.1 V0.5 O55",
        ] {
            assert!(has(&support, c), "{c}: {support:?}");
        }
        // Pulled back far when cut, or not as it leaves the extruder, if the settings say so.
        let cut = change(25.0, 5.2, &[("long_retraction_when_cut", Value::Bool(true))]);
        assert!(has(&cut, "M620.11 P1 L0 I0 B-1 E-10 F200"));
        let ec = change(25.0, 5.2, &[("long_retraction_when_ec", Value::Bool(false))]);
        assert!(has(&ec, "M620.11 K0 I0 B-1 R0"));
        // On the first layer, its acceleration and the auxiliary fans at the first layers' speed.
        let first = change(0.0, 0.2, &[]);
        assert!(has(&first, "M204 S500") && has(&first, "M106 P2 S102") && !has(&first, "M204 S10000"));
        // Climbing to the filament's speed by its full-speed layer, then the chamber's cooling.
        let ramp = [
            ("close_additional_fan_first_x_layers", Value::Num(1.0)),
            ("additional_fan_full_speed_layer", Value::Num(6.0)),
            ("additional_cooling_fan_speed", Value::Num(70.0)),
            ("min_vitrification_temperature", Value::Num(45.0)),
            ("max_additional_fan", Value::Num(70.0)),
        ];
        assert!(has(&change(2.0, 0.6, &ramp), "M106 P10 S132.6"));
        assert!(has(&change(5.0, 1.2, &ramp), "M142 P1 R30 S40 U0.7 V1.0 O45"));
    }

    /// The commands the X2D's layer change gives layer `layer` (from 0) of a filament that holds
    /// its auxiliary fans at `s0` % for `n` layers and reaches `s1` % at layer `f`, softening at
    /// `soft` °C, without the counter's two.
    fn layer_change(layer: f64, [n, f, s0, s1, soft]: [f64; 5]) -> Vec<String> {
        let scope = [
            ("layer_num", layer),
            ("current_filament_id", 0.0),
            ("close_additional_fan_first_x_layers", n),
            ("additional_fan_full_speed_layer", f),
            ("first_x_layer_fan_speed", s0),
            ("additional_cooling_fan_speed", s1),
            ("max_additional_fan", s1),
            ("min_vitrification_temperature", soft),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), Value::Num(v)))
        .collect();
        let text =
            layercraft_machine::expand(&printer("Bambu Lab X2D").unwrap().layer_change, &scope).unwrap();
        let commands: Vec<String> =
            text.lines().map(|l| l.split(';').next().unwrap_or("").trim().to_string()).collect();
        assert_eq!(commands.iter().filter(|c| c.is_empty()).count(), 0, "{text}");
        let counter = [format!("M73 L{}", layer + 1.0), format!("M991 S0 P{layer}")];
        let (body, last) = commands.split_at_checked(commands.len().saturating_sub(2)).unwrap();
        assert_eq!(last, counter, "{text}");
        body.to_vec()
    }

    /// As the maker's own layer change gave them, measured by compiling with it.
    #[test]
    fn the_x2d_layer_change_holds_the_auxiliary_fans_then_hands_them_to_the_chamber() {
        let fans = |s: &str| vec![format!("M106 P2 S{s}"), format!("M106 P10 S{s}")];
        let pla = vec!["M142 P1 R30 S40 U0.7 V1.0 O45".to_string()];
        for (filament, by_layer) in [
            ([1.0, 0.0, 40.0, 70.0, 45.0], vec![fans("102"), pla.clone(), vec![]]),
            (
                [1.0, 5.0, 40.0, 70.0, 45.0],
                vec![fans("102"), fans("121.125"), fans("140.25"), fans("159.375"), pla.clone(), vec![]],
            ),
            (
                [2.0, 6.0, 40.0, 70.0, 45.0],
                vec![fans("102"), fans("102"), fans("121.125"), fans("140.25"), fans("159.375"), pla.clone()],
            ),
            (
                [0.0, 4.0, 40.0, 70.0, 45.0],
                vec![fans("121.125"), fans("140.25"), fans("159.375"), pla.clone()],
            ),
            ([3.0, 3.0, 40.0, 70.0, 45.0], vec![fans("102"), fans("102"), fans("102"), pla.clone(), vec![]]),
            ([1.0, 0.0, 60.0, 70.0, 50.0], vec![fans("153"), pla.clone()]),
            (
                [3.0, 0.0, 40.0, 10.0, 70.0],
                vec![
                    fans("102"),
                    fans("102"),
                    fans("102"),
                    vec!["M142 P1 R50 S55 U0.1 V0.5 O55".to_string()],
                ],
            ),
            ([0.0, 0.0, 40.0, 70.0, 51.0], vec![vec!["M142 P1 R50 S55 U0.7 V0.5 O55".to_string()], vec![]]),
        ] {
            for (layer, commands) in by_layer.iter().enumerate() {
                assert_eq!(&layer_change(layer as f64, filament), commands, "{filament:?} layer {layer}");
            }
        }
    }

    #[test]
    fn a_print_of_petg_and_pla_gives_each_filament_its_own_settings() {
        // PETG for the part, PLA for the supports' interface, the last standing for the third:
        // each its temperatures and its own item in the templates' per-filament lists; no
        // change flushes.
        let read = |name| crate::settings::read(&machine("0.20", name).unwrap(), None).unwrap();
        let (petg, more) = (read("petg-tr"), [read("pla")]);
        let filaments = crate::settings::Filaments { first: &petg, more: &more };
        let heat = filaments.heat(3);
        assert_eq!(heat.nozzle, [[250.0, 245.0], [220.0, 220.0], [220.0, 220.0]]);
        let grey = "#808080".to_string();
        let scope = filaments.scope(&[grey.clone(), grey.clone()]);
        let list = |items: &[Value]| Some(Value::List(items.to_vec()));
        let kinds = [Value::Str("PETG".into()), Value::Str("PLA".into())];
        assert_eq!(scope.get("filament_type").cloned(), list(&kinds));
        assert_eq!(scope.get("nozzle_temperature").cloned(), list(&[Value::Num(245.0), Value::Num(220.0)]));
        let flush = filaments.flush(&[grey.clone(), grey.clone(), grey]);
        let at = |a: usize, b: usize| flush.get(a).and_then(|row| row.get(b)).copied().unwrap_or(-1.0);
        assert!((0..3).all(|a| (0..3).all(|b| at(a, b) == 0.0)), "{flush:?}");
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
