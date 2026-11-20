//! The X2D's defaults hold together, and its own G-code templates give, for each part, plate, filament
//! and layer, the commands the maker's gave on compiles measured one setting at a time.
#![allow(clippy::unwrap_used)]
use super::*;
use layercraft_template::Value;

/// The X2D's measurements make a nozzle: finite, the face no wider than the column, the flank
/// rising, the margin kept under the head within the nozzle's reach; and its G-code is there.
#[test]
fn the_x2d_makes_a_nozzle_and_has_its_own_gcode() {
    let n = &X2D.nozzle;
    let sizes = [n.face_diameter, n.column_diameter, n.protrusion, X2D.head_margin, X2D.spiral_lift_radius];
    assert!(sizes.iter().all(|v| v.is_finite() && *v >= 0.0));
    assert!(n.face_diameter <= n.column_diameter);
    assert!(n.flank_degrees > 0.0 && n.flank_degrees <= 90.0);
    assert!(X2D.head_margin <= n.protrusion);
    let g = &X2D.gcode;
    for sequence in [g.start, g.prologue, g.layer_change, g.filament_change, g.end] {
        assert!(!sequence.trim().is_empty());
    }
}

/// Each quality and material keeps its own name, and every material's temperatures are ones a
/// nozzle and a bed can be asked for.
#[test]
fn every_quality_and_material_is_named_once_and_heats_soundly() {
    for names in
        [QUALITIES.iter().map(|(n, _)| *n).collect::<Vec<_>>(), MATERIALS.iter().map(|(n, _)| *n).collect()]
    {
        assert!(names.iter().enumerate().all(|(i, n)| !names.iter().skip(i + 1).any(|m| m == n)));
    }
    assert!(QUALITIES.iter().any(|(n, _)| *n == QUALITY) && MATERIALS.iter().any(|(n, _)| *n == MATERIAL));
    for (name, m) in &MATERIALS {
        let hot = m.nozzle.iter().chain([&m.flush_temperature]).all(|t| (150.0..=300.0).contains(t));
        let bed = m.beds.iter().all(|(_, t)| (0.0..=110.0).contains(t)) && !m.beds.is_empty();
        assert!(hot && bed && m.nozzle.iter().all(|t| *t <= m.flush_temperature), "{name}");
    }
}

#[test]
fn the_x2d_end_lowers_the_plate_to_80_mm_or_clear_of_a_taller_part() {
    let end = X2D.gcode.end;
    for (top, plate) in
        [(5.0, "G1 Z80 F600"), (69.0, "G1 Z80 F600"), (150.0, "G1 Z160 F600"), (255.0, "G1 Z261 F600")]
    {
        let scope = [("max_layer_z", top), ("printable_height", 261.0)]
            .into_iter()
            .map(|(k, v)| (k.to_string(), Value::Num(v)))
            .collect();
        let text = layercraft_template::expand(end, &scope).unwrap();
        assert!(text.lines().any(|l| l == plate), "top {top}: {text}");
    }
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
    let text = layercraft_template::expand(X2D.gcode.start, &scope).unwrap();
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
    assert!(!has(&tpu, "G1 E-3 F1800") && has(&tpu, "M1015.3 S1 H0.4") && has(&tpu, "M1015.4 S0 K0 H0.4"));
    let pc = start(5.0, "Textured PEI Plate", 55.0, "PC");
    assert!(has(&pc, "M109 S170 A") && has(&pc, "M1015.4 S0 K0 H0.4"));
    assert!(has(&start(5.0, "Textured PEI Plate", 55.0, "PETG-CF"), "M1015.4 S1 K1 H0.4"));
}

/// The X2D's filament change from PETG to PETG at layer `layer` (from 0), top `z`, with the
/// settings `with` over the support's (Bambu PETG Translucent, 191.006 mm flushed).
fn change(layer: f64, z: f64, with: &[(&str, Value)]) -> Vec<String> {
    let num = |v: f64| Value::Num(v);
    let pair = |v: f64| Value::List(vec![num(v), num(v)]);
    let mut scope: layercraft_template::Scope = [
        ("current_filament_id", num(0.0)),
        ("next_filament_id", num(1.0)),
        ("current_nozzle_id", num(0.0)),
        ("next_nozzle_id", num(0.0)),
        ("current_hotend", num(-1.0)),
        ("next_hotend", num(-1.0)),
        ("flush_length", num(191.006)),
        ("old_filament_temp", num(245.0)),
        ("new_filament_temp", num(245.0)),
        ("long_retractions_when_cut", Value::List(vec![Value::Bool(false); 2])),
        ("retraction_distances_when_cut", pair(10.0)),
        ("long_retractions_when_ec", Value::List(vec![Value::Bool(true); 2])),
        ("retraction_distances_when_ec", pair(3.0)),
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
    let text = layercraft_template::expand(X2D.gcode.filament_change, &scope).unwrap();
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
    let cut = change(25.0, 5.2, &[("long_retractions_when_cut", Value::List(vec![Value::Bool(true); 2]))]);
    assert!(has(&cut, "M620.11 P1 L0 I0 B-1 E-10 F200"));
    let ec = change(25.0, 5.2, &[("long_retractions_when_ec", Value::List(vec![Value::Bool(false); 2]))]);
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
    let text = layercraft_template::expand(X2D.gcode.layer_change, &scope).unwrap();
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
        ([0.0, 4.0, 40.0, 70.0, 45.0], vec![fans("121.125"), fans("140.25"), fans("159.375"), pla.clone()]),
        ([3.0, 3.0, 40.0, 70.0, 45.0], vec![fans("102"), fans("102"), fans("102"), pla.clone(), vec![]]),
        ([1.0, 0.0, 60.0, 70.0, 50.0], vec![fans("153"), pla.clone()]),
        (
            [3.0, 0.0, 40.0, 10.0, 70.0],
            vec![fans("102"), fans("102"), fans("102"), vec!["M142 P1 R50 S55 U0.1 V0.5 O55".to_string()]],
        ),
        ([0.0, 0.0, 40.0, 70.0, 51.0], vec![vec!["M142 P1 R50 S55 U0.7 V0.5 O55".to_string()], vec![]]),
    ] {
        for (layer, commands) in by_layer.iter().enumerate() {
            assert_eq!(&layer_change(layer as f64, filament), commands, "{filament:?} layer {layer}");
        }
    }
}
