//! The machine's G-code around the passes: its templates expanded, a variable they miss named, and
//! a filament change between passes of two materials at the temperatures each holds.

use super::*;

#[test]
fn missing_template_variables_are_named() {
    let mut p = params();
    p.templates.end = "M140 S[bed]".into();
    let program = compiled(&line(1.0), &p);
    let err = frame(&program, &p.templates, &steady(), &scope()).unwrap_err().to_string();
    assert!(err.contains("bed"), "{err}");
}

#[test]
fn passes_of_other_materials_change_the_filament_between_them() {
    let p = params();
    let templates = Templates {
        layer_change: "; layer [layer_num]\n".into(),
        change_filament: "; change [current_filament_id] to [next_filament_id], [old_filament_temp] to [new_filament_temp]\n".into(),
        ..Templates::default()
    };
    let bead = |x: f64| Move {
        e: 0.1,
        bead: Some(Bead { role: Role::OUTER_WALL, width: 0.4 }),
        ..travel([x, 0.0, 0.2], 50.0)
    };
    let pass = |index: usize, material: usize, x: f64| Pass {
        index,
        z: 0.2 * (index + 1) as f64,
        height: 0.2,
        warp: Default::default(),
        free: false,
        moves: vec![travel([x - 1.0, 0.0, 0.2], 100.0), bead(x)],
        material,
    };
    // Layer 1 in two materials, layer 2 in the second only.
    let program = Program {
        carrier: Default::default(),
        passes: vec![pass(0, 0, 1.0), pass(1, 0, 2.0), pass(1, 1, 3.0), pass(2, 1, 4.0)],
    };
    // Each filament a first layer hotter than the rest, the bed cooler after the first layer.
    let heat = Heat { nozzle: vec![[250.0, 245.0], [240.0, 235.0]], bed: [70.0, 65.0] };
    let f = frame(&program, &templates, &heat, &scope()).unwrap();
    // The start leaves the first layer's temperatures; the second layer sets the others for the
    // filament loaded; after a change the nozzle waits for the new filament as hot as on its
    // first layer, for the pass laid on plastic the change left to cool, then on the next layer
    // takes its own.
    assert_eq!(
        f.passes,
        [
            "; layer 0\n",
            "; layer 1\nM140 S65 ; the bed for this layer\nM104 S245 ; the nozzle for this layer\n",
            "; change 0 to 1, 245 to 235\nM109 S240 ; wait for the new filament, as hot as on the first layer\n",
            "; layer 2\nM104 S235 ; the nozzle for this layer\n"
        ]
    );
    // A program that keeps its temperatures sets none.
    let one = Program { carrier: Default::default(), passes: vec![pass(0, 0, 1.0), pass(1, 0, 2.0)] };
    let still = Heat { nozzle: vec![[220.0, 220.0]], bed: [60.0, 60.0] };
    assert_eq!(frame(&one, &templates, &still, &scope()).unwrap().passes, ["; layer 0\n", "; layer 1\n"]);
    // The machine's end sees the filament last loaded.
    let end = Templates { end: "; end [current_filament_id]\n".into(), ..templates.clone() };
    assert!(frame(&program, &end, &heat, &scope()).unwrap().end.contains("; end 1"));
    // Without a change in the machine's templates, no frame.
    let bare = Templates { change_filament: String::new(), ..templates.clone() };
    assert!(frame(&program, &bare, &heat, &scope()).is_err());
    // Nor without a temperature for each material, or with one no printer can be asked for.
    assert!(frame(&program, &templates, &still, &scope()).is_err());
    for bad in [f64::NAN, f64::INFINITY, 0.0, -10.0] {
        let hostile = Heat { nozzle: vec![[250.0, 245.0], [240.0, bad]], bed: [70.0, 65.0] };
        assert!(frame(&program, &templates, &hostile, &scope()).is_err(), "{bad}");
        let hostile = Heat { bed: [70.0, bad], ..heat.clone() };
        assert!(frame(&program, &templates, &hostile, &scope()).is_err(), "{bad}");
    }
    // Layers are counted once, filament by material, the first layer over its passes.
    let s = summary_of(&program, &p);
    assert_eq!(s.layers, 3);
    assert_eq!(s.filament_mm_by_material, [0.2, 0.2]);
    assert!((s.filament_mm - 0.4).abs() < 1e-12);
    assert_eq!(s.volume_per_layer.len(), 3);
}
