//! Programs merged on one plate: layer by layer, nearest first, each with its filament as it left
//! it; passes of each material and free strands kept as they are.

use super::*;

#[test]
fn programs_merge_layer_by_layer_nearest_first_with_their_filament_as_they_left_it() {
    let bead = Some(Bead { role: Role::SOLID, width: 0.4 });
    // Over, down, push back what the layer below pulled, a bead, pull.
    let piece = |x: f64, pull: f64, first: bool| {
        vec![
            travel([x, 0.0, 0.6], 300.0),
            Move { to: None, e: if first { 0.0 } else { pull }, ..travel([0.0; 3], 30.0) },
            Move { e: 0.1, bead, ..travel([x + 5.0, 0.0, 0.2], 50.0) },
            Move { to: None, e: -pull, ..travel([0.0; 3], 30.0) },
        ]
    };
    let program = |x: f64, pull: f64, layers: usize| Program {
        carrier: Default::default(),
        passes: (0..layers)
            .map(|i| Pass {
                index: i,
                z: 0.2 * (i + 1) as f64,
                height: 0.2,
                warp: Default::default(),
                free: false,
                moves: piece(x, pull, i == 0),
                material: 0,
            })
            .collect(),
    };
    // A far one pulling 0.4, a near one pulling 0.8 and one layer taller.
    let merged = merge(&[program(100.0, 0.4, 2), program(10.0, 0.8, 3)], 25.0).unwrap();
    assert_eq!(merged.passes.len(), 3);
    assert_eq!(merged.passes.iter().map(|l| l.z).collect::<Vec<_>>(), vec![0.2, 0.4, 0.6000000000000001]);
    // At one height, the first given first, then the next nearest to where it ended.
    let starts: Vec<f64> = merged.passes[1]
        .moves
        .iter()
        .filter(|m| m.bead.is_none() && m.e == 0.0)
        .filter_map(|m| m.to)
        .map(|p| p[0])
        .collect();
    assert_eq!(starts, vec![10.0, 100.0]);
    // The filament balances: every push gives back exactly what is pulled before it.
    let mut pulled = 0.0;
    for m in all(&merged).into_iter().filter(|m| m.bead.is_none()) {
        pulled -= m.e;
        assert!(pulled > -1e-9, "pushed more than was pulled: {pulled}");
    }
    // The near one leaves 0.8 pulled; the far one expects 0.4: 0.4 is given back first, at 25 mm/s.
    let fix = merged.passes[1].moves.iter().find(|m| m.speed == 25.0).unwrap();
    assert!((fix.e - 0.4).abs() < 1e-12 && fix.to.is_none());
}

#[test]
fn merged_passes_keep_their_material_and_free_strands() {
    let pass = |z: f64, material: usize, free: bool, x: f64| Pass {
        index: 0,
        z,
        height: 0.2,
        warp: Default::default(),
        free,
        moves: vec![travel([x, 0.0, z], 300.0)],
        material,
    };
    // One program in two materials, its second layer in the second; one with free strands.
    let two = Program {
        carrier: Default::default(),
        passes: vec![pass(0.2, 0, false, 1.0), pass(0.2, 1, false, 2.0), pass(0.4, 1, false, 3.0)],
    };
    let strands = Program {
        carrier: Default::default(),
        passes: vec![pass(0.2, 0, false, 4.0), pass(0.4, 0, true, 5.0)],
    };
    let merged = merge(&[two, strands], 25.0).unwrap();
    // A layer's passes material by material, the filament loaded first, each with its own pieces.
    let kinds: Vec<(usize, usize, bool)> =
        merged.passes.iter().map(|p| (p.index, p.material, p.free)).collect();
    assert_eq!(kinds, [(0, 0, false), (0, 1, false), (1, 1, false), (1, 0, true)]);
    let xs: Vec<Vec<f64>> =
        merged.passes.iter().map(|p| p.moves.iter().filter_map(|m| m.to).map(|t| t[0]).collect()).collect();
    assert_eq!(xs, [vec![1.0, 4.0], vec![2.0], vec![3.0], vec![5.0]]);
}
