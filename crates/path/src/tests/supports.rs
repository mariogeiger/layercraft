//! The supports' lines: each column a loop stacked layer on layer, spanned at its top by its
//! interface, the whole top in the interface's filament.

use super::*;

#[test]
fn supports_are_loops_under_a_cantilever_spanned_by_their_interface() {
    // A T: a 4 mm stem under a 20 mm bar, which overhangs 8 mm on either side from z = 10.
    let mesh = Mesh {
        bodies: vec![
            cuboid("stem", [-2.0, -2.0, 0.0], [2.0, 2.0, 10.0]),
            cuboid("bar", [-10.0, -2.0, 10.0], [10.0, 2.0, 12.0]),
        ],
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(region_params())).unwrap();
    let paths = trace(&regions, &Field::uniform(PATHS)).unwrap();
    let of = |k: usize, keep: &dyn Fn(Role) -> bool| -> Vec<Path> {
        paths.passes[k].paths.iter().filter(|p| keep(p.role)).cloned().collect()
    };
    let loops = |k: usize| of(k, &|r| r == Role::SUPPORT);
    let closed = |r: &Path| r.points.iter().chain(r.points.first()).copied().collect::<Vec<Point>>();
    let interface = |k: usize| of(k, &|r| r.kind == Kind::Support && r.face == Face::Top);
    // Each 7.65 × 4 mm column is a loop a bead's width in, the same on every layer, and nothing
    // else up to its interface.
    let perimeter = 2.0 * ((7.65 - 0.4) + (4.0 - 0.4));
    for k in [1, 20, 46] {
        let rings = loops(k);
        assert!(rings.len() == 2 && rings.iter().all(|r| r.closed), "layer {k}");
        for ring in &rings {
            let mut points = ring.points.clone();
            points.extend(ring.points.first().copied());
            assert!((polyline_length(&points) - perimeter).abs() < 0.05, "layer {k}");
            let stacked: Vec<Vec<Point>> = loops(20).iter().map(closed).collect();
            assert!(ring.points.iter().all(|&p| distance_to(p, &stacked) < 1e-3), "layer {k}");
        }
        assert!(interface(k).is_empty(), "layer {k}");
    }
    // The bar's underside is at z 10: the columns' tops, their interface, 0.2 mm under it, on
    // layers 47 and 48; the first over air, slowly, the second on it; layer 49 is the gap.
    let rings: Vec<Vec<Point>> = loops(46).iter().map(closed).collect();
    for (k, support) in [(47, Rest::Air), (48, Rest::Solid)] {
        let lines = interface(k);
        assert!(!lines.is_empty() && lines.iter().all(|p| p.role.rest == support), "layer {k}");
        // Across each column the short way, every line runs from its loop to its loop, held at
        // both ends by the loop under it.
        for line in &lines {
            let (Some(&a), Some(&b)) = (line.points.first(), line.points.last()) else { continue };
            assert!((a[0] - b[0]).abs() < 1e-3, "layer {k}: {a:?} to {b:?} not across");
            let off = distance_to(a, &rings).max(distance_to(b, &rings));
            assert!(off < 1e-3, "layer {k}: {a:?} to {b:?} ends {off} mm off the loop");
        }
    }
    assert_eq!(of(49, &|r| r.kind == Kind::Support).len(), 0);
    // The bar rests on the columns as solid, a layer thick, not as strands hanging over air.
    assert!(paths.passes[50].paths.iter().all(|p| p.role.rest != Rest::Air || p.role.kind == Kind::Wall));
    // Every support line stays 0.35 mm off the stem, less its half width.
    for layer in &paths.passes {
        for path in layer.paths.iter().filter(|p| p.role.kind == Kind::Support) {
            assert!(path.points.iter().all(|p| p[0].abs() >= 2.35 + 0.2 - 1e-3 || p[1].abs() > 2.0));
        }
    }
}

#[test]
fn an_interface_layer_lays_the_columns_loop_in_the_interface_filament() {
    // The T's columns in filament 0, their interface in filament 1: below the interface, the
    // loops are filament 0's; on its two layers the whole top, loops and lines, is filament 1's,
    // the lines ending on a loop of their own filament, and filament 0 lays no support there.
    let mesh = Mesh {
        bodies: vec![
            cuboid("stem", [-2.0, -2.0, 0.0], [2.0, 2.0, 10.0]),
            cuboid("bar", [-10.0, -2.0, 10.0], [10.0, 2.0, 12.0]),
        ],
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let mut params = region_params();
    params.supports.interface_material = Some(1);
    let regions = split(&sections, &Field::uniform(params)).unwrap();
    let paths = trace(&regions, &Field::uniform(PATHS)).unwrap();
    let support = |index: usize, material: usize, role: Role| {
        let passes = paths.passes.iter().filter(|p| p.index == index && p.material == material);
        passes.flat_map(|p| &p.paths).filter(|p| p.role == role).count()
    };
    assert!(support(46, 0, Role::SUPPORT) == 2 && support(46, 1, Role::SUPPORT) == 0);
    for (index, lines) in [(47, Role::SUPPORT_INTERFACE), (48, Role::SUPPORT_INTERFACE_RESTING)] {
        assert_eq!(support(index, 1, Role::SUPPORT), 2, "layer {index}: the loops");
        assert!(support(index, 1, lines) > 0, "layer {index}: the lines");
        let filament_0 = paths.passes.iter().filter(|p| p.index == index && p.material == 0);
        assert!(filament_0.flat_map(|p| &p.paths).all(|p| p.role.kind != Kind::Support), "layer {index}");
    }
}
