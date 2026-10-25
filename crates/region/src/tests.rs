//! Region splits of analytic solids, checked against areas computed by hand.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::area;
use layercraft_mesh::Mesh;
use layercraft_mesh::solids::cuboid;
use layercraft_section::{Layering, section};

const PARAMS: RegionParams = RegionParams {
    walls: 2,
    outer_wall_width: 0.4,
    inner_wall_width: 0.5,
    top_layers: 3,
    bottom_layers: 2,
    first_layer_shrink: 0.0,
    first_layer_width: 0.4,
    one_wall_on_top: false,
    sparse_gap: 2.0,
    min_sparse_area: 15.0,
};

/// Equal up to the nanometre lattice: offsets round to it, moving a 10 mm square's area by
/// at most its perimeter times a step.
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 40.0 * layercraft_geom::GRID_MM
}

fn cube_regions() -> Regions {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])] };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    split(&sections, &PARAMS).unwrap()
}

#[test]
fn walls_sit_at_their_centre_lines() {
    let layer = &cube_regions().layers[4];
    assert_eq!(layer.walls.len(), 2);
    assert!(close(area(&layer.walls[0]), 9.6 * 9.6));
    // The outer bead's side on the edge, then beads at their tiling spacing s = w − h(1 − π/4).
    let s = |w: f64| w - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    let inner = 0.2 + s(0.4) / 2.0 + s(0.5) / 2.0;
    let side = 10.0 - 2.0 * inner;
    assert!(close(area(&layer.walls[1]), side * side), "{}", area(&layer.walls[1]));
}

#[test]
fn shells_and_sparse_partition_the_inside() {
    let regions = cube_regions();
    let s = |w: f64| w - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    for layer in &regions.layers {
        // The first layer's beads are all `first_layer_width` (0.4 here).
        let inner = if layer.index == 0 { 0.4 } else { 0.5 };
        let side = 10.0 - 2.0 * (0.2 + s(0.4) / 2.0 + s(inner));
        let inside = side * side;
        let total = area(&layer.top_surface)
            + area(&layer.solid)
            + area(&layer.bridge)
            + area(&layer.bridge_on_sparse)
            + area(&layer.sparse);
        assert!(close(total, inside), "layer {}: {total}", layer.index);
    }
    // 10 layers: 2 bottom shells, 5 sparse, then 3 top shells: the first bridges the sparse
    // infill, the last is exposed.
    let kinds: Vec<(bool, bool, bool, bool)> = regions
        .layers
        .iter()
        .map(|l| {
            (
                !l.top_surface.is_empty(),
                !l.solid.is_empty(),
                !l.bridge_on_sparse.is_empty(),
                !l.sparse.is_empty(),
            )
        })
        .collect();
    let solid = (false, true, false, false);
    let sparse = (false, false, false, true);
    let bridge = (false, false, true, false);
    let top = (true, false, false, false);
    assert_eq!(kinds, vec![solid, solid, sparse, sparse, sparse, sparse, sparse, bridge, solid, top]);
}

#[test]
fn bad_params_are_refused() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])] };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    assert!(split(&sections, &RegionParams { outer_wall_width: f64::NAN, ..PARAMS }).is_err());
    assert!(split(&sections, &RegionParams { walls: 1_000_000, ..PARAMS }).is_err());
}

#[test]
fn an_overhang_over_air_is_a_bridge() {
    // A 10 mm slab on two 2 mm posts: its first layer spans the 6 mm between them.
    let mesh = Mesh {
        bodies: vec![
            cuboid("left", [0.0; 3], [2.0, 10.0, 1.0]),
            cuboid("right", [8.0, 0.0, 0.0], [10.0, 10.0, 1.0]),
            cuboid("slab", [0.0, 0.0, 1.0], [10.0, 10.0, 2.0]),
        ],
    };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    let regions = split(&sections, &PARAMS).unwrap();
    let first = &regions.layers[5];
    let s = |w: f64| w - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    let inside = 10.0 - 2.0 * (0.2 + s(0.4) / 2.0 + s(0.5));
    // The 6 mm gap, plus a 0.5 mm bead's width onto each post.
    assert!(close(area(&first.bridge), 7.0 * inside), "{}", area(&first.bridge));
    assert!(regions.layers[6].bridge.is_empty());
}

#[test]
fn the_first_layer_is_wider_and_the_top_has_one_wall() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])] };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    let params = RegionParams { first_layer_width: 0.5, one_wall_on_top: true, ..PARAMS };
    let regions = split(&sections, &params).unwrap();
    assert!(close(area(&regions.layers[0].walls[0]), 9.5 * 9.5));
    let top = regions.layers.last().unwrap();
    // Nothing covers the top layer: its inner walls are drawn nowhere.
    assert!(top.inner_walls_within.as_ref().is_some_and(|r| r.is_empty()));
    let s = 0.4 - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    let side = 10.0 - 2.0 * (0.2 + s / 2.0);
    assert!(close(area(&top.top_surface), side * side), "{}", area(&top.top_surface));
    assert!(regions.layers[8].inner_walls_within.as_ref().is_some_and(|r| !r.is_empty()));
}

#[test]
fn a_band_narrower_than_the_sparse_gap_is_not_a_bridge() {
    // A wide block whose top shells start early over a narrow step: the step's ring over the
    // sparse infill below is 1 mm wide, under the 2 mm gap between sparse lines.
    let mesh = Mesh {
        bodies: vec![
            cuboid("base", [0.0; 3], [20.0, 20.0, 2.0]),
            cuboid("step", [0.0, 0.0, 2.0], [20.0, 1.9, 3.0]),
        ],
    };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    let regions = split(&sections, &PARAMS).unwrap();
    for layer in &regions.layers {
        let a = area(&layer.bridge_on_sparse);
        assert!(a == 0.0 || a > 10.0, "layer {}: a {a} mm² sliver of bridge", layer.index);
    }
}

#[test]
fn small_pieces_of_sparse_are_solid() {
    // Inside its walls a 5 × 5 mm post leaves 3.3² = 11 mm² (under 15: solid); a 10 × 10 mm
    // post leaves 69 mm² (sparse).
    for (side, sparse) in [(5.0, false), (10.0, true)] {
        let mesh = Mesh { bodies: vec![cuboid("post", [0.0; 3], [side, side, 3.0])] };
        let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
        let regions = split(&sections, &PARAMS).unwrap();
        let middle = &regions.layers[7];
        assert_eq!(!middle.sparse.is_empty(), sparse, "{side} mm post");
        assert!(!middle.solid.is_empty() || sparse);
    }
}
