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
};

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
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
    // Outer bead 0.4, then the centre of a 0.5 bead: 0.65 in from the edge.
    assert!(close(area(&layer.walls[1]), 8.7 * 8.7));
}

#[test]
fn shells_and_sparse_partition_the_inside() {
    let regions = cube_regions();
    let inside = 8.2 * 8.2; // 0.4 + 0.5 of walls on each side
    for layer in &regions.layers {
        let total = area(&layer.top_surface) + area(&layer.solid) + area(&layer.sparse);
        assert!(close(total, inside), "layer {}: {total}", layer.index);
    }
    // 10 layers: 2 bottom shells, 3 top shells (the last one exposed), 5 sparse in between.
    let kinds: Vec<(bool, bool, bool)> = regions
        .layers
        .iter()
        .map(|l| (!l.top_surface.is_empty(), !l.solid.is_empty(), !l.sparse.is_empty()))
        .collect();
    let solid = (false, true, false);
    let sparse = (false, false, true);
    let top = (true, false, false);
    assert_eq!(kinds, vec![solid, solid, sparse, sparse, sparse, sparse, sparse, solid, solid, top]);
}

#[test]
fn bad_params_are_refused() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])] };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    assert!(split(&sections, &RegionParams { outer_wall_width: f64::NAN, ..PARAMS }).is_err());
    assert!(split(&sections, &RegionParams { walls: 1_000_000, ..PARAMS }).is_err());
}
