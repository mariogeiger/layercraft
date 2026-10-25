//! Beads of analytic solids: coverage, seams and order.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{area, bead_area, polyline_length};
use layercraft_mesh::Mesh;
use layercraft_mesh::solids::cuboid;
use layercraft_region::{RegionParams, split};
use layercraft_section::{Layering, section};

const PATHS: PathParams = PathParams {
    outer_wall_width: 0.4,
    inner_wall_width: 0.4,
    top_surface_width: 0.4,
    solid_width: 0.4,
    sparse_width: 0.4,
    sparse_density: 0.2,
    infill_wall_overlap: 0.0,
    infill_angle: 45.0,
};

fn paths_of(mesh: &Mesh) -> (Regions, Paths) {
    let sections = section(mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    let params = RegionParams {
        walls: 2,
        outer_wall_width: 0.4,
        inner_wall_width: 0.4,
        top_layers: 3,
        bottom_layers: 3,
        first_layer_shrink: 0.0,
    };
    let regions = split(&sections, &params).unwrap();
    let paths = trace(&regions, &PATHS).unwrap();
    (regions, paths)
}

fn filled_area(paths: &[Path], role: Role) -> f64 {
    paths
        .iter()
        .filter(|p| p.role == role)
        .map(|p| {
            let mut pts = p.points.clone();
            if p.closed {
                pts.push(pts[0]);
            }
            // The plastic laid, spread over the layer's height: the area it fills.
            polyline_length(&pts) * bead_area(p.width, 0.2) / 0.2
        })
        .sum()
}

#[test]
fn solid_hatch_covers_its_area() {
    let (regions, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [20.0, 20.0, 3.0])] });
    let layer = &paths.layers[0];
    let solid = area(&regions.layers[0].solid);
    let covered = filled_area(&layer.paths, Role::Solid);
    assert!((covered / solid - 1.0).abs() < 0.03, "{covered} for {solid}");
}

#[test]
fn sparse_grid_has_its_density() {
    let (regions, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [40.0, 40.0, 3.0])] });
    let i = 7;
    let sparse = area(&regions.layers[i].sparse);
    let covered = filled_area(&paths.layers[i].paths, Role::Sparse);
    assert!((covered / sparse - 0.2).abs() < 0.02, "{}", covered / sparse);
}

#[test]
fn walls_come_inner_first_and_seams_align() {
    let (_, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])] });
    let roles: Vec<Role> = paths.layers[5].paths.iter().map(|p| p.role).take(2).collect();
    assert_eq!(roles, vec![Role::InnerWall, Role::OuterWall]);
    let seams: Vec<Point> = paths
        .layers
        .iter()
        .map(|l| l.paths.iter().find(|p| p.role == Role::OuterWall).unwrap().points[0])
        .collect();
    assert!(seams.windows(2).all(|w| w[0] == w[1]), "{seams:?}");
}

#[test]
fn separate_islands_are_printed_whole() {
    let mesh = Mesh {
        bodies: vec![cuboid("a", [0.0; 3], [5.0, 5.0, 1.0]), cuboid("b", [20.0, 0.0, 0.0], [25.0, 5.0, 1.0])],
    };
    let (_, paths) = paths_of(&mesh);
    let side: Vec<bool> = paths.layers[2].paths.iter().map(|p| p.points[0][0] > 10.0).collect();
    let switches = side.windows(2).filter(|w| w[0] != w[1]).count();
    assert_eq!(switches, 1, "{side:?}");
}

#[test]
fn beads_have_no_zero_length_segments() {
    let (_, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])] });
    for path in paths.layers.iter().flat_map(|l| l.paths.iter()) {
        let mut pts = path.points.clone();
        if path.closed {
            pts.push(pts[0]);
        }
        assert!(pts.windows(2).all(|w| w[0] != w[1]), "{path:?}");
    }
}

#[test]
fn sparse_beads_never_cross_on_a_layer() {
    let (_, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [40.0, 40.0, 3.0])] });
    for layer in &paths.layers {
        let directions: Vec<f64> = layer
            .paths
            .iter()
            .filter(|p| p.role == Role::Sparse)
            .map(|p| {
                let (a, b) = (p.points[0], p.points[p.points.len() - 1]);
                ((b[1] - a[1]).atan2(b[0] - a[0]).to_degrees() + 360.0) % 180.0
            })
            .collect();
        assert!(
            directions.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-6),
            "layer {}: {directions:?}",
            layer.index
        );
    }
}
