//! Sections of analytic solids: areas, orientation, overlaps and vertices on the plane.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{GRID_MM, area, polyline_length, signed_area};
use layercraft_mesh::solids::{cuboid, prism, rounded_rectangle};

const LAYERING: Layering = Layering { first_layer_height: 0.2, layer_height: 0.2 };

#[test]
fn a_cube_cuts_into_squares() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])] };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert_eq!(sections.layers.len(), 10);
    assert_eq!(sections.open_chains, 0);
    for layer in &sections.layers {
        assert!((area(&layer.region) - 100.0).abs() < 1e-9, "{}", area(&layer.region));
    }
    let (loops, _) = section_loops(&mesh, 1.0);
    assert!(loops.iter().all(|l| signed_area(l) > 0.0), "loops must be counterclockwise");
}

#[test]
fn a_plane_through_vertices_still_closes() {
    // z = 1 holds the top vertices of the lower box and the bottom ones of the upper box. Vertices
    // on the plane count as above it, so the cut is the one just below: the lower box's square.
    let mesh = Mesh {
        bodies: vec![
            cuboid("lower", [0.0; 3], [4.0, 4.0, 1.0]),
            cuboid("upper", [1.0, 1.0, 1.0], [3.0, 3.0, 2.0]),
        ],
    };
    let (loops, open) = section_loops(&mesh, 1.0);
    assert_eq!(open, 0);
    assert!((area(&winding_region(&loops).unwrap()) - 16.0).abs() < 1e-9);
}

#[test]
fn overlapping_bodies_merge() {
    let mesh = Mesh {
        bodies: vec![cuboid("a", [0.0; 3], [4.0, 4.0, 1.0]), cuboid("b", [2.0, 2.0, 0.0], [6.0, 6.0, 1.0])],
    };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert!((area(&sections.layers[0].region) - 28.0).abs() < 1e-9);
}

#[test]
fn the_stamp_block_has_its_rounded_area() {
    let outline = rounded_rectangle(0.0, 0.0, 24.0, 12.0, 3.0, 64);
    let mesh = Mesh { bodies: vec![prism("block", &outline, 0.0, 25.0)] };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert_eq!(sections.layers.len(), 125);
    let exact = signed_area(&outline);
    let expected_limit = 288.0 - (4.0 - std::f64::consts::PI) * 9.0;
    assert!((exact - expected_limit).abs() < 0.01);
    // Snapping to the nanometre lattice moves each vertex by at most half a step on each axis.
    let mut closed = outline.clone();
    closed.push(outline[0]);
    let snap = polyline_length(&closed) * GRID_MM;
    for layer in &sections.layers {
        assert!((area(&layer.region) - exact).abs() < snap, "{}", area(&layer.region) - exact);
    }
}

#[test]
fn bad_layering_is_refused() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])] };
    let bad = Layering { first_layer_height: f64::NAN, layer_height: 0.2 };
    assert!(section(&mesh, &bad).is_err());
    let tiny = Layering { first_layer_height: 0.2, layer_height: 0.0 };
    assert!(section(&mesh, &tiny).is_err());
    assert!(section(&Mesh::default(), &LAYERING).is_err());
}

#[test]
fn a_hole_in_the_mesh_is_counted_not_fatal() {
    let mut body = cuboid("c", [0.0; 3], [1.0; 3]);
    body.triangles
        .retain(|t| !(body.vertices[t[0] as usize][0] == 1.0 && body.vertices[t[1] as usize][0] == 1.0));
    let (_, open) = section_loops(&Mesh { bodies: vec![body] }, 0.5);
    assert!(open > 0);
}
