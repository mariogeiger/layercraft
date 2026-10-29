//! Sections of analytic solids: areas, orientation, overlaps and vertices on the plane.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::Relief;
use layercraft_geom::{GRID_MM, area, polyline_length, signed_area};
use layercraft_mesh::solids::{cuboid, height_field, prism};
use std::collections::HashMap;

/// A rectangle `[x0, x1] × [y0, y1]` with corners rounded to `radius`, each arc cut into
/// `segments` chords: the outline of the pottery stamp's block.
fn rounded_rectangle(x0: f64, y0: f64, x1: f64, y1: f64, radius: f64, segments: usize) -> Vec<[f64; 2]> {
    let centres = [
        (x1 - radius, y0 + radius),
        (x1 - radius, y1 - radius),
        (x0 + radius, y1 - radius),
        (x0 + radius, y0 + radius),
    ];
    let quarter = std::f64::consts::FRAC_PI_2;
    let mut outline = Vec::new();
    for (k, (cx, cy)) in centres.iter().enumerate() {
        for s in 0..=segments {
            let a = -quarter + k as f64 * quarter + quarter * s as f64 / segments as f64;
            outline.push([cx + radius * a.cos(), cy + radius * a.sin()]);
        }
    }
    outline
}

const LAYERING: Layering = Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None };

#[test]
fn a_cube_cuts_into_squares() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])] };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert_eq!(sections.layers.len(), 10);
    assert_eq!(sections.open_chains, 0);
    for layer in &sections.layers {
        assert!((area(&layer.region) - 100.0).abs() < 1e-9, "{}", area(&layer.region));
    }
    let (loops, _) = section_loops(&mesh, |_| 1.0);
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
    let (loops, open) = section_loops(&mesh, |_| 1.0);
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
    let bad = Layering { first_layer_height: f64::NAN, layer_height: 0.2, foliation: None };
    assert!(section(&mesh, &bad).is_err());
    let tiny = Layering { first_layer_height: 0.2, layer_height: 0.0, foliation: None };
    assert!(section(&mesh, &tiny).is_err());
    assert!(section(&Mesh::default(), &LAYERING).is_err());
}

#[test]
fn a_hole_in_the_mesh_is_counted_not_fatal() {
    let mut body = cuboid("c", [0.0; 3], [1.0; 3]);
    body.triangles
        .retain(|t| !(body.vertices[t[0] as usize][0] == 1.0 && body.vertices[t[1] as usize][0] == 1.0));
    let (_, open) = section_loops(&Mesh { bodies: vec![body] }, |_| 0.5);
    assert!(open > 0);
}

/// Bent layers that follow top surfaces of up to 20°, within the X2D's head.
fn bent() -> Layering {
    Layering {
        first_layer_height: 0.2,
        layer_height: 0.2,
        foliation: Some(Foliation {
            max_slope: 20.0,
            min_layer_height: 0.1,
            max_layer_height: 0.3,
            max_rise: 3.0,
            parallel_layers: 2,
            cell: 0.5,
            face_radius: 0.5,
        }),
    }
}

/// Every edge of a body used once each way.
fn closed(body: &layercraft_mesh::Body) -> bool {
    let mut uses: HashMap<(u32, u32), i32> = HashMap::new();
    for t in &body.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *uses.entry((a.min(b), a.max(b))).or_default() += if a < b { 1 } else { -1 };
        }
    }
    uses.values().all(|&u| u == 0)
}

/// The top of the highest layer whose region holds `p`, and that layer.
fn top_at(sections: &Sections, p: [f64; 2]) -> Option<(f64, usize)> {
    let inside = |r: &Region| {
        r.iter()
            .any(|shape| layercraft_geom::encloses(shape.iter().flat_map(|c| layercraft_geom::edges(c)), p))
    };
    sections.layers.iter().rev().find(|l| inside(&l.region)).map(|l| (l.top + l.warp.at(p), l.index))
}

/// The volume the layers fill, summing each one's thickness over a fine grid of its region.
fn filled(sections: &Sections, lo: [f64; 2], hi: [f64; 2], step: f64) -> f64 {
    let mut volume = 0.0;
    let zero = Relief::default();
    for l in &sections.layers {
        let below = l.index.checked_sub(1).map(|b| &sections.layers[b]);
        let edges: Vec<_> = l.region.iter().flatten().flat_map(|c| layercraft_geom::edges(c)).collect();
        let mut y = lo[1] + step / 2.0;
        while y < hi[1] {
            let mut x = lo[0] + step / 2.0;
            while x < hi[0] {
                let p = [x, y];
                if layercraft_geom::encloses(edges.iter().copied(), p) {
                    let bottom = below.map_or(0.0, |b| b.top + b.warp.at(p));
                    let _ = &zero;
                    volume += (l.top + l.warp.at(p) - bottom) * step * step;
                }
                x += step;
            }
            y += step;
        }
    }
    volume
}

/// The thinnest and thickest slab over the rectangle, the steepest leaf.
fn bounds_of(sections: &Sections, lo: [f64; 2], hi: [f64; 2]) -> (f64, f64, f64) {
    let (mut thin, mut thick, mut steep) = (f64::MAX, 0.0f64, 0.0f64);
    for l in &sections.layers {
        steep = steep.max(l.warp.steepest());
        let below = l.index.checked_sub(1).map(|b| &sections.layers[b]);
        for j in 0..=40 {
            for i in 0..=40 {
                let p =
                    [lo[0] + (hi[0] - lo[0]) * i as f64 / 40.0, lo[1] + (hi[1] - lo[1]) * j as f64 / 40.0];
                let t = l.top + l.warp.at(p) - below.map_or(0.0, |b| b.top + b.warp.at(p));
                thin = thin.min(t);
                thick = thick.max(t);
            }
        }
    }
    (thin, thick, steep)
}

#[test]
fn bent_layers_follow_a_sloped_top_exactly() {
    // A wedge rising 2 mm over 10 mm, 11.3°, and a gable roof of 8.5° each way.
    type Surface = (&'static str, fn(f64, f64) -> f64, f64);
    let surfaces: [Surface; 2] = [
        ("wedge", |x, _| 3.0 + 0.2 * x, 320.0),
        ("gable", |x, _| 4.0 - 0.15 * (x - 5.0).abs(), 8.0 * (40.0 - 0.15 * 25.0)),
    ];
    for (name, top, volume) in surfaces {
        let mesh = Mesh { bodies: vec![height_field(name, [0.0, 0.0], [10.0, 8.0], 10, top)] };
        assert!((mesh.volume() - volume).abs() < 1e-9, "{name}: {}", mesh.volume());
        let sections = section(&mesh, &bent()).unwrap();
        assert_eq!((sections.open_chains, sections.unsplit_cells), (0, 0), "{name}");
        assert!(sections.layers.iter().any(|l| !l.warp.is_zero()), "{name}: nothing bent");
        // The highest layer over each point tops out on the surface itself, away from the
        // ridge; across it, within the grid's rounding of a crease: a cell times the change of
        // slope, over four.
        for p in [[0.7, 0.4], [2.3, 4.1], [3.9, 7.6], [6.1, 2.2], [9.6, 7.7], [4.95, 3.3], [5.0, 6.0]] {
            let (z, layer) = top_at(&sections, p).unwrap();
            let off = (z - top(p[0], p[1])).abs();
            let near_ridge = name == "gable" && (p[0] - 5.0).abs() < 0.5 * std::f64::consts::SQRT_2;
            let bound = if near_ridge { 0.5 * 0.3 / 4.0 } else { 1e-9 };
            assert!(off <= bound, "{name} at {p:?}: layer {layer} tops at {z}, {off} off");
        }
        let (thin, thick, steep) = bounds_of(&sections, [0.0, 0.0], [10.0, 8.0]);
        assert!(thin >= 0.1 - 1e-9 && thick <= 0.3 + 1e-9, "{name}: slabs {thin} to {thick} mm");
        // Along the grid's sides and diagonals a leaf leans at most tan 20°; across a triangle,
        // where two of them disagree, √2 times that.
        assert!(
            steep <= 20f64.to_radians().tan() * std::f64::consts::SQRT_2 + 1e-9,
            "{name}: a leaf leans {steep}"
        );
        let v = filled(&sections, [0.0, 0.0], [10.0, 8.0], 0.05);
        assert!((v - volume).abs() < 2e-3 * volume, "{name}: the layers fill {v} mm³ of {volume}");
    }
}

/// How far, at worst, the leaf below stands above the nozzle's flat face, of radius `face`,
/// printing each layer: over the face's disc about points of the rectangle, every vertex of the
/// leaves' grid and a point inside each of its cells.
fn face_contact(sections: &Sections, lo: [f64; 2], hi: [f64; 2], face: f64) -> f64 {
    let grid = sections.layers.iter().find(|l| !l.warp.is_zero()).unwrap().warp.grid;
    let [i0, j0] = grid.coords(lo).map(|c| c.floor() as i64);
    let [i1, j1] = grid.coords(hi).map(|c| c.ceil() as i64);
    let mut worst = f64::NEG_INFINITY;
    for pair in sections.layers.windows(2) {
        let (below, layer) = (&pair[0], &pair[1]);
        if below.warp.is_zero() && layer.warp.is_zero() {
            continue;
        }
        for j in j0..=j1 {
            for i in i0..=i1 {
                for [a, b] in [[0.0, 0.0], [0.62, 0.29]] {
                    let v = grid.vertex(i, j);
                    let x = [v[0] + a * grid.cell, v[1] + b * grid.cell];
                    let tip = layer.top + layer.warp.at(x);
                    for r in [face, face / 2.0] {
                        for k in 0..64 {
                            let phi = k as f64 * std::f64::consts::TAU / 64.0;
                            let y = [x[0] + r * phi.cos(), x[1] + r * phi.sin()];
                            worst = worst.max(below.top + below.warp.at(y) - tip);
                        }
                    }
                }
            }
        }
    }
    worst
}

#[test]
fn the_leaf_below_never_stands_above_the_nozzle_face() {
    // Slopes of 17° along the plate's diagonal, which the grid's triangles take worst, and 16.7°
    // along its side, steep enough that the face, not the thinnest slab, bounds how far the
    // leaves below come back to flat.
    type Surface = (&'static str, fn(f64, f64) -> f64);
    let surfaces: [Surface; 2] =
        [("diagonal", |x, y| 2.0 + 0.216 * (x + y)), ("gable", |x, _| 4.0 - 0.3 * (x - 5.0).abs())];
    let layering = bent();
    let face = layering.foliation.as_ref().unwrap().face_radius;
    for (name, top) in surfaces {
        let mesh = Mesh { bodies: vec![height_field(name, [0.0, 0.0], [10.0, 8.0], 10, top)] };
        let sections = section(&mesh, &layering).unwrap();
        assert!(sections.layers.iter().any(|l| !l.warp.is_zero()), "{name}: nothing bent");
        let contact = face_contact(&sections, [0.0, 0.0], [10.0, 8.0], face);
        assert!(contact <= 1e-9, "{name}: the leaf below stands {contact} mm above the face");
    }
}

#[test]
fn refining_keeps_the_mesh_closed_and_its_volume() {
    let mesh = Mesh {
        bodies: vec![height_field("wedge", [0.0, 0.0], [10.0, 8.0], 7, |x, y| 3.0 + 0.2 * x - 0.05 * y)],
    };
    let grid = layercraft_geom::Grid { origin: [-0.37, 0.21], cell: 0.5 };
    let lines = layercraft_geom::Lines { levels: [[-2, 30], [-5, 25], [-40, 40]] };
    let (refined, failed) = refine(&mesh, &grid, &lines, [-1.0, 10.0]);
    assert_eq!(failed, 0);
    assert!(refined.bodies[0].triangles.len() > 10 * mesh.bodies[0].triangles.len());
    assert!(closed(&refined.bodies[0]));
    assert!((refined.volume() - mesh.volume()).abs() < 1e-9 * mesh.volume());
}
