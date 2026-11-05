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
fn materials_partition_each_layer_the_later_winning() {
    // A 10 × 10 × 2 base and a 4 × 4 inlay of another material from z = 1 to 3, overlapping it.
    let base = cuboid("base", [0.0; 3], [10.0, 10.0, 2.0]);
    let inlay = layercraft_mesh::Body { material: 1, ..cuboid("inlay", [2.0, 2.0, 1.0], [6.0, 6.0, 3.0]) };
    let colour = |c: &str| layercraft_mesh::Material { colour: Some(c.into()) };
    let mesh = Mesh { bodies: vec![base, inlay], materials: vec![colour("#0A2989"), colour("#FEC600")] };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert_eq!(sections.layers.len(), 15);
    for layer in &sections.layers {
        let mid = layer.top - layer.height / 2.0;
        let want = if mid < 1.0 {
            [100.0, 0.0]
        } else if mid < 2.0 {
            [84.0, 16.0]
        } else {
            [0.0, 16.0]
        };
        let got: Vec<f64> = layer.by_material().into_iter().map(area).collect();
        assert_eq!(got.len(), 2);
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-9, "z = {mid}: {got:?}, want {want:?}");
        }
        assert!((got.iter().sum::<f64>() - area(&layer.region)).abs() < 1e-9);
    }
    // One material: no parts listed, the layer is its own.
    let one = section(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])], ..Mesh::default() }, &LAYERING)
        .unwrap();
    assert!(one.layers.iter().all(|l| l.materials.is_empty() && l.by_material().len() == 1));
}

#[test]
fn a_cube_cuts_into_squares() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])], ..Mesh::default() };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert_eq!(sections.layers.len(), 10);
    assert_eq!(sections.open_chains, 0);
    for layer in &sections.layers {
        assert!((area(&layer.region) - 100.0).abs() < 1e-9, "{}", area(&layer.region));
    }
    let loops = section_loops(&mesh, |_| 1.0).loops;
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
        ..Mesh::default()
    };
    let cut = section_loops(&mesh, |_| 1.0);
    assert_eq!(cut.open, 0);
    assert!((area(&winding_region(&cut.loops).unwrap()) - 16.0).abs() < 1e-9);
}

#[test]
fn overlapping_bodies_merge() {
    let mesh = Mesh {
        bodies: vec![cuboid("a", [0.0; 3], [4.0, 4.0, 1.0]), cuboid("b", [2.0, 2.0, 0.0], [6.0, 6.0, 1.0])],
        ..Mesh::default()
    };
    let sections = section(&mesh, &LAYERING).unwrap();
    assert!((area(&sections.layers[0].region) - 28.0).abs() < 1e-9);
}

#[test]
fn the_stamp_block_has_its_rounded_area() {
    let outline = rounded_rectangle(0.0, 0.0, 24.0, 12.0, 3.0, 64);
    let mesh = Mesh { bodies: vec![prism("block", &outline, 0.0, 25.0)], ..Mesh::default() };
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
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])], ..Mesh::default() };
    let bad = Layering { first_layer_height: f64::NAN, layer_height: 0.2, foliation: None };
    assert!(section(&mesh, &bad).is_err());
    let tiny = Layering { first_layer_height: 0.2, layer_height: 0.0, foliation: None };
    assert!(section(&mesh, &tiny).is_err());
    assert!(section(&Mesh::default(), &LAYERING).is_err());
}

#[test]
fn a_hole_in_the_mesh_is_counted_and_closed_across() {
    let mut body = cuboid("c", [0.0; 3], [1.0; 3]);
    body.triangles
        .retain(|t| !(body.vertices[t[0] as usize][0] == 1.0 && body.vertices[t[1] as usize][0] == 1.0));
    let cut = section_loops(&Mesh { bodies: vec![body], ..Mesh::default() }, |_| 0.5);
    assert!(cut.open > 0);
    assert!(cut.widest_gap <= 1.0 + 1e-12);
    assert!((area(&winding_region(&cut.loops).unwrap()) - 1.0).abs() < 1e-9);
}

/// The regular `n`-gon of circumradius `r`, a vertex on each axis when `n` is a multiple of four.
fn regular(n: usize, r: f64) -> Vec<[f64; 2]> {
    (0..n)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            [r * a.cos(), r * a.sin()]
        })
        .collect()
}

#[test]
fn a_crack_between_two_tessellations_keeps_the_hole() {
    // A tube of outer radius 8 around a hole of radius 2, its walls tessellated with 24 and 12
    // chords below z = 1 and with 16 and 20 above, so that the faces meet along z = 1 without a
    // shared edge: the crack SolidCraft's exports leave where two faces meet along a circle. A
    // leaf tilted across the crack cuts the lower part where x < 0 and the upper one where x > 0;
    // both outlines have vertices on the y axis, so its section is half of each polygon.
    let wall = |name: &str, n: usize, r: f64, z0: f64, z1: f64, hole: bool| {
        let mut body = prism(name, &regular(n, r), z0, z1);
        let at = |i: u32| body.vertices[i as usize][2];
        body.triangles.retain(|t| !t.iter().all(|&i| at(i) == 1.0));
        if hole {
            body.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }
        body
    };
    let mesh = Mesh {
        bodies: vec![
            wall("outer below", 24, 8.0, 0.0, 1.0, false),
            wall("outer above", 16, 8.0, 1.0, 2.0, false),
            wall("hole below", 12, 2.0, 0.0, 1.0, true),
            wall("hole above", 20, 2.0, 1.0, 2.0, true),
        ],
        ..Mesh::default()
    };
    let cut = section_loops(&mesh, |v| 1.0 + 0.05 * v[0]);
    let region = winding_region(&cut.loops).unwrap();
    let half = |n: usize, r: f64| n as f64 * r * r * (std::f64::consts::TAU / n as f64).sin() / 4.0;
    let exact = half(24, 8.0) + half(16, 8.0) - half(12, 2.0) - half(20, 2.0);
    assert!(cut.open > 0);
    assert_eq!(region.len(), 1, "one piece");
    assert_eq!(region[0].len(), 2, "its outline and the hole");
    // Within the lattice the region is snapped to, along its outline.
    let perimeter = std::f64::consts::TAU * (8.0 + 2.0);
    assert!((area(&region) - exact).abs() < perimeter * GRID_MM, "{} vs {exact}", area(&region));
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
            face_reach: 0.5 + 0.42 / 2.0,
            ironing: 0.3,
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

/// Each layer's top over its plane, its warp on the stage's carrier.
fn tops(sections: &Sections) -> Vec<Relief> {
    sections.layers.iter().map(|l| sections.carrier.relief(&l.warp).unwrap()).collect()
}

/// The height of each layer's top at `p`.
fn heights(sections: &Sections, p: [f64; 2]) -> Vec<f64> {
    sections.layers.iter().zip(tops(sections)).map(|(l, top)| l.top + top.at(p)).collect()
}

/// The top of the highest layer whose region holds `p`, and that layer.
fn top_at(sections: &Sections, p: [f64; 2]) -> Option<(f64, usize)> {
    let inside = |r: &Region| {
        r.iter()
            .any(|shape| layercraft_geom::encloses(shape.iter().flat_map(|c| layercraft_geom::edges(c)), p))
    };
    let heights = heights(sections, p);
    sections.layers.iter().zip(heights).rev().find(|(l, _)| inside(&l.region)).map(|(l, z)| (z, l.index))
}

/// The volume the layers fill, summing each one's thickness over a fine grid of its region.
fn filled(sections: &Sections, lo: [f64; 2], hi: [f64; 2], step: f64) -> f64 {
    let mut volume = 0.0;
    let tops = tops(sections);
    let height = |k: usize, p: [f64; 2]| sections.layers[k].top + tops[k].at(p);
    for (k, l) in sections.layers.iter().enumerate() {
        let edges: Vec<_> = l.region.iter().flatten().flat_map(|c| layercraft_geom::edges(c)).collect();
        let mut y = lo[1] + step / 2.0;
        while y < hi[1] {
            let mut x = lo[0] + step / 2.0;
            while x < hi[0] {
                let p = [x, y];
                if layercraft_geom::encloses(edges.iter().copied(), p) {
                    let bottom = k.checked_sub(1).map_or(0.0, |b| height(b, p));
                    volume += (height(k, p) - bottom) * step * step;
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
    let tops = tops(sections);
    let height = |k: usize, p: [f64; 2]| sections.layers[k].top + tops[k].at(p);
    for (k, top) in tops.iter().enumerate() {
        steep = steep.max(top.steepest());
        for j in 0..=40 {
            for i in 0..=40 {
                let p =
                    [lo[0] + (hi[0] - lo[0]) * i as f64 / 40.0, lo[1] + (hi[1] - lo[1]) * j as f64 / 40.0];
                let t = height(k, p) - k.checked_sub(1).map_or(0.0, |b| height(b, p));
                thin = thin.min(t);
                thick = thick.max(t);
            }
        }
    }
    (thin, thick, steep)
}

/// The most a leaf linear on a triangle can lean, per unit of the bound along its edges: the
/// secant of half its largest angle.
fn lean_bound([a, b, c]: [[f64; 2]; 3]) -> f64 {
    let angle = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        let (u, v) = ([q[0] - p[0], q[1] - p[1]], [r[0] - p[0], r[1] - p[1]]);
        (u[0] * v[1] - u[1] * v[0]).abs().atan2(u[0] * v[0] + u[1] * v[1])
    };
    let largest = angle(a, b, c).max(angle(b, c, a)).max(angle(c, a, b));
    1.0 / (largest / 2.0).cos()
}

/// The steepest any layer leans over its own bound, the slope times its triangle's lean bound.
fn steepest_lean(sections: &Sections, slope: f64) -> f64 {
    let mut worst = 0.0f64;
    for top in tops(sections) {
        for (corners, g) in top.facets() {
            worst = worst.max(g[0].hypot(g[1]) / (slope * lean_bound(corners)));
        }
    }
    worst
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
        let mesh =
            Mesh { bodies: vec![height_field(name, [0.0, 0.0], [10.0, 8.0], 10, top)], ..Mesh::default() };
        assert!((mesh.volume() - volume).abs() < 1e-9, "{name}: {}", mesh.volume());
        let sections = section(&mesh, &bent()).unwrap();
        assert_eq!((sections.open_chains, sections.unsplit_cells), (0, 0), "{name}");
        assert!(sections.layers.iter().any(|l| !l.warp.is_zero()), "{name}: nothing bent");
        // Stored, the stage holds its carrier once and each layer's warp on it.
        let stored = serde_json::to_string(&sections).unwrap();
        assert_eq!(stored.matches("\"triangles\"").count(), 1, "{name}");
        assert_eq!(serde_json::from_str::<Sections>(&stored).unwrap(), sections, "{name}");
        // The highest layer over each point tops out on the surface itself, the ridge included:
        // the leaves bend along it, but for the lattice their corners lie on, a nanometre.
        for p in [[0.7, 0.4], [2.3, 4.1], [3.9, 7.6], [6.1, 2.2], [9.6, 7.7], [4.95, 3.3], [5.0, 6.0]] {
            let (z, layer) = top_at(&sections, p).unwrap();
            let off = (z - top(p[0], p[1])).abs();
            assert!(off <= 0.3 * GRID_MM, "{name} at {p:?}: layer {layer} tops at {z}, {off} off");
        }
        let (thin, thick, _) = bounds_of(&sections, [0.0, 0.0], [10.0, 8.0]);
        assert!(thin >= 0.1 - 1e-9 && thick <= 0.3 + 1e-9, "{name}: slabs {thin} to {thick} mm");
        // Between neighbouring corners a leaf leans at most tan 20°; across a triangle, where its
        // edges disagree, by the secant of half its largest angle more.
        let lean = steepest_lean(&sections, 20f64.to_radians().tan());
        assert!(lean <= 1.0 + 1e-9, "{name}: a leaf leans {lean} times its bound");
        let v = filled(&sections, [0.0, 0.0], [10.0, 8.0], 0.05);
        assert!((v - volume).abs() < 2e-3 * volume, "{name}: the layers fill {v} mm³ of {volume}");
    }
}

/// The mesh turned by `angle` about the origin, mirrored first across the x axis when `mirror`.
fn turned(mesh: &Mesh, angle: f64, mirror: bool) -> Mesh {
    let (c, s) = (angle.cos(), angle.sin());
    let mut out = mesh.clone();
    for body in &mut out.bodies {
        for v in &mut body.vertices {
            let y = if mirror { -v[1] } else { v[1] };
            *v = [c * v[0] - s * y, s * v[0] + c * y, v[2]];
        }
        if mirror {
            body.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }
    }
    out
}

#[test]
fn turned_or_mirrored_the_leaves_turn_with_the_solid() {
    let mesh = Mesh {
        bodies: vec![height_field("gable", [0.0, 0.0], [10.0, 8.0], 10, |x, _| 4.0 - 0.15 * (x - 5.0).abs())],
        ..Mesh::default()
    };
    let sections = section(&mesh, &bent()).unwrap();
    for (angle, mirror) in [(37f64, false), (110f64, true)] {
        let (c, s) = (angle.to_radians().cos(), angle.to_radians().sin());
        let map = |p: [f64; 2]| {
            let y = if mirror { -p[1] } else { p[1] };
            [c * p[0] - s * y, s * p[0] + c * y]
        };
        let other = section(&turned(&mesh, angle.to_radians(), mirror), &bent()).unwrap();
        assert_eq!(other.layers.len(), sections.layers.len());
        assert_eq!(other.unsplit_cells, 0);
        // Every leaf, at every point, a micrometre's rounding apart at most.
        let mut worst = 0.0f64;
        let mut at_worst = (0, [0.0; 2], 0.0, 0.0);
        let pairs = sections.layers.iter().zip(tops(&sections)).zip(other.layers.iter().zip(tops(&other)));
        for ((a, top_a), (b, top_b)) in pairs {
            for j in 0..=32 {
                for i in 0..=40 {
                    let p = [-1.0 + 12.0 * i as f64 / 40.0, -1.0 + 10.0 * j as f64 / 32.0];
                    let d = (top_a.at(p) - top_b.at(map(p))).abs();
                    if d > worst {
                        worst = d;
                        at_worst = (a.index, p, top_a.at(p), top_b.at(map(p)));
                    }
                }
            }
            let area = |l: &Layer| layercraft_geom::area(&l.region);
            assert!((area(a) - area(b)).abs() < 1e-3 * area(a).max(1.0), "layer {}", a.index);
        }
        assert!(
            worst < 1e-5,
            "turned {angle}°, mirrored {mirror}: the leaves differ by {worst} mm at {at_worst:?}"
        );
    }
}

/// How far, at worst, the leaf below stands above the nozzle's flat face, of radius `face`,
/// printing each layer: over the face's disc about points of the rectangle, every corner of the
/// leaves' triangles and a point inside each.
fn face_contact(sections: &Sections, lo: [f64; 2], hi: [f64; 2], face: f64) -> f64 {
    let mut worst = f64::NEG_INFINITY;
    let inside = |p: [f64; 2]| p[0] >= lo[0] && p[0] <= hi[0] && p[1] >= lo[1] && p[1] <= hi[1];
    let tops = tops(sections);
    for (pair, warps) in sections.layers.windows(2).zip(tops.windows(2)) {
        let ((below, layer), (under, over)) = ((&pair[0], &pair[1]), (&warps[0], &warps[1]));
        if under.is_zero() && over.is_zero() {
            continue;
        }
        let mut samples: Vec<[f64; 2]> = Vec::new();
        for warp in [under, over] {
            for (c, _) in warp.facets() {
                samples.extend(c);
                let w = [0.62, 0.29, 0.09];
                samples.push([
                    w[0] * c[0][0] + w[1] * c[1][0] + w[2] * c[2][0],
                    w[0] * c[0][1] + w[1] * c[1][1] + w[2] * c[2][1],
                ]);
            }
        }
        for x in samples.into_iter().filter(|&p| inside(p)) {
            let tip = layer.top + over.at(x);
            for r in [face, face / 2.0] {
                for k in 0..64 {
                    let phi = k as f64 * std::f64::consts::TAU / 64.0;
                    let y = [x[0] + r * phi.cos(), x[1] + r * phi.sin()];
                    worst = worst.max(below.top + under.at(y) - tip);
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
    let face = layering.foliation.as_ref().unwrap().face_reach;
    for (name, top) in surfaces {
        let mesh =
            Mesh { bodies: vec![height_field(name, [0.0, 0.0], [10.0, 8.0], 10, top)], ..Mesh::default() };
        let sections = section(&mesh, &layering).unwrap();
        assert!(sections.layers.iter().any(|l| !l.warp.is_zero()), "{name}: nothing bent");
        let contact = face_contact(&sections, [0.0, 0.0], [10.0, 8.0], face);
        assert!(contact <= 1e-9, "{name}: the leaf below stands {contact} mm above the face");
    }
}

#[test]
fn a_leaf_leans_no_more_than_the_ironing_allows() {
    // The wedge of 11.3°: followed when the face may iron 0.3 mm, not when only 0.1 mm, which
    // over the face's reach of 0.71 mm allows some 7°.
    let mesh = Mesh {
        bodies: vec![height_field("wedge", [0.0, 0.0], [10.0, 8.0], 10, |x, _| 3.0 + 0.2 * x)],
        ..Mesh::default()
    };
    let bends = |ironing: f64| {
        let mut layering = bent();
        layering.foliation.as_mut().unwrap().ironing = ironing;
        section(&mesh, &layering).unwrap().layers.iter().any(|l| !l.warp.is_zero())
    };
    // The face's reach as equilateral triangles a cell wide resolve it, at the least.
    let f = bent().foliation.unwrap();
    let spread = f.cell * f.cell / 3.0;
    let steepest = 0.1 / crate::face::reach(f.face_reach, spread, spread);
    assert!(steepest < 0.2 && steepest > 0.1, "{steepest}");
    assert!(bends(0.3) && !bends(0.1));
}

#[test]
fn neighbouring_corners_are_bound_by_the_ironing_where_it_binds_tighter_than_the_slope() {
    // Two triangles of a unit square, a leaf pinned 1 mm down at one corner: it comes back
    // towards flat along the edge to the next corner by the slope per mm, or by the ironing's
    // depth over the face's reach there where that is less. A slab thick enough that nothing
    // binds across the leaves.
    use crate::envelope::{Bounds, Graph};
    use crate::face::{Corners, Laps};
    let points = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let triangles = [[0, 1, 2], [0, 2, 3]];
    let corners = Corners::new(&points, &triangles, 0.7);
    let reach = corners.reach(0, 1);
    for (slope, ironing) in [(0.2, 0.3), (0.36, 0.1)] {
        let graph = Graph::new(&points, &triangles, &corners, slope, Some(ironing));
        let laps =
            Laps { down: 1.0, thin: 0.1, reach: corners.coarsest(), farthest: corners.farthest(), slope };
        let bounds = Bounds { graph: &graph, corners: &corners, laps, up: 1.0, count: 2 };
        let leaves = bounds.leaves(points.len(), &[(1, 0, -1.0)]).unwrap();
        let rise = leaves[1].as_ref().unwrap()[1] + 1.0;
        let bound = slope.min(ironing / reach);
        assert!((rise - bound).abs() < 1e-12, "slope {slope}, ironing {ironing}: {rise} for {bound}");
    }
    assert!(0.3 / reach > 0.2 && 0.1 / reach < 0.36, "{reach}");
}

#[test]
fn under_a_steep_top_the_slab_thickens_only_as_the_face_needs() {
    // A gable of 16.7° each way: a slab parallel under it must be thick enough that the bead
    // below, its reach uphill, stands no higher than the face, `0.3 ρ`, more than a layer; and,
    // as the triangles resolve the face, as thick as its pairs of corners ask, the same under
    // every point. The top still meets the surface.
    let layering = bent();
    let f = layering.foliation.as_ref().unwrap();
    let least = 0.3 * f.face_reach;
    assert!(least > 0.2 && least < 0.3, "{least}");
    let top = |x: f64, _: f64| 4.0 - 0.3 * (x - 5.0).abs();
    let mesh =
        Mesh { bodies: vec![height_field("gable", [0.0, 0.0], [10.0, 8.0], 10, top)], ..Mesh::default() };
    let sections = section(&mesh, &layering).unwrap();
    let mut slabs = Vec::new();
    for p in [[2.3, 4.1], [3.1, 2.6], [6.9, 5.2], [7.7, 3.3]] {
        let (z, layer) = top_at(&sections, p).unwrap();
        assert!((z - top(p[0], p[1])).abs() <= 0.3 * GRID_MM, "at {p:?}: layer {layer} tops at {z}");
        slabs.push(z - heights(&sections, p)[layer - 1]);
    }
    for slab in &slabs {
        assert!(*slab >= least - 1e-9 && *slab <= 0.3 + 1e-9, "the slab under the top is {slab} mm");
        assert!((slab - slabs[0]).abs() <= 1e-9, "the slabs under the top differ: {slabs:?}");
    }
}

#[test]
fn refining_keeps_the_mesh_closed_and_its_volume() {
    // A tilted wedge under triangles half a millimetre wide over a turned square, a few large
    // ones round it out past the mesh, every edge of the mesh crossing theirs at a slant.
    let mesh = Mesh {
        bodies: vec![height_field("wedge", [0.0, 0.0], [10.0, 8.0], 7, |x, y| 3.0 + 0.2 * x - 0.05 * y)],
        ..Mesh::default()
    };
    let (c, s) = (0.3f64.cos(), 0.3f64.sin());
    let square: Vec<[f64; 2]> = [[-3.0, -3.0], [3.0, -3.0], [3.0, 3.0], [-3.0, 3.0]]
        .iter()
        .map(|p| [5.0 + c * p[0] - s * p[1], 4.0 + s * p[0] + c * p[1]])
        .collect();
    let cover =
        layercraft_geom::union_contours(&[vec![[-2.0, -2.0], [12.0, -2.0], [12.0, 10.0], [-2.0, 10.0]]])
            .unwrap();
    let edge: Vec<[f64; 2]> = square.iter().chain(square.first()).copied().collect();
    let filled = |p: [f64; 2]| {
        layercraft_geom::encloses(square.iter().copied().zip(square.iter().cycle().skip(1).copied()), p)
    };
    let t = layercraft_geom::triangulate_evenly(&cover, &[edge], &filled, 0.5).unwrap();
    let net = crate::trace::Net::new(&t.points, &t.triangles, t.filled).unwrap();
    let refined = crate::refine::refine(&mesh, &net, [[0.0, 0.0], [10.0, 8.0]], [-1.0, 10.0]);
    assert_eq!(refined.unsplit, 0);
    let body = &refined.mesh.bodies[0];
    assert!(body.triangles.len() > 10 * mesh.bodies[0].triangles.len());
    assert!(closed(body));
    assert!((refined.mesh.volume() - mesh.volume()).abs() < 1e-9 * mesh.volume());
    // Each new triangle lies over one of the triangles: its corners' places name its corners.
    let own: std::collections::HashSet<[u32; 3]> = t
        .triangles
        .iter()
        .map(|&tri| {
            let mut s = tri;
            s.sort_unstable();
            s
        })
        .collect();
    let flat = |v: u32| [body.vertices[v as usize][0], body.vertices[v as usize][1]];
    for tri in &body.triangles {
        // A steep triangle's shadow is a segment, its centre on it but for rounding: a leaf cuts
        // it along that segment however it bends there.
        let [a, b, c] = [flat(tri[0]), flat(tri[1]), flat(tri[2])];
        let twice = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        if twice.abs() <= 1e-12 {
            continue;
        }
        let mut corners: Vec<u32> = tri.iter().flat_map(|&v| refined.places[0][v as usize].named()).collect();
        corners.sort_unstable();
        corners.dedup();
        let fits = match corners.as_slice() {
            [a, b, c] => own.contains(&[*a, *b, *c]),
            [a, b] => own.iter().any(|t| t.contains(a) && t.contains(b)),
            _ => corners.len() < 3,
        };
        assert!(fits, "a triangle over corners {corners:?}");
    }
}

/// A cap of a sphere of radius `big` over a disc of radius `disc`, in rings and spokes, with an
/// up-facing surface only: enough to read its patch.
fn cap(centre: [f64; 3], disc: f64, big: f64, rings: usize, spokes: usize) -> Body {
    let height = |r: f64| centre[2] + (big * big - r * r).sqrt() - big;
    let mut vertices = vec![[centre[0], centre[1], height(0.0)]];
    let mut triangles = Vec::new();
    for i in 1..=rings {
        let r = disc * i as f64 / rings as f64;
        for j in 0..spokes {
            let a = std::f64::consts::TAU * j as f64 / spokes as f64;
            vertices.push([centre[0] + r * a.cos(), centre[1] + r * a.sin(), height(r)]);
        }
    }
    let at = |i: usize, j: usize| (1 + (i - 1) * spokes + j % spokes) as u32;
    for j in 0..spokes {
        triangles.push([0, at(1, j), at(1, j + 1)]);
        for i in 1..rings {
            triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    Body { name: "cap".into(), vertices, triangles, ..Body::default() }
}

#[test]
fn facets_overlapping_on_the_plate_tile_it_whichever_way_it_lies() {
    // Two caps one above the other, their curved outlines crossing, and a third whose outline
    // runs along the second's for a stretch: finely cut outlines, cut differently on each side.
    let bodies = vec![
        cap([20.0, 20.0, 4.0], 11.0, 40.0, 14, 96),
        cap([26.0, 21.0, 9.0], 9.0, 34.0, 12, 80),
        cap([26.0, 21.0, 14.0], 9.0005, 34.0, 12, 110),
    ];
    let mesh = Mesh { bodies, ..Mesh::default() };
    for (angle, mirror) in [(0.0f64, false), (37.0, false), (110.0, true)] {
        let turned = turned(&mesh, angle.to_radians(), mirror);
        let shadow: Vec<[f64; 2]> =
            turned.bodies.iter().flat_map(|b| b.vertices.iter().map(|v| [v[0], v[1]])).collect();
        let patches = crate::patches::patches(&turned, 20f64.to_radians().tan()).unwrap();
        let t =
            crate::triangulation::triangulation(&patches, 0.25, 2.0, &crate::triangulation::hull(&shadow))
                .unwrap();
        let inner = &t.triangles[..t.inner];
        assert!(!inner.is_empty());
        // The net refuses a triangle not counterclockwise, and an edge used twice the same way.
        crate::trace::Net::new(&t.points, &t.triangles, t.inner).unwrap();
        // Used once, an edge lies on the outline of all the triangles, whose area is the plate's:
        // so that no gap opens between them, and no leaf is refused for bending along one.
        let mut uses: std::collections::HashMap<(u32, u32), u32> = Default::default();
        for &[a, b, c] in &t.triangles {
            for (p, q) in [(a, b), (b, c), (c, a)] {
                *uses.entry((p.min(q), p.max(q))).or_default() += 1;
            }
        }
        assert!(uses.values().all(|&n| n <= 2), "{angle}°: an edge under three triangles");
        // The edges used once make one ring, that of the plate: any other is a hole.
        let mut next: std::collections::HashMap<u32, Vec<u32>> = Default::default();
        for (&(p, q), _) in uses.iter().filter(|(_, n)| **n == 1) {
            next.entry(p).or_default().push(q);
            next.entry(q).or_default().push(p);
        }
        let Some(&start) = next.keys().next() else { panic!("{angle}°: no outline") };
        let (mut seen, mut open) = (std::collections::HashSet::from([start]), vec![start]);
        while let Some(p) = open.pop() {
            for &q in next.get(&p).into_iter().flatten() {
                if seen.insert(q) {
                    open.push(q);
                }
            }
        }
        assert_eq!(seen.len(), next.len(), "{angle}°: a hole among the triangles");
    }
}
