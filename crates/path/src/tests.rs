//! Beads of analytic solids: coverage, seams, order, and the rolls' shape.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{Region, area, bead_area, bead_spacing, polyline_length};
use layercraft_mesh::Mesh;
use layercraft_mesh::solids::cuboid;
use layercraft_region::{RegionParams, SupportParams, split};
use layercraft_section::{Layering, section};
use layercraft_tune::Field;

const PATHS: PathParams = PathParams {
    outer_wall_width: 0.4,
    inner_wall_width: 0.4,
    top_surface_width: 0.4,
    solid_width: 0.4,
    sparse_width: 0.4,
    sparse_density: 0.2,
    infill_wall_overlap: 0.0,
    infill_angle: 45.0,
    first_layer_width: 0.4,
    bridge_width: 0.4,
    monotone: Vec::new(),
    // The X2D's solid infill: 250 mm/s at 10 000 mm/s².
    crossing: Crossing { top: 6.25, inside: 6.25, first_layer: 6.25 },
    resolution: 0.012,
    sparse_pattern: SparsePattern::Lines,
    support: None,
};

fn region_params() -> RegionParams {
    RegionParams {
        walls: 2,
        outer_wall_width: 0.4,
        inner_wall_width: 0.4,
        top_layers: 3,
        bottom_layers: 3,
        first_layer_shrink: 0.0,
        first_layer_width: 0.4,
        one_wall_on_top: false,
        sparse_gap: 4.0,
        bridge_width: 0.4,
        min_sparse_area: 0.0,
        supports: None,
    }
}

pub(crate) fn paths_of(mesh: &Mesh) -> (Regions, Paths) {
    let sections =
        section(mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(region_params())).unwrap();
    let paths = trace(&regions, &Field::uniform(PATHS)).unwrap();
    (regions, paths)
}

/// The segments of a role's beads that run in the hatch direction, `angle` degrees.
fn hatch_segments(paths: &[Path], role: Role, angle: f64) -> Vec<([f64; 2], [f64; 2])> {
    let (s, c) = angle.to_radians().sin_cos();
    paths
        .iter()
        .filter(|p| p.role == role)
        .flat_map(|p| p.points.windows(2).map(|w| (w[0], w[1])).collect::<Vec<_>>())
        .filter(|(a, b)| {
            let d = [b[0] - a[0], b[1] - a[1]];
            (d[0] * s - d[1] * c).abs() < 1e-6 * d[0].hypot(d[1]).max(1.0)
        })
        .collect()
}

/// The plastic a role's beads lay, spread over the layer's height `h`: the area they fill.
fn laid_area(paths: &[Path], role: Role, h: f64) -> f64 {
    paths
        .iter()
        .filter(|p| p.role == role)
        .map(|p| {
            let mut points = p.points.clone();
            points.extend(p.points.first().filter(|_| p.closed));
            polyline_length(&points) * bead_area(p.width, h) / h
        })
        .sum()
}

/// The plastic the hatch lines lay, spread over the layer's height: the area they fill. The
/// joins along the edge lie in the walls' overlap and are left out.
fn filled_area(paths: &[Path], role: Role, angle: f64) -> f64 {
    let width = paths.iter().find(|p| p.role == role).map(|p| p.width).unwrap_or(0.0);
    let length: f64 =
        hatch_segments(paths, role, angle).iter().map(|(a, b)| polyline_length(&[*a, *b])).sum();
    length * bead_area(width, 0.2) / 0.2
}

#[test]
fn solid_hatch_covers_its_area() {
    let (regions, paths) =
        paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [20.0, 20.0, 3.0])], ..Mesh::default() });
    let layer = &paths.layers[0];
    let solid = area(&regions.layers[0].solid);
    // On the bed, the solid is the part's bottom surface.
    let covered = laid_area(&layer.paths, Role::BOTTOM_SURFACE, 0.2);
    assert!((covered / solid - 1.0).abs() < 0.03, "{covered} for {solid}");
}

#[test]
fn sparse_grid_has_its_density() {
    let (regions, paths) =
        paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [40.0, 40.0, 3.0])], ..Mesh::default() });
    let i = 7;
    let sparse = area(&regions.layers[i].sparse);
    let covered = filled_area(&paths.layers[i].paths, Role::SPARSE, 135.0);
    assert!((covered / sparse - 0.2).abs() < 0.02, "{}", covered / sparse);
}

#[test]
fn walls_come_inner_first_and_seams_align() {
    let (_, paths) =
        paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])], ..Mesh::default() });
    let roles: Vec<Role> = paths.layers[5].paths.iter().map(|p| p.role).take(2).collect();
    assert_eq!(roles, vec![Role::INNER_WALL, Role::OUTER_WALL]);
    let seams: Vec<Point> = paths
        .layers
        .iter()
        .map(|l| l.paths.iter().find(|p| p.role == Role::OUTER_WALL).unwrap().points[0])
        .collect();
    assert!(seams.windows(2).all(|w| w[0] == w[1]), "{seams:?}");
}

#[test]
fn separate_islands_are_printed_whole() {
    let mesh = Mesh {
        bodies: vec![cuboid("a", [0.0; 3], [5.0, 5.0, 1.0]), cuboid("b", [20.0, 0.0, 0.0], [25.0, 5.0, 1.0])],
        ..Mesh::default()
    };
    let (_, paths) = paths_of(&mesh);
    let side: Vec<bool> = paths.layers[2].paths.iter().map(|p| p.points[0][0] > 10.0).collect();
    let switches = side.windows(2).filter(|w| w[0] != w[1]).count();
    assert_eq!(switches, 1, "{side:?}");
}

#[test]
fn beads_have_no_zero_length_segments() {
    let (_, paths) =
        paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])], ..Mesh::default() });
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
    let (_, paths) =
        paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [40.0, 40.0, 3.0])], ..Mesh::default() });
    for layer in &paths.layers {
        let angle = 45.0 + if layer.index % 2 == 1 { 90.0 } else { 0.0 };
        let sparse: Vec<&Path> = layer.paths.iter().filter(|p| p.role == Role::SPARSE).collect();
        // Every long segment runs in the layer's one direction; the rest are short joins.
        let along = hatch_segments(&layer.paths, Role::SPARSE, angle).len();
        let all: usize = sparse.iter().map(|p| p.points.len() - 1).sum();
        let joins = all - along;
        assert!(joins <= along + sparse.len(), "layer {}: {joins} joins for {along} lines", layer.index);
        for p in &sparse {
            for w in p.points.windows(2) {
                let l = polyline_length(&[w[0], w[1]]);
                let is_line =
                    hatch_segments(std::slice::from_ref(*p), Role::SPARSE, angle).contains(&(w[0], w[1]));
                assert!(is_line || l <= 3.0 * 2.0 * 0.4 / 0.2 + 1e-9, "layer {}: a {l} mm join", layer.index);
            }
        }
    }
}

#[test]
fn moves_between_neighbouring_lines_stay_inside() {
    let (_, paths) =
        paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [20.0, 20.0, 3.0])], ..Mesh::default() });
    let solid: Vec<&Path> = paths.layers[1].paths.iter().filter(|p| p.role == Role::SOLID).collect();
    let outside = solid.iter().skip(1).filter(|p| !p.travel_inside).count();
    assert!(outside <= 1, "{outside} of {} moves leave the part", solid.len());
}

#[test]
fn a_bridge_crosses_its_gap_the_short_way() {
    // A slab on two posts 8 mm apart: its strands run post to post, along x, 8 mm (plus what
    // reaches into the walls), not 12 mm along y nor 11 mm at 45°.
    let mesh = Mesh {
        bodies: vec![
            cuboid("left", [0.0; 3], [4.0, 12.0, 1.0]),
            cuboid("right", [12.0, 0.0, 0.0], [16.0, 12.0, 1.0]),
            cuboid("slab", [0.0, 0.0, 1.0], [16.0, 12.0, 2.0]),
        ],
        ..Mesh::default()
    };
    let (_, paths) = paths_of(&mesh);
    let strands: Vec<&Path> = paths.layers[5].paths.iter().filter(|p| p.role == Role::BRIDGE).collect();
    assert!(!strands.is_empty());
    for s in strands {
        let (a, b) = (s.points[0], s.points[s.points.len() - 1]);
        assert!((a[1] - b[1]).abs() < 1e-6, "a strand not along x: {a:?} → {b:?}");
        // The 8 mm gap and a 0.4 mm bead's width onto each post, plus the infill overlap.
        let length = polyline_length(&s.points);
        assert!((8.7..=8.95).contains(&length), "{length}");
    }
}

#[test]
fn a_bridge_runs_from_support_to_support() {
    // A slab 6 mm wide on two posts 20 mm apart, open at its sides: across the width, 6 mm, a
    // strand would hang from nothing at both ends. Its strands run post to post, along x.
    let mesh = Mesh {
        bodies: vec![
            cuboid("left", [0.0; 3], [4.0, 6.0, 1.0]),
            cuboid("right", [24.0, 0.0, 0.0], [28.0, 6.0, 1.0]),
            cuboid("slab", [0.0, 0.0, 1.0], [28.0, 6.0, 2.0]),
        ],
        ..Mesh::default()
    };
    let (_, paths) = paths_of(&mesh);
    let strands: Vec<&Path> = paths.layers[5].paths.iter().filter(|p| p.role == Role::BRIDGE).collect();
    assert!(!strands.is_empty());
    for s in strands {
        let (a, b) = (s.points[0], s.points[s.points.len() - 1]);
        assert!((a[1] - b[1]).abs() < 1e-6, "a strand not along x: {a:?} → {b:?}");
        // The 20 mm gap and a 0.4 mm bead's width onto each post, plus the infill overlap.
        let length = polyline_length(&s.points);
        assert!((20.7..=20.95).contains(&length), "{length}");
    }
}

/// The offset across `angle` degrees of each `role` line of `paths`, with its direction along.
fn sweep_of(paths: &[Path], role: Role, angle: f64) -> Vec<(f64, f64)> {
    let (s, c) = angle.to_radians().sin_cos();
    paths
        .iter()
        .filter(|p| p.role == role)
        .filter_map(|p| Some((*p.points.first()?, *p.points.last()?)))
        .map(|(a, b)| (((a[0] + b[0]) * -s + (a[1] + b[1]) * c) / 2.0, (b[0] - a[0]) * c + (b[1] - a[1]) * s))
        .collect()
}

#[test]
fn a_monotone_surface_is_laid_in_one_sweep() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])], ..Mesh::default() };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(region_params())).unwrap();
    let monotone = PathParams { monotone: vec![Face::Top], ..PATHS };
    let top = regions.layers.len() - 1;
    let paths = trace(&regions, &Field::uniform(monotone)).unwrap();
    // The surface's lines: straight, along and as wide as its longest. Its corners, narrower than
    // its beads' crossing width, are laid round in insets, and what these leave in beads of their
    // own widths.
    let surface = paths.layers[top].paths.iter().filter(|p| p.role == Role::TOP_SURFACE);
    let longest = surface
        .clone()
        .filter(|p| p.points.len() == 2)
        .max_by(|p, q| polyline_length(&p.points).total_cmp(&polyline_length(&q.points)))
        .unwrap();
    let (a, b) = (longest.points[0], longest.points[1]);
    let angle = (b[1] - a[1]).atan2(b[0] - a[0]);
    let parallel = |p: &&Path| {
        let (c, d) = (p.points[0], *p.points.last().unwrap());
        let across = (d[0] - c[0]) * angle.sin() - (d[1] - c[1]) * angle.cos();
        p.points.len() == 2 && p.width == longest.width && across.abs() < 1e-5
    };
    let straight: Vec<Path> = surface.filter(parallel).cloned().collect();
    let lines = sweep_of(&straight, Role::TOP_SURFACE, angle.to_degrees());
    assert!(lines.len() > 10);
    // Across the surface one way, each line running back the way the last one came.
    let one_way = |s: f64| lines.windows(2).all(|w| s * (w[1].0 - w[0].0) > 0.0);
    assert!(one_way(1.0) || one_way(-1.0), "{lines:?}");
    assert!(lines.windows(2).all(|w| w[0].1 * w[1].1 < 0.0), "{lines:?}");
}

#[test]
fn each_island_is_traced_by_the_parameters_at_its_centre() {
    let mesh = Mesh {
        bodies: vec![
            cuboid("a", [0.0; 3], [10.0, 10.0, 2.0]),
            cuboid("b", [20.0, 0.0, 0.0], [30.0, 10.0, 2.0]),
        ],
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(region_params())).unwrap();
    let tuning: layercraft_tune::Tuning =
        serde_json::from_str(r#"[{"within": {"x": [15, 40]}, "set": {"path": {"sparse_density": 0.4}}}]"#)
            .unwrap();
    let paths = trace(&regions, &Field::new(PATHS, &tuning, "path").unwrap()).unwrap();
    let sparse = |lo: f64, hi: f64| -> f64 {
        paths.layers[5]
            .paths
            .iter()
            .filter(|p| p.role == Role::SPARSE && (lo..hi).contains(&p.points[0][0]))
            .map(|p| p.points.windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1])).sum::<f64>())
            .sum()
    };
    // Twice the density, twice the lines, in the same area.
    let ratio = sparse(15.0, 40.0) / sparse(-1.0, 15.0);
    assert!((ratio - 2.0).abs() < 0.25, "{ratio}");
}

#[test]
fn strands_resting_on_sparse_infill_fill_their_layer_exactly() {
    // Under the top shells of a wide slab, the first solid layer rests on sparse infill.
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [30.0, 30.0, 2.0])], ..Mesh::default() };
    let (regions, paths) = paths_of(&mesh);
    let i = regions.layers.iter().position(|l| !l.bridge_on_sparse.is_empty()).unwrap();
    assert!(regions.layers[i].bridge.is_empty());
    let strands: f64 = paths.layers[i]
        .paths
        .iter()
        .filter(|p| p.role == Role::INTERNAL_BRIDGE)
        .map(|p| polyline_length(&p.points) * std::f64::consts::PI * 0.4 * 0.4 / 4.0)
        .sum();
    // The strands' plastic is the area's slab, up to a line's share at the hatch's ends.
    let slab = area(&regions.layers[i].bridge_on_sparse) * 0.2;
    assert!((strands / slab - 1.0).abs() < 0.03, "{strands} mm³ of strands for a {slab} mm³ slab");
}

#[test]
fn an_inner_wall_under_the_layer_above_stays_a_loop_at_the_seam() {
    // With one wall on top, inner walls are drawn only under the layer above. Inside a cube the
    // layer above covers them whole: each stays a closed loop starting at the seam, so the move
    // to the outer wall is the diagonal between their corners, not a trip around the part.
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])], ..Mesh::default() };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let params = RegionParams { one_wall_on_top: true, ..region_params() };
    let regions = split(&sections, &Field::uniform(params)).unwrap();
    assert!(regions.layers[5].inner_walls_within.is_some());
    let paths = trace(&regions, &Field::uniform(PATHS)).unwrap();
    let layer = &paths.layers[5].paths;
    let (inner, outer) = (&layer[0], &layer[1]);
    assert_eq!((inner.role, outer.role), (Role::INNER_WALL, Role::OUTER_WALL));
    assert!(inner.closed);
    let step = polyline_length(&[inner.points[0], outer.points[0]]);
    let spacing = 0.4 - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    assert!(
        step < spacing * 2f64.sqrt() + 1e-6,
        "{step} mm from the inner wall's end to the outer wall's start"
    );
}

#[test]
fn a_wall_hangs_the_share_of_its_bead_past_the_layer_below() {
    // A block whose +x side steps out by `step` every 0.2 mm layer. The outer bead, 0.4 mm wide
    // with its side on the edge, hangs `step` past the layer below: the share `step / 0.4` of
    // its width, all of it once the step passes the bead. The other sides stand straight.
    for (step, share) in [(0.1, 0.25), (0.2, 0.5), (0.6, 1.0)] {
        let bodies = (0..10)
            .map(|k| {
                let z = 0.2 * k as f64;
                cuboid("step", [0.0, 0.0, z], [10.0 + step * k as f64, 10.0, z + 0.2])
            })
            .collect();
        let (_, paths) = paths_of(&Mesh { bodies, ..Mesh::default() });
        let outer = paths.layers[5].paths.iter().find(|p| p.role == Role::OUTER_WALL).unwrap();
        assert_eq!(outer.overhang.len(), outer.points.len(), "one share per segment of the loop");
        let edge = 10.0 + 5.0 * step - 0.2;
        let mut points = outer.points.clone();
        points.push(points[0]);
        for (w, &u) in points.windows(2).zip(&outer.overhang) {
            let (a, b) = (w[0], w[1]);
            let middle = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
            if (a[0] - edge).abs() < 1e-6 && (b[0] - edge).abs() < 1e-6 && (1.0..9.0).contains(&middle[1]) {
                assert!((u - share).abs() < 1e-6, "step {step}: {u} on the overhanging side");
            } else if middle[0] < edge - 1.0 {
                assert_eq!(u, 0.0, "step {step}: {u} at {middle:?}");
            }
        }
    }
}

#[test]
fn a_move_over_what_the_layer_above_covers_stays_down() {
    // A square ring, 20 mm with a 8 mm hole. Inside the part, under the layer above, the nozzle
    // goes from bead to bead without leaving: from wall to wall and on into the infill, never
    // over the hole. On the top layer, nothing covers what oozes: only the links between
    // neighbouring lines of the surface stay down.
    let mesh = Mesh {
        bodies: vec![
            cuboid("front", [0.0; 3], [20.0, 6.0, 3.0]),
            cuboid("back", [0.0, 14.0, 0.0], [20.0, 20.0, 3.0]),
            cuboid("left", [0.0, 6.0, 0.0], [6.0, 14.0, 3.0]),
            cuboid("right", [14.0, 6.0, 0.0], [20.0, 14.0, 3.0]),
        ],
        ..Mesh::default()
    };
    let (_, paths) = paths_of(&mesh);
    let over_hole = |a: Point, b: Point| {
        (0..=100).map(|k| k as f64 / 100.0).any(|t| {
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            (6.0..=14.0).contains(&p[0]) && (6.0..=14.0).contains(&p[1])
        })
    };
    let moves = |layer: &PathLayer| -> Vec<(Point, Point, bool, Role)> {
        layer
            .paths
            .windows(2)
            .map(|w| {
                let end = if w[0].closed { w[0].points[0] } else { *w[0].points.last().unwrap() };
                (end, w[1].points[0], w[1].travel_inside, w[1].role)
            })
            .collect()
    };
    let middle = moves(&paths.layers[7]);
    for &(a, b, inside, _) in &middle {
        assert!(!(inside && over_hole(a, b)), "{a:?} → {b:?} over the hole");
    }
    let down = middle.iter().filter(|m| m.2).count();
    let around = middle.iter().filter(|m| !m.2 && !over_hole(m.0, m.1)).count();
    assert_eq!(around, 0, "{down} moves stay down, {around} around the hole leave");
    let top = moves(paths.layers.last().unwrap());
    assert!(top.iter().all(|&(_, _, inside, role)| !inside || role == Role::TOP_SURFACE));
}

#[test]
fn a_dense_strip_is_laid_along_its_length() {
    // A strip 20 mm long and five beads wide, lying at 30°: lines across it would be short, all
    // ends; its insets run its length, round its ends, and tile its width.
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let at = |u: f64, v: f64| [u * c - v * s, u * s + v * c];
    let piece: Region = vec![vec![vec![at(0.0, -0.2), at(20.0, -0.2), at(20.0, 1.8), at(0.0, 1.8)]]];
    let spacing = families::Spacing { spacing: 0.4, width: 0.4, crossing: 6.25 };
    let laid = families::fill(&piece, spacing, &families::Beneath::default(), |_| None).unwrap();
    assert!(laid.lines.is_empty(), "{:?}", laid.lines);
    assert_eq!(laid.insets.len(), 2, "{:?}", laid.insets);
    for (points, closed) in &laid.insets {
        assert!(closed);
        let mut ring = points.clone();
        ring.push(ring[0]);
        for w in ring.windows(2) {
            let angle = (w[1][1] - w[0][1]).atan2(w[1][0] - w[0][0]).to_degrees().rem_euclid(180.0);
            let length = polyline_length(w);
            assert!(length < 2.0 || (angle - 30.0).abs() < 0.01, "a {length} mm stretch at {angle}°");
        }
    }
}

#[test]
fn a_strip_too_thin_for_two_lines_takes_one_bead_along_it() {
    // A square ring 10 mm across and 2 mm thick: within its walls a band about half a millimetre
    // wide runs round it. Lines across it would be short, all ends; one bead along its centre
    // line, as wide as the band is thick, fills it.
    let mesh = Mesh {
        bodies: vec![
            cuboid("front", [0.0; 3], [10.0, 2.0, 3.0]),
            cuboid("back", [0.0, 8.0, 0.0], [10.0, 10.0, 3.0]),
            cuboid("left", [0.0, 2.0, 0.0], [2.0, 8.0, 3.0]),
            cuboid("right", [8.0, 2.0, 0.0], [10.0, 8.0, 3.0]),
        ],
        ..Mesh::default()
    };
    let (regions, paths) = paths_of(&mesh);
    let i = 7;
    let band = &regions.layers[i].sparse;
    assert!(area(band) > 0.0);
    let beads: Vec<&Path> = paths.layers[i].paths.iter().filter(|p| p.role.kind != Kind::Wall).collect();
    assert!(beads.iter().all(|p| p.role == Role::SOLID), "{beads:?}");
    for bead in &beads {
        for q in &bead.points {
            let d = (q[0] - 5.0).abs().max((q[1] - 5.0).abs());
            assert!((d - 4.0).abs() < 0.02, "{q:?} off the band's centre line");
        }
    }
    let plastic: f64 = beads
        .iter()
        .map(|p| {
            let mut points = p.points.clone();
            points.extend(p.points.first().filter(|_| p.closed));
            polyline_length(&points) * bead_area(p.width, 0.2)
        })
        .sum();
    let slab = area(band) * 0.2;
    assert!((plastic / slab - 1.0).abs() < 0.03, "{plastic} mm³ of beads for a {slab} mm³ band");
}

fn dot(x: f64) -> Path {
    Path::new(Role::SOLID, 0.4, false, vec![[x, 0.0]])
}

#[test]
fn a_tour_moves_a_path_rather_than_zigzag() {
    // Nearest first goes 1, −2, 4, −8: 22 mm; the least, one way or another, is 16.
    let mut at = [0.0, 0.0];
    let mut out = Vec::new();
    tour::tour([1.0, -2.0, 4.0, -8.0].map(dot).to_vec(), &mut at, &mut out, &|_, _| true);
    let travel = out
        .iter()
        .fold((0.0, 0.0), |(sum, x): (f64, f64), p| (sum + (p.points[0][0] - x).abs(), p.points[0][0]));
    assert_eq!(travel, (16.0, at[0]));
}

#[test]
fn a_tour_goes_farther_to_keep_its_moves_hidden() {
    // The move between 1 and 2 retracts: 1, 3, 2 goes 4 mm without it, not 1, 2, 3 in 3 mm.
    let leaves = |a: Point, b: Point| (a[0] - b[0]).abs() == 1.0 && a[0].min(b[0]) == 1.0;
    let mut at = [0.0, 0.0];
    let mut out = Vec::new();
    tour::tour([1.0, 2.0, 3.0].map(dot).to_vec(), &mut at, &mut out, &|a, b| !leaves(a, b));
    let xs: Vec<f64> = out.iter().map(|p| p.points[0][0]).collect();
    assert_eq!(xs, [1.0, 3.0, 2.0]);
}

#[test]
fn a_tour_lays_open_paths_either_way_and_loops_from_their_start() {
    // Two strokes side by side: the second laid back toward the first's end.
    let stroke = |y: f64| Path::new(Role::SOLID, 0.4, false, vec![[0.0, y], [10.0, y]]);
    let mut at = [0.0, 0.0];
    let mut out = Vec::new();
    tour::tour(vec![stroke(1.0), stroke(0.0)], &mut at, &mut out, &|_, _| true);
    assert_eq!(out[0].points, [[0.0, 0.0], [10.0, 0.0]]);
    assert_eq!(out[1].points, [[10.0, 1.0], [0.0, 1.0]]);
    let mut ring = Path::new(Role::OUTER_WALL, 0.4, true, vec![[5.0, 0.0], [6.0, 1.0], [5.0, 1.0]]);
    ring.reverse();
    assert_eq!(ring.points, [[5.0, 0.0], [6.0, 1.0], [5.0, 1.0]]);
}

#[test]
fn a_reversed_bead_keeps_each_segment_overhang() {
    let mut bead = Path::new(Role::OVERHANG_WALL, 0.4, false, vec![[0.0, 0.0], [1.0, 0.0], [3.0, 0.0]]);
    bead.overhang = vec![0.0, 0.5];
    bead.reverse();
    assert_eq!(bead.points, [[3.0, 0.0], [1.0, 0.0], [0.0, 0.0]]);
    assert_eq!(bead.overhang, [0.5, 0.0]);
}

#[test]
fn a_top_on_a_bent_layer_is_hatched_along_its_fall_line() {
    use crate::families::fall_line;
    use layercraft_geom::{Grid, Relief};
    let grid = Grid { origin: [0.0, 0.0], cell: 0.5 };
    let relief =
        |f: &dyn Fn(Point) -> f64| Relief::from_fn(grid, [0, 0], [41, 41], |i, j| f(grid.vertex(i, j)));
    let piece: Region = vec![vec![vec![[5.0, 5.0], [15.0, 5.0], [15.0, 15.0], [5.0, 15.0]]]];
    // A plane falling along 30°: hatched along 30°.
    let (s, c) = 30f64.to_radians().sin_cos();
    let plane = relief(&|p| 0.2 * (c * p[0] + s * p[1]));
    assert!((fall_line(&piece, &plane).unwrap() - 30.0).abs() < 1e-9);
    // A roof, its two sides falling opposite ways along x: their axes agree, along x.
    let roof = relief(&|p| 0.2 * (10.0 - (p[0] - 10.0).abs()));
    assert!(fall_line(&piece, &roof).unwrap().abs() < 1e-9);
    // Under a degree, or flat: no fall line, the layer's own angles.
    assert_eq!(fall_line(&piece, &relief(&|p| 0.01 * p[0])), None);
    assert_eq!(fall_line(&piece, &Relief::default()), None);
}

/// A pentagon with no symmetry, 16 by 8 mm.
const PENTAGON: [Point; 5] = [[0.0, 0.0], [16.0, 0.0], [16.0, 4.0], [7.0, 8.0], [0.0, 8.0]];

/// Layers of sparse infill alone over `shape`, 0.2 mm each from z = 0.4, rolls `tilt` degrees
/// off square, the other way every millimetre: their sparse beads.
fn rolls_over(shape: &[Point], tilt: f64, layers: usize) -> Vec<Vec<Path>> {
    let layer = |k: usize| layercraft_region::RegionLayer {
        index: k,
        top: 0.2 * (k + 1) as f64,
        height: 0.2,
        sparse: vec![vec![shape.to_vec()]],
        ..Default::default()
    };
    let regions = Regions { layers: (1..=layers).map(layer).collect() };
    let params = PathParams { sparse_pattern: SparsePattern::Rolls { tilt, ply: 1.0 }, ..PATHS };
    let paths = trace(&regions, &Field::uniform(params)).unwrap();
    paths
        .layers
        .into_iter()
        .map(|l| l.paths.into_iter().filter(|p| p.role == Role::SPARSE).collect())
        .collect()
}

/// How far `p` lies from the nearest of `lines`.
fn distance_to(p: Point, lines: &[Vec<Point>]) -> f64 {
    let segment = |a: Point, b: Point| {
        let d = [b[0] - a[0], b[1] - a[1]];
        let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]).max(1e-30))
            .clamp(0.0, 1.0);
        (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
    };
    lines.iter().flat_map(|l| l.windows(2).map(|w| segment(w[0], w[1]))).fold(f64::INFINITY, f64::min)
}

/// At each end of each bead, the angle in degrees from the wall it meets to the bead, signed
/// anticlockwise from the wall's way round the shape; with how far the end lies from the wall.
fn wall_angles(beads: &[Path], shape: &[Point]) -> Vec<(f64, f64)> {
    let walls: Vec<(Point, Point)> =
        (0..shape.len()).map(|k| (shape[k], shape[(k + 1) % shape.len()])).collect();
    let mut out = Vec::new();
    for bead in beads {
        let n = bead.points.len();
        for (end, inner) in [
            (bead.points[0], bead.points[1.min(n - 1)]),
            (bead.points[n - 1], bead.points[n.saturating_sub(2)]),
        ] {
            let (a, b) = *walls
                .iter()
                .min_by(|x, y| {
                    distance_to(end, &[vec![x.0, x.1]]).total_cmp(&distance_to(end, &[vec![y.0, y.1]]))
                })
                .unwrap();
            let gap = distance_to(end, &[vec![a, b]]);
            let (wall, into) =
                ((b[1] - a[1]).atan2(b[0] - a[0]), (inner[1] - end[1]).atan2(inner[0] - end[0]));
            out.push((((into - wall).to_degrees() + 360.0) % 360.0, gap));
        }
    }
    out
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

#[test]
fn rolls_run_from_wall_to_wall_square_to_them_and_lay_the_lines_plastic() {
    let layers = rolls_over(&PENTAGON, 0.0, 2);
    let beads = &layers[0];
    assert!(beads.len() >= 5, "{} strokes", beads.len());
    // Every stroke ends on a wall: no loose end inside.
    let angles = wall_angles(beads, &PENTAGON);
    assert!(angles.iter().all(|&(_, gap)| gap < 0.01), "{angles:?}");
    // Square to the walls, but where a corner bends them round.
    let off_square: Vec<f64> = angles.iter().map(|&(a, _)| (a - 90.0).abs()).collect();
    assert!(median(off_square.clone()) < 8.0, "{off_square:?}");
    // As much plastic as straight lines at the spacing: the area over it.
    let spacing = (0.4 - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4)) / 0.2;
    let length: f64 = beads.iter().map(|b| polyline_length(&b.points)).sum();
    let lines = area(&vec![vec![PENTAGON.to_vec()]]) / spacing;
    assert!((length / lines - 1.0).abs() < 0.12, "{length} mm of rolls for {lines} mm of lines");
}

#[test]
fn tilted_rolls_rest_on_those_below_and_turn_the_other_way_each_ply() {
    // Layers at z = 0.4 … 1.4: the first three below a millimetre, the rest above.
    let layers = rolls_over(&PENTAGON, 30.0, 6);
    let tilt = |beads: &[Path]| median(wall_angles(beads, &PENTAGON).iter().map(|&(a, _)| a).collect());
    let (first, second) = (tilt(&layers[0]), tilt(&layers[4]));
    assert!((first - 60.0).abs() < 10.0 || (first - 120.0).abs() < 10.0, "{first}°");
    assert!(((first - 90.0) + (second - 90.0)).abs() < 12.0, "{first}° then {second}°");
    // Within a ply, a layer's lines lie on those below, to half a bead.
    let below: Vec<Vec<Point>> = layers[1].iter().map(|b| b.points.clone()).collect();
    let points: Vec<Point> = layers[2].iter().flat_map(|b| b.points.clone()).collect();
    let resting = points.iter().filter(|&&p| distance_to(p, &below) <= 0.2).count();
    assert!(resting as f64 >= 0.95 * points.len() as f64, "{resting} of {} points", points.len());
}

#[test]
fn rolls_turn_and_move_with_their_part() {
    let (s, c) = 40f64.to_radians().sin_cos();
    let moved = |p: Point| [30.0 + c * p[0] - s * p[1], -5.0 + s * p[0] + c * p[1]];
    let back = |p: Point| [c * (p[0] - 30.0) + s * (p[1] + 5.0), -s * (p[0] - 30.0) + c * (p[1] + 5.0)];
    let shape: Vec<Point> = PENTAGON.iter().map(|&p| moved(p)).collect();
    let (here, there) = (rolls_over(&PENTAGON, 30.0, 2), rolls_over(&shape, 30.0, 2));
    let lines: Vec<Vec<Point>> = here[1].iter().map(|b| b.points.clone()).collect();
    let length = |beads: &[Path]| beads.iter().map(|b| polyline_length(&b.points)).sum::<f64>();
    assert!(
        (length(&here[1]) - length(&there[1])).abs() < 1e-3,
        "{} against {}",
        length(&here[1]),
        length(&there[1])
    );
    for p in there[1].iter().flat_map(|b| b.points.iter()) {
        assert!(distance_to(back(*p), &lines) < 1e-3, "{p:?} off the lines turned back");
    }
}

#[test]
fn supports_are_lines_stacked_under_a_cantilever_and_dense_on_top() {
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
    let supports = SupportParams {
        max_bridge: None,
        top_gap: 0.2,
        bottom_gap: 0.2,
        xy_gap: 0.35,
        interface_layers: 2,
        on_plate_only: false,
    };
    let regions =
        split(&sections, &Field::uniform(RegionParams { supports: Some(supports), ..region_params() }))
            .unwrap();
    // Without support lines set, the supports cannot be laid.
    assert!(trace(&regions, &Field::uniform(PATHS)).is_err());
    let lines = SupportLines { width: 0.4, gap: 2.0, interface_gap: 0.4 };
    let paths = trace(&regions, &Field::uniform(PathParams { support: Some(lines), ..PATHS })).unwrap();
    let roles = |k: usize, role: Role| paths.layers[k].paths.iter().filter(|p| p.role == role).count();
    // The body stands in lines at the layer's angle, the same on every layer; the two layers
    // above are its interface, across them; the two layers under the bar are the gap, air under
    // the bar's first strands, which hang 0.4 mm.
    let pitch = bead_spacing(0.4, 0.2) + 2.0;
    for k in [1, 20, 45] {
        assert!(roles(k, Role::SUPPORT) > 0 && roles(k, Role::SUPPORT_INTERFACE) == 0, "layer {k}");
        let laid = hatch_segments(&paths.layers[k].paths, Role::SUPPORT, 45.0);
        let length: f64 = laid.iter().map(|(a, b)| polyline_length(&[*a, *b])).sum();
        // The lines fill the two 7.65 × 4 mm columns, their beads' sides inside, at their pitch,
        // to within a line's ends.
        let area = 2.0 * (7.65 - 0.4) * (4.0 - 0.4);
        assert!((length * pitch - area).abs() < 0.15 * area, "layer {k}: {length} mm of lines");
        // Laid either way, in any order: the same lines.
        let unordered = |mut segments: Vec<([f64; 2], [f64; 2])>| {
            let mut ends: Vec<[[f64; 2]; 2]> =
                segments.drain(..).map(|(a, b)| if a < b { [a, b] } else { [b, a] }).collect();
            ends.sort_by(|x, y| x.partial_cmp(y).unwrap());
            ends
        };
        let stacked = hatch_segments(&paths.layers[20].paths, Role::SUPPORT, 45.0);
        assert_eq!(unordered(laid), unordered(stacked), "layer {k}");
    }
    for k in [46, 47] {
        assert!(roles(k, Role::SUPPORT) == 0 && roles(k, Role::SUPPORT_INTERFACE) > 0, "layer {k}");
        assert!(!hatch_segments(&paths.layers[k].paths, Role::SUPPORT_INTERFACE, 135.0).is_empty());
    }
    for k in [48, 49] {
        assert_eq!(roles(k, Role::SUPPORT) + roles(k, Role::SUPPORT_INTERFACE), 0, "layer {k}");
    }
    // Every support line stays 0.35 mm off the stem, less its half width.
    for layer in &paths.layers {
        for path in layer.paths.iter().filter(|p| p.role.kind == Kind::Support) {
            assert!(path.points.iter().all(|p| p[0].abs() >= 2.35 + 0.2 - 1e-3 || p[1].abs() > 2.0));
        }
    }
}

#[test]
fn ridges_along_either_axis_lay_their_lines_along_themselves() {
    // A plate under a cross of ridges 3 mm wide, one along x and one along y, as on a game board.
    // Within its walls each ridge holds a band four beads wide, which hatching at one angle for
    // the whole cross would lay along one ridge and across the other, in dots.
    let mesh = Mesh {
        bodies: vec![
            cuboid("plate", [0.0; 3], [30.0, 30.0, 1.0]),
            cuboid("along x", [2.0, 13.5, 1.0], [28.0, 16.5, 2.0]),
            cuboid("along y", [13.5, 2.0, 1.0], [16.5, 28.0, 2.0]),
        ],
        ..Mesh::default()
    };
    let (_, paths) = paths_of(&mesh);
    for layer in &paths.layers[7..10] {
        let dense: Vec<&Path> = layer.paths.iter().filter(|p| p.role.kind == Kind::Solid).collect();
        assert!(!dense.is_empty(), "layer {}", layer.index);
        for bead in dense {
            let mut points = bead.points.clone();
            points.extend(bead.points.first().filter(|_| bead.closed));
            for w in points.windows(2) {
                let (a, b) = (w[0], w[1]);
                let middle = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
                let (dx, dy) = ((b[0] - a[0]).abs(), (b[1] - a[1]).abs());
                // Round the junction and the ridges' ends, the insets turn.
                let (x, y) = (middle[0], middle[1]);
                let junction = (x - 15.0).abs().max((y - 15.0).abs()) < 1.5;
                let end = x.min(y) < 3.5 || x.max(y) > 26.5;
                if polyline_length(w) < 0.5 || junction || end {
                    continue;
                }
                let along = if (middle[1] - 15.0).abs() < 1.5 { dy } else { dx };
                assert!(along < 1e-6, "layer {}: {a:?} → {b:?} across its ridge", layer.index);
            }
        }
    }
}

/// A part with a thick body, a finger thinner than its beads' crossing width along its foot, a
/// thinner arm out of its side and a bite out of its top: every kind of dense area, and no
/// symmetry.
pub(crate) fn bracket() -> Vec<Point> {
    let mut outline = vec![
        [0.0, 0.0],
        [60.0, 0.0],
        [60.0, 3.0],
        [25.0, 3.0],
        [25.0, 6.0],
        [60.0, 6.0],
        [60.0, 30.0],
        [75.0, 30.0],
        [75.0, 32.5],
        [60.0, 32.5],
        [60.0, 40.0],
        [23.0, 40.0],
    ];
    outline.extend((1..32).map(|k| {
        let t = -std::f64::consts::PI * k as f64 / 32.0;
        [15.0 + 8.0 * t.cos(), 40.0 + 8.0 * t.sin()]
    }));
    outline.extend([[7.0, 40.0], [0.0, 40.0]]);
    outline
}

/// The beads of a slab a millimetre thick over `outline`, all of it dense.
pub(crate) fn slab_beads(outline: &[Point]) -> Paths {
    let mesh =
        Mesh { bodies: vec![layercraft_mesh::solids::prism("b", outline, 0.0, 1.0)], ..Mesh::default() };
    paths_of(&mesh).1
}

/// The map of the plate that mirrors (across the x axis), turns by `angle` degrees and moves by
/// `shift`, and the outline it takes `part` to, still anticlockwise.
pub(crate) fn placed(
    part: &[Point],
    angle: f64,
    mirror: bool,
    shift: Point,
) -> (impl Fn(Point) -> Point, Vec<Point>) {
    let (s, c) = angle.to_radians().sin_cos();
    let g = move |p: Point| {
        let p = if mirror { [p[0], -p[1]] } else { p };
        [c * p[0] - s * p[1] + shift[0], s * p[0] + c * p[1] + shift[1]]
    };
    let mut outline: Vec<Point> = part.iter().map(|&p| g(p)).collect();
    if mirror {
        outline.reverse();
    }
    (g, outline)
}

mod directions;

#[test]
fn a_solid_is_what_holds_together_and_its_frame_mirrors_with_it() {
    use crate::frame::Frame;
    use crate::solids::Solids;
    use layercraft_region::RegionLayer;
    // Two posts under one beam stand apart on the lower layers but make one solid; a block
    // beside them is a solid of its own.
    let bodies = vec![
        cuboid("post", [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
        cuboid("post", [8.0, 0.0, 0.0], [10.0, 2.0, 1.0]),
        cuboid("beam", [0.0, 0.0, 1.0], [10.0, 2.0, 1.6]),
        cuboid("block", [20.0, 0.0, 0.0], [23.0, 5.0, 1.6]),
    ];
    let (regions, _) = paths_of(&Mesh { bodies, ..Mesh::default() });
    let solids = Solids::of(&regions.layers, RegionLayer::outline).unwrap();
    let at = |p: Point| solids.solid_at(0, p);
    assert!(
        at([1.0, 1.0]).is_some() && at([1.0, 1.0]) == at([9.0, 1.0]) && at([21.5, 2.5]) != at([1.0, 1.0])
    );
    assert!(at([21.5, 2.5]).is_some() && at([5.0, 1.0]).is_none());
    // An L and its mirror image: mirrored frames, of the other hand, the same in each.
    let l = vec![[0.0, 0.0], [6.0, 0.0], [6.0, 1.0], [1.0, 1.0], [1.0, 3.0], [0.0, 3.0]];
    let mirrored: Vec<Point> = l.iter().rev().map(|p| [p[0], -p[1]]).collect();
    let frame = |outline: &[Point]| {
        let region = layercraft_geom::union_contours(&[outline.to_vec()]).unwrap();
        Frame::of(std::iter::once((&region, 1.0)))
    };
    let (f, g) = (frame(&l), frame(&mirrored));
    assert_ne!(f.right_handed(), g.right_handed());
    for p in &l {
        let (u, v) = (f.local(*p), g.local([p[0], -p[1]]));
        assert!((u[0] - v[0]).abs() < 1e-6 && (u[1] - v[1]).abs() < 1e-6, "{p:?}: {u:?} against {v:?}");
    }
}
