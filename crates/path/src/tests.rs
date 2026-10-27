//! Beads of analytic solids: coverage, seams and order.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{area, bead_area, polyline_length};
use layercraft_mesh::Mesh;
use layercraft_mesh::solids::cuboid;
use layercraft_region::{RegionParams, split};
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
    }
}

fn paths_of(mesh: &Mesh) -> (Regions, Paths) {
    let sections = section(mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
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
    let (regions, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [20.0, 20.0, 3.0])] });
    let layer = &paths.layers[0];
    let solid = area(&regions.layers[0].solid);
    // On the bed, the solid is the part's bottom surface.
    let covered = filled_area(&layer.paths, Role::BOTTOM_SURFACE, 45.0);
    assert!((covered / solid - 1.0).abs() < 0.03, "{covered} for {solid}");
}

#[test]
fn sparse_grid_has_its_density() {
    let (regions, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [40.0, 40.0, 3.0])] });
    let i = 7;
    let sparse = area(&regions.layers[i].sparse);
    let covered = filled_area(&paths.layers[i].paths, Role::SPARSE, 135.0);
    assert!((covered / sparse - 0.2).abs() < 0.02, "{}", covered / sparse);
}

#[test]
fn walls_come_inner_first_and_seams_align() {
    let (_, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])] });
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
    let (_, paths) = paths_of(&Mesh { bodies: vec![cuboid("c", [0.0; 3], [20.0, 20.0, 3.0])] });
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
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])] };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
    let regions = split(&sections, &Field::uniform(region_params())).unwrap();
    let monotone = PathParams { monotone: vec![Face::Top], ..PATHS };
    let top = regions.layers.len() - 1;
    let angle = if top % 2 == 1 { 135.0 } else { 45.0 };
    let paths = trace(&regions, &Field::uniform(monotone)).unwrap();
    let lines = sweep_of(&paths.layers[top].paths, Role::TOP_SURFACE, angle);
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
    };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
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
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [30.0, 30.0, 2.0])] };
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
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 3.0])] };
    let sections = section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2 }).unwrap();
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
