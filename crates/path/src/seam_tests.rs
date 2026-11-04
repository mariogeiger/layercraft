//! Tests of where loops start and which way they run: the seams follow the part, not the plate.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use crate::tests::{bracket, paths_of, placed, slab_beads};
use layercraft_geom::{polyline_length, signed_area};
use layercraft_mesh::Mesh;

#[test]
fn closed_beads_start_and_run_where_the_moved_part_has_them() {
    // Each loop of the turned part starts where the moved loop does and runs the way its image
    // does: under a mirror the loops run the other way round.
    let part = bracket();
    let here = slab_beads(&part);
    for (angle, mirror, shift) in [(37.0, false, [12.5, -7.0]), (110.0, true, [-30.0, 4.0])] {
        let (g, outline) = placed(&part, angle, mirror, shift);
        let there = slab_beads(&outline);
        for (a, b) in here.layers.iter().zip(&there.layers) {
            for loop_ in a.paths.iter().filter(|p| p.closed && p.points.len() > 2) {
                let image: Vec<Point> = loop_.points.iter().map(|&p| g(p)).collect();
                let close = |u: Point, v: Point| polyline_length(&[u, v]) < 1e-3;
                let found = b.paths.iter().any(|m| {
                    m.closed
                        && m.role == loop_.role
                        && close(m.points[0], image[0])
                        && signed_area(&m.points) * signed_area(&image) > 0.0
                });
                assert!(
                    found,
                    "layer {}, {angle}°: {:?} loop from {:?} not found",
                    a.index, loop_.role, image[0]
                );
            }
        }
    }
}

/// The first outer wall of each layer of `paths`.
fn seams_of(paths: &Paths) -> Vec<&Path> {
    paths.layers.iter().filter_map(|l| l.paths.iter().find(|p| p.role == Role::OUTER_WALL)).collect()
}

#[test]
fn a_notch_gives_the_outer_wall_a_concave_seam_on_every_layer() {
    // A 20 mm slab with a notch 4 mm wide and 3 deep in one side: the seam goes to one of the
    // notch's two inner corners, the same on every layer.
    let outline = [
        [0.0, 0.0],
        [8.0, 0.0],
        [8.0, 3.0],
        [12.0, 3.0],
        [12.0, 0.0],
        [20.0, 0.0],
        [20.0, 20.0],
        [0.0, 20.0],
    ];
    let paths = slab_beads(&outline);
    let seams: Vec<Point> = seams_of(&paths).iter().map(|p| p.points[0]).collect();
    let corners = [[8.0, 3.0], [12.0, 3.0]];
    let near = |s: &Point| corners.iter().any(|c| polyline_length(&[*s, *c]) < 0.3);
    assert!(seams.iter().all(near), "{seams:?}");
    assert!(seams.windows(2).all(|w| w[0] == w[1]), "{seams:?}");
}

#[test]
fn a_cylinders_seams_line_up_at_the_far_end_of_its_axis() {
    use crate::frame::Frame;
    use crate::solids::Solids;
    let ring: Vec<Point> = (0..64)
        .map(|k| {
            let t = std::f64::consts::TAU * f64::from(k) / 64.0;
            [10.0 * t.cos(), 10.0 * t.sin()]
        })
        .collect();
    let mesh = Mesh { bodies: vec![layercraft_mesh::solids::prism("c", &ring, 0.0, 2.0)], ..Mesh::default() };
    let (regions, paths) = paths_of(&mesh);
    let seams: Vec<Point> = seams_of(&paths).iter().map(|p| p.points[0]).collect();
    assert!(seams.windows(2).all(|w| w[0] == w[1]), "{seams:?}");
    let solids = Solids::of(&regions.layers, layercraft_region::RegionLayer::outline).unwrap();
    let frame: Frame = solids.frame_at(0, [0.0, 0.0]);
    let far = |p: &Point| p[0] * frame.axis()[0] + p[1] * frame.axis()[1];
    // The wall's own farthest point along the axis, the wall lying inside the outline.
    let wall = seams_of(&paths)[0];
    let best = wall.points.iter().map(far).fold(f64::MIN, f64::max);
    assert!((far(&seams[0]) - best).abs() < 1e-3, "{:?} against {best}", seams[0]);
}

/// A prism over `outline` with a prism-shaped hole through it over `hole`.
fn holed_prism(outline: &[Point], hole: &[Point], z1: f64) -> layercraft_mesh::Body {
    let mut body = layercraft_mesh::solids::prism("h", outline, 0.0, z1);
    let mut inner = layercraft_mesh::solids::prism("h", hole, 0.0, z1);
    let base = body.vertices.len() as u32;
    body.vertices.append(&mut inner.vertices);
    body.triangles.extend(inner.triangles.iter().map(|t| [t[0] + base, t[2] + base, t[1] + base]));
    body
}

/// Whether every outer-wall loop of `here`, mapped by `g`, starts where a loop of `there` does.
fn outer_walls_start_alike(here: &Paths, there: &Paths, g: &dyn Fn(Point) -> Point) -> bool {
    here.layers.iter().zip(&there.layers).all(|(a, b)| {
        a.paths.iter().filter(|p| p.role == Role::OUTER_WALL).all(|p| {
            let start = g(p.points[0]);
            b.paths.iter().any(|m| {
                m.role == Role::OUTER_WALL && m.closed && polyline_length(&[m.points[0], start]) < 1e-3
            })
        })
    })
}

#[test]
fn islands_of_one_solid_follow_the_seams_below_whatever_their_order() {
    // Two posts under a beam make one solid: the beam's seams follow the nearest post's, however
    // the part lies on the plate.
    let part = |outline: Vec<Point>, z0: f64, z1: f64| (outline, z0, z1);
    // Nothing of the part is symmetric: its frame is settled, and so are its seams.
    let rectangle = |x: f64, w: f64, d: f64| vec![[x, 0.0], [x + w, 0.0], [x + w, d], [x, d]];
    let parts = [
        part(rectangle(0.0, 2.0, 8.0), 0.0, 1.0),
        part(rectangle(7.0, 3.0, 1.5), 0.0, 1.0),
        part(rectangle(0.0, 10.0, 8.0), 1.0, 1.6),
    ];
    let printed = |angle: f64, mirror: bool, shift: Point| {
        let bodies = parts
            .iter()
            .map(|(outline, z0, z1)| {
                let (g, moved) = placed(outline, angle, mirror, shift);
                (g, layercraft_mesh::solids::prism("b", &moved, *z0, *z1))
            })
            .collect::<Vec<_>>();
        let g = placed(&parts[0].0, angle, mirror, shift).0;
        (g, paths_of(&Mesh { bodies: bodies.into_iter().map(|(_, b)| b).collect(), ..Mesh::default() }).1)
    };
    let (_, here) = printed(0.0, false, [0.0, 0.0]);
    for (angle, mirror, shift) in [(37.0, false, [12.5, -7.0]), (110.0, true, [-30.0, 4.0])] {
        let (g, there) = printed(angle, mirror, shift);
        assert!(outer_walls_start_alike(&here, &there, &g), "{angle}°");
    }
}

#[test]
fn the_seam_round_a_small_round_hole_turns_with_its_part() {
    // A 48-gon hole 1.2 mm in radius: all its vertices are concave alike, rounding alone would
    // tell them apart.
    let slab = [[0.0, 0.0], [20.0, 0.0], [20.0, 12.0], [0.0, 12.0]];
    let hole: Vec<Point> = (0..48)
        .map(|k| {
            let t = std::f64::consts::TAU * f64::from(k) / 48.0;
            [7.0 + 1.2 * t.cos(), 5.0 + 1.2 * t.sin()]
        })
        .collect();
    let printed = |angle: f64, shift: Point| {
        let (g, outline) = placed(&slab, angle, false, shift);
        let (_, moved) = placed(&hole, angle, false, shift);
        (g, paths_of(&Mesh { bodies: vec![holed_prism(&outline, &moved, 1.0)], ..Mesh::default() }).1)
    };
    let (_, here) = printed(0.0, [0.0, 0.0]);
    assert!(here.layers[0].paths.iter().filter(|p| p.role == Role::OUTER_WALL).count() == 2);
    for angle in [7.0, 37.0, 61.0, 103.0, 151.0, 211.0, 299.0] {
        let (g, there) = printed(angle, [3.0, -2.0]);
        assert!(outer_walls_start_alike(&here, &there, &g), "{angle}°");
    }
}
