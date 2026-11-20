//! The PLY layout, and stages drawn where their beads and sections are, against hand values.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{Carrier, Point, Warp, union_contours};
use layercraft_mesh::{Body, Mesh};
use layercraft_motion::{Arc, Bead, Limits, Move, Pass, Program};
use layercraft_path::{Path, PathPass, Paths};
use layercraft_region::{RegionPass, Regions};
use layercraft_section::{Layer, Sections};

fn square(x: f64, y: f64, side: f64) -> Vec<Point> {
    vec![[x, y], [x + side, y], [x + side, y + side], [x, y + side]]
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9 * (1.0 + a.abs().max(b.abs()))
}

fn area<const E: usize, const F: usize>(c: &Complex<0, E, F>) -> f64 {
    c.triangles
        .rows
        .iter()
        .map(|(t, _)| {
            let [a, b, p] = t.map(|i| c.points.rows[i as usize].0);
            ((b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])) / 2.0
        })
        .sum()
}

#[test]
fn a_ply_file_says_what_it_holds() {
    let mut c = Complex::new([float("t")], [int("layer"), float("width")], [int("body")]);
    c.comments.push("role: 0 outer\nwall".into());
    let ids: Vec<u32> = (0..3).map(|k| c.point([k as f64, 0.0, 0.0], [k as f64 * 0.5]).unwrap()).collect();
    c.segment([ids[0], ids[1]], [7.0, 0.42]);
    c.triangle([ids[0], ids[1], ids[2]], [2.0]);
    let mut bytes = Vec::new();
    c.write(&mut bytes).unwrap();
    let header = "ply\nformat binary_little_endian 1.0\ncomment role: 0 outer wall\n\
        element vertex 3\nproperty double x\nproperty double y\nproperty double z\nproperty double t\n\
        element edge 1\nproperty int vertex1\nproperty int vertex2\nproperty int layer\nproperty double width\n\
        element face 1\nproperty list uchar int vertex_indices\nproperty int body\nend_header\n";
    assert_eq!(&bytes[..header.len()], header.as_bytes());
    let body = &bytes[header.len()..];
    assert_eq!(body.len(), 3 * 32 + (8 + 4 + 8) + (1 + 12 + 4));
    // The third point's time, then the segment's layer and width.
    assert_eq!(f64::from_le_bytes(body[2 * 32 + 24..3 * 32].try_into().unwrap()), 1.0);
    assert_eq!(i32::from_le_bytes(body[96 + 8..96 + 12].try_into().unwrap()), 7);
    assert_eq!(f64::from_le_bytes(body[96 + 12..96 + 20].try_into().unwrap()), 0.42);
    assert_eq!(body[116], 3);
}

#[test]
fn a_triangle_on_a_missing_vertex_is_refused() {
    let body =
        Body { name: "b".into(), vertices: vec![[0.0; 3]; 3], triangles: vec![[0, 1, 3]], material: 0 };
    assert!(matches!(mesh(&Mesh { bodies: vec![body], ..Mesh::default() }), Err(ViewError::Invalid(_))));
}

#[test]
fn sections_and_regions_lie_at_mid_height_and_cover_their_area() {
    let region = union_contours(&[square(0.0, 0.0, 10.0), square(2.0, 2.0, 6.0)]).unwrap();
    let s = Sections {
        carrier: Default::default(),
        layers: vec![Layer {
            index: 3,
            top: 0.8,
            height: 0.2,
            warp: Default::default(),
            region: region.clone(),
            materials: Vec::new(),
        }],
        open_chains: 0,
        widest_crack: 0.0,
        unsplit_triangles: 0,
    };
    let c = sections(&s).unwrap();
    assert!(close(area(&c), 64.0));
    assert!(c.points.rows.iter().all(|(p, _)| close(p[2], 0.7)));
    assert!(c.triangles.rows.iter().all(|(_, v)| v == &[3.0]));

    let empty = Vec::new();
    let layer = RegionPass {
        index: 3,
        material: 0,
        top: 0.8,
        height: 0.2,
        warp: Default::default(),
        walls: vec![region.clone(), union_contours(&[square(1.0, 1.0, 8.0)]).unwrap()],
        inner_walls_within: None,
        top_surface: empty.clone(),
        solid: union_contours(&[square(3.0, 3.0, 4.0)]).unwrap(),
        bridge: empty.clone(),
        bridge_on_sparse: empty.clone(),
        sparse: empty,
        below: None,
        above: Vec::new(),
        supports: Vec::new(),
        support_interface: Vec::new(),
    };
    let c = regions(&Regions { carrier: Default::default(), passes: vec![layer] }).unwrap();
    // The outer wall's two loops and the inner wall's one, four sides each.
    let roles: Vec<f64> = c.segments.rows.iter().map(|(_, v)| v[1]).collect();
    assert_eq!(roles, [[0.0; 8].as_slice(), &[2.0; 4]].concat());
    assert!(close(area(&c), 16.0));
    assert!(c.triangles.rows.iter().all(|(_, v)| v == &[3.0, 6.0]));
}

/// A pyramid of height 1 over the square `[0, 10]²`, on four triangles about its centre.
fn pyramid() -> (Carrier, Warp, impl Fn(Point) -> f64) {
    let points = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [5.0, 5.0]];
    let carrier = Carrier::new(points, vec![[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]]).unwrap();
    let warp = Warp::new(vec![0.0, 0.0, 0.0, 0.0, 1.0]).unwrap();
    (carrier, warp, |p: Point| 1.0 - (p[0] - 5.0).abs().max((p[1] - 5.0).abs()) / 5.0)
}

#[test]
fn a_bent_layer_is_drawn_on_its_bend() {
    let (carrier, warp, peak) = pyramid();
    let layer = |index: usize, top: f64, warp: &Warp| Layer {
        index,
        top,
        height: 0.2,
        warp: warp.clone(),
        region: union_contours(&[square(1.0, 1.0, 8.0)]).unwrap(),
        materials: Vec::new(),
    };
    let flat = Warp::default();
    let s = Sections {
        carrier: carrier.clone(),
        layers: vec![layer(0, 0.2, &flat), layer(1, 0.4, &warp), layer(2, 0.6, &warp)],
        open_chains: 0,
        widest_crack: 0.0,
        unsplit_triangles: 0,
    };
    let c = sections(&s).unwrap();
    // Layer 1 stands on a flat top, layer 2 on layer 1's: its middle bends half as much, then all.
    for (t, v) in &c.triangles.rows {
        let [a, b, p] = t.map(|i| c.points.rows[i as usize].0);
        let (plane, share) = match v[0] as usize {
            0 => (0.1, 0.0),
            1 => (0.3, 0.5),
            _ => (0.5, 1.0),
        };
        for q in [a, b, p] {
            assert!(close(q[2], plane + share * peak([q[0], q[1]])), "{q:?} on layer {}", v[0]);
        }
        // Inside each triangle too: none cuts across the pyramid's ridges.
        let middle = [(a[0] + b[0] + p[0]) / 3.0, (a[1] + b[1] + p[1]) / 3.0];
        let z = (a[2] + b[2] + p[2]) / 3.0;
        assert!(close(z, plane + share * peak(middle)), "{middle:?} on layer {}", v[0]);
    }
    assert!(close(area(&c), 3.0 * 64.0), "{}", area(&c));

    let bead = |points: Vec<Point>, closed: bool| Path {
        role: Role::OUTER_WALL,
        width: 0.42,
        closed,
        points,
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let p = Paths {
        carrier: carrier.clone(),
        passes: vec![PathPass {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: warp.clone(),
            paths: vec![bead(vec![[0.0, 5.0], [10.0, 5.0]], false), bead(square(3.0, 3.0, 4.0), true)],
            material: 0,
        }],
    };
    let c = paths(&p).unwrap();
    // The first bead climbs to the summit and down, cut there; the ring lies level at 0.6, cut
    // at its corners, where it crosses the ridges, and closed.
    let on = |q: [f64; 3]| close(q[2], 0.3 + peak([q[0], q[1]]));
    assert!(c.points.rows.iter().all(|(q, _)| on(*q)));
    assert!(c.points.rows.iter().any(|(q, _)| close(q[2], 1.3)), "the summit is drawn");
    let lengths: f64 = c
        .segments
        .rows
        .iter()
        .map(|(e, _)| {
            let [a, b] = e.map(|i| c.points.rows[i as usize].0);
            (a[0] - b[0]).hypot(a[1] - b[1])
        })
        .sum();
    assert!(close(lengths, 10.0 + 16.0), "{lengths}");

    // A wall of the regions lies on the same middle as the section.
    let r = Regions {
        carrier,
        passes: vec![RegionPass {
            index: 1,
            material: 0,
            top: 0.4,
            height: 0.2,
            warp,
            walls: vec![union_contours(&[square(1.0, 1.0, 8.0)]).unwrap()],
            inner_walls_within: None,
            top_surface: Vec::new(),
            solid: union_contours(&[square(2.0, 2.0, 6.0)]).unwrap(),
            bridge: Vec::new(),
            bridge_on_sparse: Vec::new(),
            sparse: Vec::new(),
            below: None,
            above: Vec::new(),
            supports: Vec::new(),
            support_interface: Vec::new(),
        }],
    };
    let c = regions(&r).unwrap();
    assert!(c.points.rows.iter().all(|(q, _)| close(q[2], 0.3 + peak([q[0], q[1]]) / 2.0)));
    assert!(close(area(&c), 36.0), "{}", area(&c));
}

#[test]
fn beads_lie_half_their_height_under_the_nozzle_and_bridges_hang_round() {
    let wall = Path {
        role: Role::OUTER_WALL,
        width: 0.42,
        closed: true,
        points: square(0.0, 0.0, 5.0),
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let strand = Path {
        role: Role::BRIDGE,
        width: 0.4,
        closed: false,
        points: vec![[0.0, 0.0], [5.0, 0.0]],
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let p = Paths {
        carrier: Default::default(),
        passes: vec![PathPass {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![wall, strand],
            material: 0,
        }],
    };
    let c = paths(&p).unwrap();
    assert_eq!(c.segments.rows.len(), 4 + 1);
    let (ends, values) = &c.segments.rows[4];
    assert_eq!(values, &[1.0, 7.0, 0.4, 0.4, 1.0]);
    assert!(close(c.points.rows[ends[0] as usize].0[2], 0.2));
    let (ends, values) = &c.segments.rows[3];
    assert_eq!(values, &[1.0, 0.0, 0.42, 0.2, 0.0]);
    // The loop closes on its first point, a layer's mid-height up.
    assert_eq!(ends[1], c.segments.rows[0].0[0]);
    assert!(close(c.points.rows[ends[1] as usize].0[2], 0.3));
}

#[test]
fn the_program_lays_beads_turns_helices_and_keeps_time() {
    let mv = |to: [f64; 3], arc: Option<Arc>, e: f64, speed: f64, bead: Option<Bead>| Move {
        to: Some(to),
        arc,
        e,
        speed,
        accel: 1000.0,
        fan: 0.0,
        bead,
        wait: 0.0,
    };
    let wall = Bead { role: Role::OUTER_WALL, width: 0.42 };
    let helix = Arc { centre: [10.0, -3.0], turn: std::f64::consts::TAU };
    let moves = vec![
        mv([0.0, 0.0, 0.2], None, 0.0, 100.0, None),
        mv([10.0, 0.0, 0.2], None, 0.5, 50.0, Some(wall)),
        mv([10.0, 0.0, 0.6], Some(helix), 0.0, 10.0, None),
        mv([20.0, 0.0, 0.6], None, 0.0, 200.0, None),
    ];
    let limits = Limits { max_accel: 10_000.0, jerk: 10.0, max_speed_xy: 500.0, max_speed_z: 20.0 };
    let pass =
        Pass { index: 0, z: 0.2, height: 0.2, warp: Default::default(), free: false, moves, material: 0 };
    let c = program(&Program { carrier: Default::default(), passes: vec![pass] }, &limits).unwrap();
    let segments = &c.segments.rows;
    // Nothing before the nozzle is known; the bead, the helix's chords, the travel.
    let moves: Vec<f64> = segments.iter().map(|(_, v)| v[1]).collect();
    let n = moves.iter().filter(|m| **m == 3.0).count();
    assert!(n >= 3);
    assert_eq!(moves.len(), n + 2);
    assert_eq!((moves[0], moves[n + 1]), (0.0, 1.0));
    let point = |i: u32| c.points.rows[i as usize];
    let (bead, values) = &segments[0];
    assert_eq!(values, &[0.0, 0.0, 0.0, 0.42, 0.2, 50.0, 0.5, 0.0]);
    assert!(bead.iter().all(|i| close(point(*i).0[2], 0.1)));
    for (ends, _) in &segments[1..=n] {
        for i in ends {
            let p = point(*i).0;
            assert!(close((p[0] - 10.0).hypot(p[1] + 3.0), 3.0));
        }
    }
    assert_eq!(point(segments[n].0[1]).0, [10.0, 0.0, 0.6]);
    // Time runs forward along every segment and from one to the next.
    let mut t = 0.0;
    for (ends, _) in segments {
        for i in ends {
            assert!(point(*i).1[0] >= t);
            t = point(*i).1[0];
        }
    }
    assert!(t > 0.0);
    // The legend: every label value named, `-1` the moves that lay no plastic.
    assert!(c.comments.iter().any(|l| l.starts_with("role: 0 outer wall, ")
        && l.ends_with(", 9 sparse infill, 10 support, 11 support interface, -1 none")));
    assert!(c.comments.contains(&"move: 0 extrude, 1 travel, 2 wipe, 3 lift".to_string()));
}

#[test]
fn the_thumbnail_shades_a_layer_alike_in_all_its_passes() {
    let wall = Bead { role: Role::OUTER_WALL, width: 0.42 };
    let pass = |index: usize, material: usize, y: f64| Pass {
        index,
        z: 0.2 * (index + 1) as f64,
        height: 0.2,
        warp: Default::default(),
        free: false,
        moves: vec![
            Move {
                to: Some([0.0, y, 0.2]),
                arc: None,
                e: 0.0,
                speed: 100.0,
                accel: 1000.0,
                fan: 0.0,
                bead: None,
                wait: 0.0,
            },
            Move {
                to: Some([10.0, y, 0.2]),
                arc: None,
                e: 0.5,
                speed: 50.0,
                accel: 1000.0,
                fan: 0.0,
                bead: Some(wall),
                wait: 0.0,
            },
        ],
        material,
    };
    // Layer 0 in two materials, then layer 1: two shades, one a layer, not one a pass.
    let program = Program {
        carrier: Default::default(),
        passes: vec![pass(0, 0, 0.0), pass(0, 1, 5.0), pass(1, 0, 10.0)],
    };
    let image = thumbnail(&program, 64);
    let mut shades: Vec<&[u8]> = image.rgba.chunks(4).filter(|px| px[3] == 255).collect();
    shades.sort();
    shades.dedup();
    assert_eq!(shades.len(), 2, "{shades:?}");
}

#[test]
fn the_script_draws_a_sketch_per_layer_and_role_within_the_layers_asked() {
    let line = |role: Role, y: f64| Path {
        role,
        width: 0.42,
        closed: false,
        points: vec![[0.0, y], [5.0, y]],
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let layer = |index: usize| PathPass {
        index,
        top: 0.2 * (index + 1) as f64,
        height: 0.2,
        warp: Default::default(),
        paths: vec![line(Role::OUTER_WALL, 0.0), line(Role::OUTER_WALL, 1.0), line(Role::SPARSE, 2.0)],
        material: 0,
    };
    let p = Paths { carrier: Default::default(), passes: vec![layer(0), layer(1), layer(2)] };
    let script = solidcraft_script(&p, 1..3);
    let commands = script["commands"].as_array().unwrap();
    let names: Vec<&str> = commands.iter().filter_map(|c| c["params"]["name"].as_str()).collect();
    assert_eq!(names, ["L001 outer wall", "L001 sparse infill", "L002 outer wall", "L002 sparse infill"]);
    // Each sketch: its lines, then finished; the layer's plane at its top.
    // Two layers, two roles each: a create and a finish per sketch, three lines a layer.
    assert_eq!(commands.len(), 2 * (2 * 2 + 3));
    assert_eq!(commands[0]["params"]["plane"]["origin"][2], 0.4);
    assert_eq!(commands[1]["command"], "sketch.line");
    assert_eq!(commands[3]["command"], "sketch.finish");
}
