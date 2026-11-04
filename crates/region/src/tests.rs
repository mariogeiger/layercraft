//! Region splits of analytic solids, checked against areas computed by hand.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{Region, area, difference};
use layercraft_mesh::Mesh;
use layercraft_mesh::solids::cuboid;
use layercraft_section::{Layering, section};
use layercraft_tune::Field;

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
    bridge_width: 0.4,
    min_sparse_area: 15.0,
    supports: None,
};

/// Equal up to the nanometre lattice: offsets round to it, moving a 10 mm square's area by
/// at most its perimeter times a step.
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 40.0 * layercraft_geom::GRID_MM
}

fn cube_regions() -> Regions {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])], ..Mesh::default() };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    split(&sections, &Field::uniform(PARAMS.clone())).unwrap()
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
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])], ..Mesh::default() };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    assert!(
        split(&sections, &Field::uniform(RegionParams { outer_wall_width: f64::NAN, ..PARAMS })).is_err()
    );
    assert!(split(&sections, &Field::uniform(RegionParams { walls: 1_000_000, ..PARAMS })).is_err());
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
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
    let first = &regions.layers[5];
    let s = |w: f64| w - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    let inside = 10.0 - 2.0 * (0.2 + s(0.4) / 2.0 + s(0.5));
    // The 6 mm gap, plus a 0.5 mm bead's width onto each post.
    assert!(close(area(&first.bridge), 7.0 * inside), "{}", area(&first.bridge));
    assert!(regions.layers[6].bridge.is_empty());
}

#[test]
fn a_strand_hangs_only_over_air_deeper_than_it_reaches() {
    // A plate with a 4 × 1 mm slot engraved in its bottom. A strand 0.4 mm round hangs 0.2 mm
    // below a 0.2 mm layer: over a slot one layer deep it would lie on the bed and fill it, so
    // the layer over it is solid; over a slot two layers deep it hangs.
    for (depth, hangs) in [(0.2, false), (0.4, true)] {
        let mesh = Mesh {
            bodies: vec![
                cuboid("front", [0.0; 3], [10.0, 4.5, depth]),
                cuboid("back", [0.0, 5.5, 0.0], [10.0, 10.0, depth]),
                cuboid("left", [0.0, 4.5, 0.0], [3.0, 5.5, depth]),
                cuboid("right", [7.0, 4.5, 0.0], [10.0, 5.5, depth]),
                cuboid("plate", [0.0, 0.0, depth], [10.0, 10.0, 2.0]),
            ],
            ..Mesh::default()
        };
        let sections =
            section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None })
                .unwrap();
        let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
        let over = &regions.layers[(depth / 0.2).round() as usize];
        assert_eq!(!over.bridge.is_empty(), hangs, "slot {depth} mm deep: bridge {}", area(&over.bridge));
        assert!(regions.layers.iter().all(|l| l.index == over.index || l.bridge.is_empty()));
    }
}

#[test]
fn an_inner_wall_runs_only_where_its_bead_passes() {
    // Two 10 mm squares joined by a 1.6 mm neck. Inside the outer walls the inner wall's inset
    // narrows there to 0.39 mm, under its 0.5 mm bead: the loop through the neck would run in and
    // back over its own plastic. The inner wall is two loops, one per square; the neck's band is
    // solid.
    let mesh = Mesh {
        bodies: vec![
            cuboid("left", [0.0; 3], [10.0, 10.0, 2.0]),
            cuboid("neck", [10.0, 4.2, 0.0], [14.0, 5.8, 2.0]),
            cuboid("right", [14.0, 0.0, 0.0], [24.0, 10.0, 2.0]),
        ],
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
    let layer = &regions.layers[5];
    let s = |w: f64| w - 0.2 * (1.0 - std::f64::consts::FRAC_PI_4);
    let inset = offset(&sections.layers[5].region, -(0.2 + s(0.4) / 2.0 + s(0.5) / 2.0)).unwrap();
    assert_eq!(inset.len(), 1, "the plain inset runs through the neck");
    assert_eq!(layer.walls[1].len(), 2);
    let neck = vec![vec![vec![[11.0, 4.0], [13.0, 4.0], [13.0, 6.0], [11.0, 6.0]]]];
    let filled = union(&layer.solid, &layer.sparse).unwrap();
    assert!(area(&intersection(&filled, &neck).unwrap()) > 0.5, "the neck's band is filled");
}

#[test]
fn the_first_layer_is_wider_and_the_top_has_one_wall() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [10.0, 10.0, 2.0])], ..Mesh::default() };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let params = RegionParams { first_layer_width: 0.5, one_wall_on_top: true, ..PARAMS };
    let regions = split(&sections, &Field::uniform(params.clone())).unwrap();
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
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
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
        let mesh = Mesh { bodies: vec![cuboid("post", [0.0; 3], [side, side, 3.0])], ..Mesh::default() };
        let sections =
            section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None })
                .unwrap();
        let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
        let middle = &regions.layers[7];
        assert_eq!(!middle.sparse.is_empty(), sparse, "{side} mm post");
        assert!(!middle.solid.is_empty() || sparse);
    }
}

#[test]
fn a_thin_rib_stands_on_solid() {
    // A 1 mm rib along a 20 mm block: its two walls fill it, it has no inside. The three layers
    // under it are top shells, under the rib as elsewhere: the rib stands on solid.
    let mesh = Mesh {
        bodies: vec![
            cuboid("block", [0.0; 3], [20.0, 20.0, 3.0]),
            cuboid("rib", [2.0, 9.5, 3.0], [18.0, 10.5, 4.0]),
        ],
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
    // Layers 12 to 14 (z 2.4 to 3) have the rib among their three layers above. Taken by its
    // section, the rib once left them a 16 mm² strip of sparse under it, over the 15 mm² minimum.
    for layer in &regions.layers[12..15] {
        assert!(layer.sparse.is_empty(), "layer {}: sparse {}", layer.index, area(&layer.sparse));
        assert!(!layer.solid.is_empty() || !layer.bridge_on_sparse.is_empty(), "layer {}", layer.index);
    }
    // Deeper, the whole inside is sparse again.
    assert!(area(&regions.layers[8].sparse) > 200.0);
}

#[test]
fn each_shape_splits_by_the_parameters_at_its_centre() {
    let mesh = Mesh {
        bodies: vec![
            cuboid("a", [0.0; 3], [10.0, 10.0, 2.0]),
            cuboid("b", [20.0, 0.0, 0.0], [30.0, 10.0, 2.0]),
        ],
        ..Mesh::default()
    };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let tuning: layercraft_tune::Tuning =
        serde_json::from_str(r#"[{"within": {"x": [15, 40]}, "set": {"region": {"walls": 1}}}]"#).unwrap();
    let field = Field::new(PARAMS.clone(), &tuning, "region").unwrap();
    let layer = &split(&sections, &field).unwrap().layers[4];
    // Both cubes have their outer wall; only the left one, untouched, an inner wall.
    assert_eq!(layer.walls[0].len(), 2);
    assert_eq!(layer.walls[1].len(), 1);
    assert!(layer.walls[1][0][0].iter().all(|p| p[0] < 10.0));
    // The right cube's inside starts one wall in: more sparse infill than the left one's.
    let sparse_x = |lo: f64, hi: f64| -> f64 {
        layer.sparse.iter().filter(|s| (lo..hi).contains(&s[0][0][0])).map(|s| area(&vec![s.clone()])).sum()
    };
    assert!(sparse_x(15.0, 40.0) > sparse_x(-1.0, 15.0) + 1.0);
}

/// Supports as the X2D's process sets them: 0.2 mm under and over the part, 0.35 mm beside it,
/// two interface layers, any bridge held at both ends left to bridge.
const SUPPORTS: SupportParams = SupportParams {
    max_bridge: None,
    top_gap: 0.2,
    bottom_gap: 0.2,
    xy_gap: 0.35,
    interface_layers: 2,
    on_plate_only: false,
};

fn supported(bodies: Vec<layercraft_mesh::Body>, supports: SupportParams) -> Regions {
    let sections = section(
        &Mesh { bodies, ..Mesh::default() },
        &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None },
    )
    .unwrap();
    split(&sections, &Field::uniform(RegionParams { supports: Some(supports), ..PARAMS })).unwrap()
}

/// A T: a 4 × 4 mm stem 10 mm high under a 20 × 4 mm bar, 7.8 mm past the stem's side and the
/// half bead its edge may reach on either side.
fn tee() -> Vec<layercraft_mesh::Body> {
    vec![
        cuboid("stem", [-2.0, -2.0, 0.0], [2.0, 2.0, 10.0]),
        cuboid("bar", [-10.0, -2.0, 10.0], [10.0, 2.0, 12.0]),
    ]
}

#[test]
fn a_cantilever_stands_on_supports_down_to_the_plate() {
    let regions = supported(tee(), SUPPORTS);
    // The bar's first layer is 50 (z 10 to 10.2), laid as a bridge whose round strands, 0.4 mm
    // wide, hang from z 10.2 down to 9.8; the columns' top is layer 47, at 9.6, 0.2 mm of air
    // under them. Each column runs from the stem's side, 0.35 mm off, to the bar's end:
    // 7.65 × 4 mm, twice.
    let column = 2.0 * 7.65 * 4.0;
    for layer in &regions.layers {
        let (body, interface) = (area(&layer.supports), area(&layer.support_interface));
        let (want_body, want_interface) = match layer.index {
            0..=45 => (column, 0.0),
            46 | 47 => (0.0, column),
            _ => (0.0, 0.0),
        };
        assert!(close(body, want_body), "layer {}: body {body}", layer.index);
        assert!(close(interface, want_interface), "layer {}: interface {interface}", layer.index);
        // Never inside the part, nor nearer it than the gap.
        let part = offset(&regions.layers[layer.index].walls[0], 0.2 + 0.35 - 1e-3).unwrap();
        let both = union(&layer.supports, &layer.support_interface).unwrap();
        assert!(area(&intersection(&both, &part).unwrap()) < 1e-6, "layer {}", layer.index);
    }
}

#[test]
fn a_bridge_held_at_both_ends_needs_no_support_unless_it_is_too_long() {
    // A 20 mm deck on two 4 mm pillars: its strands span the 12 mm between them, 12.4 mm long
    // with a bead's reach onto each pillar.
    let deck = || {
        vec![
            cuboid("left", [-10.0, -2.0, 0.0], [-6.0, 2.0, 10.0]),
            cuboid("right", [6.0, -2.0, 0.0], [10.0, 2.0, 10.0]),
            cuboid("deck", [-10.0, -2.0, 10.0], [10.0, 2.0, 12.0]),
        ]
    };
    let total =
        |r: &Regions| r.layers.iter().map(|l| area(&l.supports) + area(&l.support_interface)).sum::<f64>();
    assert_eq!(total(&supported(deck(), SUPPORTS)), 0.0);
    assert_eq!(total(&supported(deck(), SupportParams { max_bridge: Some(20.0), ..SUPPORTS })), 0.0);
    // Shorter than the span, or 0, which holds every strand up: the 11.3 × 4 mm between the
    // pillars, 0.35 mm off each, stands on columns, layers 0 to 47.
    for max_bridge in [Some(10.0), Some(0.0)] {
        let regions = supported(deck(), SupportParams { max_bridge, ..SUPPORTS });
        let layer = &regions.layers[20];
        assert!(close(area(&layer.supports), 11.3 * 4.0), "{max_bridge:?}: {}", area(&layer.supports));
    }
}

#[test]
fn a_column_stands_on_the_part_or_only_on_the_plate() {
    // The T over a 20 × 4 mm base 2 mm high: the columns stand on the base, 0.2 mm above it.
    let mut bodies = tee();
    bodies[0] = cuboid("stem", [-2.0, -2.0, 2.0], [2.0, 2.0, 10.0]);
    bodies.push(cuboid("base", [-10.0, -2.0, 0.0], [10.0, 2.0, 2.0]));
    let regions = supported(bodies.clone(), SUPPORTS);
    // Layers 0 to 9 are the base, 10 the gap over it; the columns start on layer 11.
    assert!(regions.layers[..11].iter().all(|l| l.supports.is_empty()));
    assert!(close(area(&regions.layers[11].supports), 2.0 * 7.65 * 4.0));
    let plate_only = supported(bodies, SupportParams { on_plate_only: true, ..SUPPORTS });
    assert!(plate_only.layers.iter().all(|l| l.supports.is_empty() && l.support_interface.is_empty()));
}

#[test]
fn bad_support_params_are_refused() {
    let mesh = Mesh { bodies: vec![cuboid("c", [0.0; 3], [1.0; 3])], ..Mesh::default() };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    for bad in [
        SupportParams { top_gap: f64::NAN, ..SUPPORTS },
        SupportParams { xy_gap: -1.0, ..SUPPORTS },
        SupportParams { max_bridge: Some(f64::INFINITY), ..SUPPORTS },
        SupportParams { interface_layers: 1_000, ..SUPPORTS },
    ] {
        assert!(split(&sections, &Field::uniform(RegionParams { supports: Some(bad), ..PARAMS })).is_err());
    }
}

#[test]
fn materials_split_into_passes_judged_against_the_whole_layer() {
    // A 10 × 10 × 2 base and a 4 × 4 inlay of another material from z = 1 to 3, set into it.
    let base = cuboid("base", [0.0; 3], [10.0, 10.0, 2.0]);
    let inlay = layercraft_mesh::Body { material: 1, ..cuboid("inlay", [2.0, 2.0, 1.0], [6.0, 6.0, 3.0]) };
    let colour = |c: &str| layercraft_mesh::Material { colour: Some(c.into()) };
    let mesh = Mesh { bodies: vec![base, inlay], materials: vec![colour("#0A2989"), colour("#FEC600")] };
    let sections =
        section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None }).unwrap();
    let regions = split(&sections, &Field::uniform(PARAMS.clone())).unwrap();
    let passes =
        |i: usize| regions.layers.iter().filter(|l| l.index == i).map(|l| l.material).collect::<Vec<_>>();
    // Below the inlay, one pass; beside it, both, each layer starting with the material the one
    // before ended with; above the base, the inlay alone.
    assert_eq!(passes(4), [0]);
    assert_eq!(passes(5), [0, 1]);
    assert_eq!(passes(6), [1, 0]);
    assert_eq!(passes(7), [0, 1]);
    assert_eq!(passes(10), [1]);
    // The base's layer under the inlay is covered by it: no top surface there.
    let under = regions.layers.iter().find(|l| l.index == 4).unwrap();
    assert!(area(&under.top_surface) < 1e-9);
    // Each pass has walls of its own: the inlay's outer wall runs 0.2 mm inside its 4 mm square,
    // the base's around both its outline and the inlay's hole.
    let side = regions.layers.iter().find(|l| l.index == 6 && l.material == 0).unwrap();
    let inlay = regions.layers.iter().find(|l| l.index == 6 && l.material == 1).unwrap();
    assert!(close(area(&inlay.walls[0]), 3.6 * 3.6));
    assert!(close(area(&side.walls[0]), 9.6 * 9.6 - 4.4 * 4.4));
    // The top of the base, beside the inlay, is a top surface; the inlay's top layers are too.
    let last_base = regions.layers.iter().find(|l| l.index == 9 && l.material == 0).unwrap();
    assert!(area(&last_base.top_surface) > 30.0);
    let last = regions.layers.last().unwrap();
    assert_eq!((last.index, last.material), (14, 1));
    assert!(area(&last.top_surface) > 0.0);
}

#[test]
fn the_supports_under_a_turned_bridge_are_the_turned_supports() {
    // The strands of a bridge too long to hang lie across its own span, on lines through its
    // own centre, not along the plate's axes: the deck turned on the plate has its supports
    // turned with it.
    let turn = 37f64.to_radians();
    let (s, c) = turn.sin_cos();
    let at = |p: [f64; 2]| [c * p[0] - s * p[1] + 12.5, s * p[0] + c * p[1] - 7.0];
    let supports = |turned: bool| {
        let post = |x0: f64, x1: f64, z1: f64, z0: f64, name: &str| {
            let outline: Vec<[f64; 2]> = [[x0, -2.0], [x1, -2.0], [x1, 2.0], [x0, 2.0]]
                .into_iter()
                .map(|p| if turned { at(p) } else { p })
                .collect();
            layercraft_mesh::solids::prism(name, &outline, z0, z1)
        };
        let bodies = vec![
            post(-10.0, -6.0, 10.0, 0.0, "left"),
            post(6.0, 10.0, 10.0, 0.0, "right"),
            post(-10.0, 10.0, 12.0, 10.0, "deck"),
        ];
        supported(bodies, SupportParams { max_bridge: Some(10.0), ..SUPPORTS }).layers[20].supports.clone()
    };
    let (flat, turned) = (supports(false), supports(true));
    let moved: Region = flat
        .iter()
        .map(|shape| shape.iter().map(|contour| contour.iter().map(|&p| at(p)).collect()).collect())
        .collect();
    let off = area(&difference(&moved, &turned).unwrap()) + area(&difference(&turned, &moved).unwrap());
    assert!(area(&turned) > 40.0 && off < 0.01 * area(&turned), "{off} mm² off of {}", area(&turned));
}
