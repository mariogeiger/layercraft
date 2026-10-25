//! Grouping a layer's beads by island and putting them in printing order.

use crate::hatch::hatch;
use crate::link::{Fill, link};
use crate::{Path, PathError, PathParams, Role};
use layercraft_geom::{Contour, Point, bead_spacing, clip_lines, offset, signed_area};
use layercraft_region::RegionLayer;

/// The beads of one connected piece of a layer.
pub struct Island {
    /// The outer boundary that decides which beads belong here.
    outline: Contour,
    walls: Vec<(usize, Path)>,
    infill: Vec<Path>,
}

fn contains(contour: &[Point], p: Point) -> bool {
    let mut inside = false;
    let n = contour.len();
    for i in 0..n {
        let (Some(a), Some(b)) = (contour.get(i), contour.get((i + n - 1) % n)) else { continue };
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

fn distance2(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// Rotates a loop to start at its vertex nearest `target`.
fn start_near(mut contour: Contour, target: Point) -> Contour {
    let best = contour
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| distance2(**a, target).total_cmp(&distance2(**b, target)))
        .map(|(i, _)| i)
        .unwrap_or(0);
    contour.rotate_left(best);
    contour
}

fn back_right(contour: &[Point]) -> Point {
    contour.iter().fold([f64::MIN, f64::MIN], |m, p| [m[0].max(p[0]), m[1].max(p[1])])
}

/// Splits a layer into islands, each holding its walls and the infill inside them, and lists
/// the layer's infill areas for joining their lines.
pub fn islands(layer: &RegionLayer, p: &PathParams) -> Result<(Vec<Island>, Vec<Fill>), PathError> {
    let first = layer.index == 0;
    let w = |width: f64| if first { p.first_layer_width } else { width };
    let loop_path = |role: Role, width: f64, contour: &Contour, outline: &Contour| Path {
        role,
        width,
        closed: true,
        points: start_near(contour.clone(), back_right(outline)),
        travel_inside: false,
    };
    // The outer wall's shapes are the islands; without walls, the infill's shapes are.
    let mut islands: Vec<Island> = match layer.walls.first() {
        Some(outer) => outer
            .iter()
            .filter_map(|shape| {
                let outline = shape.first()?.clone();
                let walls = shape
                    .iter()
                    .map(|c| (0, loop_path(Role::OuterWall, w(p.outer_wall_width), c, &outline)))
                    .collect();
                Some(Island { outline, walls, infill: Vec::new() })
            })
            .collect(),
        None => [&layer.top_surface, &layer.solid, &layer.bridge, &layer.sparse]
            .into_iter()
            .flatten()
            .filter_map(|shape| {
                Some(Island { outline: shape.first()?.clone(), walls: Vec::new(), infill: Vec::new() })
            })
            .collect(),
    };
    let home = |islands: &[Island], p: Point| -> Option<usize> {
        // The innermost outline holding the point: an island may sit in another's hole.
        let holder = islands
            .iter()
            .enumerate()
            .filter(|(_, i)| contains(&i.outline, p))
            .min_by(|(_, a), (_, b)| signed_area(&a.outline).abs().total_cmp(&signed_area(&b.outline).abs()))
            .map(|(i, _)| i);
        holder.or(if islands.is_empty() { None } else { Some(0) })
    };
    for (k, wall) in layer.walls.iter().enumerate().skip(1) {
        for contour in wall.iter().flatten() {
            let Some(&first) = contour.first() else { continue };
            let Some(i) = home(&islands, first) else { continue };
            if let Some(island) = islands.get_mut(i) {
                let path = loop_path(Role::InnerWall, w(p.inner_wall_width), contour, &island.outline);
                match &layer.inner_walls_within {
                    None => island.walls.push((k, path)),
                    // Only the stretches of the loop under the layer above.
                    Some(within) => {
                        let mut closed = path.points.clone();
                        closed.extend(path.points.first().copied());
                        for piece in clip_lines(&[closed], within)? {
                            if piece.len() >= 2 {
                                island.walls.push((k, Path { closed: false, points: piece, ..path.clone() }));
                            }
                        }
                    }
                }
            }
        }
    }
    let odd = layer.index % 2 == 1;
    let solid_angle = p.infill_angle + if odd { 90.0 } else { 0.0 };
    let h = layer.height;
    // Beads `s` apart in effect, every `S`, fill `s/S` of the area. One direction per layer,
    // turning a quarter turn each layer: two beads never cross on one layer, where a crossing
    // would lay plastic twice in one place.
    let sparse_spacing = if p.sparse_density > 0.0 {
        bead_spacing(w(p.sparse_width), h) / p.sparse_density
    } else {
        f64::INFINITY
    };
    let (top, solid) = (w(p.top_surface_width), w(p.solid_width));
    // Bridge strands hang round, a nozzle wide: they touch side by side.
    let fills = [
        (Role::TopSurface, &layer.top_surface, top, bead_spacing(top, h)),
        (Role::Solid, &layer.solid, solid, bead_spacing(solid, h)),
        (Role::Bridge, &layer.bridge, p.bridge_width, p.bridge_width),
        (Role::Sparse, &layer.sparse, w(p.sparse_width), sparse_spacing),
    ];
    let mut areas = Vec::new();
    for (role, region, width, spacing) in fills {
        if region.is_empty() {
            continue;
        }
        let reach = offset(region, p.infill_wall_overlap * width)?;
        for line in hatch(&reach, spacing, solid_angle)? {
            let (Some(&a), Some(&b)) = (line.first(), line.last()) else { continue };
            let Some(i) = home(&islands, [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]) else { continue };
            if let Some(island) = islands.get_mut(i) {
                island.infill.push(Path { role, width, closed: false, points: line, travel_inside: false });
            }
        }
        areas.push(Fill { role, reach, max_link: 3.0 * spacing });
    }
    Ok((islands, areas))
}

fn end_of(path: &Path) -> Option<Point> {
    if path.closed { path.points.first().copied() } else { path.points.last().copied() }
}

/// Orders open paths nearest first from `at`, reversing any that is nearer by its far end.
fn chain(mut paths: Vec<Path>, at: &mut Point, out: &mut Vec<Path>) {
    while !paths.is_empty() {
        let mut best = (f64::MAX, 0, false);
        for (i, path) in paths.iter().enumerate() {
            let (Some(&a), Some(&b)) = (path.points.first(), path.points.last()) else { continue };
            let (da, db) = (distance2(*at, a), if path.closed { f64::MAX } else { distance2(*at, b) });
            if da < best.0 {
                best = (da, i, false);
            }
            if db < best.0 {
                best = (db, i, true);
            }
        }
        let mut path = paths.swap_remove(best.1);
        if best.2 {
            path.points.reverse();
        }
        if let Some(end) = end_of(&path) {
            *at = end;
        }
        out.push(path);
    }
}

/// The layer's beads in printing order: island by island, nearest first; within an island the
/// walls, then each infill area's lines, joined where they can be.
pub fn sequence(mut islands: Vec<Island>, fills: &[Fill], at: &mut Point) -> Vec<Path> {
    let mut out = Vec::new();
    while !islands.is_empty() {
        let near = islands
            .iter()
            .enumerate()
            .map(|(i, island)| (i, distance2(*at, back_right(&island.outline))))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
            .unwrap_or(0);
        let mut island = islands.swap_remove(near);
        island.walls.sort_by_key(|(k, _)| std::cmp::Reverse(*k));
        let mut walls = Vec::new();
        let mut depth = None;
        for (k, path) in island.walls {
            if depth != Some(k) && !walls.is_empty() {
                chain(std::mem::take(&mut walls), at, &mut out);
            }
            depth = Some(k);
            walls.push(path);
        }
        chain(walls, at, &mut out);
        let mut infill = island.infill;
        for fill in fills {
            let (mine, rest): (Vec<Path>, Vec<Path>) = infill.into_iter().partition(|p| p.role == fill.role);
            infill = rest;
            let mut ordered = Vec::new();
            chain(mine, at, &mut ordered);
            out.extend(link(ordered, fill));
        }
        chain(infill, at, &mut out);
    }
    out
}
