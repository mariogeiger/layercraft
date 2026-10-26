//! Grouping a layer's beads by island and putting them in printing order.

use crate::hatch::{hatch, shortest_span_angle};
use crate::link::{Fill, link};
use crate::{Kind, Path, PathError, PathParams, Role, Support, check};
use layercraft_geom::{Contour, Point, Region, bead_spacing, clip_lines, offset, signed_area, union};
use layercraft_region::RegionLayer;
use layercraft_tune::{Field, centre};
use std::rc::Rc;

/// The beads of one connected piece of a layer.
pub struct Island {
    /// The outer boundary that decides which beads belong here.
    outline: Contour,
    walls: Vec<(usize, Path)>,
    infill: Vec<Path>,
    /// The infill areas of the part of the layer the island belongs to, for joining its lines.
    fills: Rc<Vec<Fill>>,
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

/// The innermost of `outlines` holding `p`: an island may sit in another's hole.
fn holder<'a>(outlines: impl IntoIterator<Item = &'a Contour>, p: Point) -> Option<usize> {
    outlines
        .into_iter()
        .enumerate()
        .filter(|(_, o)| contains(o, p))
        .min_by(|(_, a), (_, b)| signed_area(a).abs().total_cmp(&signed_area(b).abs()))
        .map(|(i, _)| i)
}

fn infill_regions(layer: &RegionLayer) -> [&Region; 5] {
    [&layer.top_surface, &layer.solid, &layer.bridge, &layer.bridge_on_sparse, &layer.sparse]
}

/// The layer's islands grouped by the parameters at their centres, each group with the part of
/// the layer in its islands. An island is a shape of the outer wall, or of the infill where
/// there is no wall; every other shape belongs to the island holding it.
pub fn parts(
    layer: &RegionLayer,
    field: &Field<PathParams>,
) -> Result<Vec<(Rc<PathParams>, RegionLayer)>, PathError> {
    let outlines: Vec<&Contour> = match layer.walls.first() {
        Some(outer) => outer.iter().filter_map(|shape| shape.first()).collect(),
        None => infill_regions(layer).into_iter().flatten().filter_map(|shape| shape.first()).collect(),
    };
    let mut groups: Vec<Rc<PathParams>> = Vec::new();
    let mut group_of = Vec::with_capacity(outlines.len());
    for outline in &outlines {
        let p = field.at(centre(outline.iter(), layer.top), check)?;
        group_of.push(match groups.iter().position(|q| Rc::ptr_eq(q, &p)) {
            Some(g) => g,
            None => {
                groups.push(p);
                groups.len() - 1
            }
        });
    }
    if groups.len() <= 1 {
        return Ok(groups.into_iter().map(|p| (p, layer.clone())).collect());
    }
    let empty = |layer: &RegionLayer| RegionLayer {
        walls: vec![Vec::new(); layer.walls.len()],
        top_surface: Vec::new(),
        solid: Vec::new(),
        bridge: Vec::new(),
        bridge_on_sparse: Vec::new(),
        sparse: Vec::new(),
        ..layer.clone()
    };
    let mut parts: Vec<RegionLayer> = groups.iter().map(|_| empty(layer)).collect();
    // An outline's own shape is its island's; any other lies strictly inside the one holding it.
    let held = |shape: &[Contour]| {
        let p = shape.first().and_then(|c| c.first()).copied().unwrap_or([0.0, 0.0]);
        holder(outlines.iter().copied(), p).and_then(|i| group_of.get(i).copied()).unwrap_or(0)
    };
    let walled = !layer.walls.is_empty();
    let mut outline = 0;
    let mut group = |shape: &[Contour], is_outline: bool| {
        if shape.is_empty() {
            None
        } else if is_outline {
            outline += 1;
            Some(group_of.get(outline - 1).copied().unwrap_or(0))
        } else {
            Some(held(shape))
        }
    };
    for (k, wall) in layer.walls.iter().enumerate() {
        for shape in wall {
            let slot =
                group(shape, k == 0).and_then(|g| parts.get_mut(g)).and_then(|part| part.walls.get_mut(k));
            if let Some(slot) = slot {
                slot.push(shape.clone());
            }
        }
    }
    let pick: [fn(&mut RegionLayer) -> &mut Region; 5] = [
        |l| &mut l.top_surface,
        |l| &mut l.solid,
        |l| &mut l.bridge,
        |l| &mut l.bridge_on_sparse,
        |l| &mut l.sparse,
    ];
    for (region, pick) in infill_regions(layer).into_iter().zip(pick) {
        for shape in region {
            if let Some(part) = group(shape, !walled).and_then(|g| parts.get_mut(g)) {
                pick(part).push(shape.clone());
            }
        }
    }
    Ok(groups.into_iter().zip(parts).collect())
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

/// Splits a layer, or a part of one, into islands, each holding its walls, the infill inside
/// them and the part's infill areas for joining their lines.
pub fn islands(layer: &RegionLayer, p: &PathParams) -> Result<Vec<Island>, PathError> {
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
                    .map(|c| (0, loop_path(Role::OUTER_WALL, w(p.outer_wall_width), c, &outline)))
                    .collect();
                Some(Island { outline, walls, infill: Vec::new(), fills: Rc::default() })
            })
            .collect(),
        None => [&layer.top_surface, &layer.solid, &layer.bridge, &layer.bridge_on_sparse, &layer.sparse]
            .into_iter()
            .flatten()
            .filter_map(|shape| {
                Some(Island {
                    outline: shape.first()?.clone(),
                    walls: Vec::new(),
                    infill: Vec::new(),
                    fills: Rc::default(),
                })
            })
            .collect(),
    };
    let home = |islands: &[Island], p: Point| -> Option<usize> {
        holder(islands.iter().map(|i| &i.outline), p).or(if islands.is_empty() { None } else { Some(0) })
    };
    for (k, wall) in layer.walls.iter().enumerate().skip(1) {
        for contour in wall.iter().flatten() {
            let Some(&first) = contour.first() else { continue };
            let Some(i) = home(&islands, first) else { continue };
            if let Some(island) = islands.get_mut(i) {
                let path = loop_path(Role::INNER_WALL, w(p.inner_wall_width), contour, &island.outline);
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
    // Bridge strands are round, a nozzle wide. Over air they hang, touching side by side. Over
    // sparse infill they rest on its lines, a solid layer like any other: spaced so that each
    // strand's section, π·w²/4, fills its spacing times the layer's height.
    let resting = std::f64::consts::PI * p.bridge_width * p.bridge_width / (4.0 * h);
    let fills = [
        (Role::TOP_SURFACE, &layer.top_surface, top, bead_spacing(top, h)),
        (if first { Role::BOTTOM_SURFACE } else { Role::SOLID }, &layer.solid, solid, bead_spacing(solid, h)),
        (Role::BRIDGE, &layer.bridge, p.bridge_width, p.bridge_width),
        (Role::INTERNAL_BRIDGE, &layer.bridge_on_sparse, p.bridge_width, resting),
        (Role::SPARSE, &layer.sparse, w(p.sparse_width), sparse_spacing),
    ];
    let mut areas = Vec::new();
    for (role, region, width, spacing) in fills {
        if region.is_empty() {
            continue;
        }
        let reach = offset(region, p.infill_wall_overlap * width)?;
        // Each bridge crosses its own gap the short way; every other fill follows the layer.
        let mut lines = Vec::new();
        // Over air, from edge to edge; over sparse infill, across its lines, which the layer's
        // direction already crosses at right angles.
        if role.support == Support::Air {
            for shape in &reach {
                let piece = vec![shape.clone()];
                let angle = shortest_span_angle(&piece, spacing, solid_angle)?;
                lines.extend(hatch(&piece, spacing, angle)?);
            }
        } else {
            lines = hatch(&reach, spacing, solid_angle)?;
        }
        for line in lines {
            let (Some(&a), Some(&b)) = (line.first(), line.last()) else { continue };
            let Some(i) = home(&islands, [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]) else { continue };
            if let Some(island) = islands.get_mut(i) {
                island.infill.push(Path { role, width, closed: false, points: line, travel_inside: false });
            }
        }
        // A role's areas join their lines as one.
        match areas.iter_mut().find(|f: &&mut Fill| f.role == role) {
            Some(fill) => fill.reach = union(&fill.reach, &reach)?,
            None => areas.push(Fill {
                role,
                reach,
                max_link: 3.0 * spacing,
                monotone: role.kind == Kind::Solid && p.monotone.contains(&role.face),
            }),
        }
    }
    let fills = Rc::new(areas);
    for island in &mut islands {
        island.fills = fills.clone();
    }
    Ok(islands)
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

/// Lays parallel lines in sweeps across their area, each line against the one before and all
/// sweeps the same way: from the side nearer the nozzle, each sweep takes the next line across
/// that overlaps the last one along their direction and lies within `reach`, and a new sweep
/// starts at the first line left. Each line starts where the last one ended.
fn sweep(mut lines: Vec<Path>, reach: f64, at: &mut Point, out: &mut Vec<Path>) {
    let Some((a, b)) = lines.first().and_then(|l| Some((*l.points.first()?, *l.points.last()?))) else {
        return chain(lines, at, out);
    };
    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
    if length <= 0.0 {
        return chain(lines, at, out);
    }
    let along = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
    let mut across = [-along[1], along[0]];
    let project = |d: [f64; 2], p: &Point| p[0] * d[0] + p[1] * d[1];
    let offset_of = |d: [f64; 2], l: &Path| {
        l.points.iter().map(|p| project(d, p)).sum::<f64>() / l.points.len().max(1) as f64
    };
    let nearest = |l: &Path| l.points.iter().map(|p| distance2(*at, *p)).fold(f64::MAX, f64::min);
    let first = lines.iter().min_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    let last = lines.iter().max_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    if let (Some(first), Some(last)) = (first, last)
        && nearest(last) < nearest(first)
    {
        across = [-across[0], -across[1]];
    }
    let span = |l: &Path| {
        l.points
            .iter()
            .map(|p| project(along, p))
            .fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)))
    };
    lines.sort_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    let mut left: Vec<Option<Path>> = lines.into_iter().map(Some).collect();
    while let Some(start) = left.iter().position(Option::is_some) {
        let mut next = Some(start);
        while let Some((current, mut line)) = next.and_then(|i| Some((i, left.get_mut(i)?.take()?))) {
            if let (Some(&s), Some(&e)) = (line.points.first(), line.points.last())
                && distance2(*at, e) < distance2(*at, s)
            {
                line.points.reverse();
            }
            let (offset, (lo, hi)) = (offset_of(across, &line), span(&line));
            if let Some(end) = end_of(&line) {
                *at = end;
            }
            out.push(line);
            next = left.iter().enumerate().skip(current + 1).find_map(|(i, l)| {
                let l = l.as_ref()?;
                let (o, (a, b)) = (offset_of(across, l), span(l));
                (o - offset > 1e-9 && o - offset <= reach && a <= hi && lo <= b).then_some(i)
            });
        }
    }
}

/// The layer's beads in printing order: island by island, nearest first; within an island the
/// walls, then each infill area's lines, in sweeps for a monotone area and nearest first
/// otherwise, joined where they can be.
pub fn sequence(mut islands: Vec<Island>, at: &mut Point) -> Vec<Path> {
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
        for fill in island.fills.iter() {
            let (mine, rest): (Vec<Path>, Vec<Path>) = infill.into_iter().partition(|p| p.role == fill.role);
            infill = rest;
            let mut ordered = Vec::new();
            if fill.monotone {
                sweep(mine, fill.max_link, at, &mut ordered);
            } else {
                chain(mine, at, &mut ordered);
            }
            out.extend(link(ordered, fill));
        }
        chain(infill, at, &mut out);
    }
    out
}
