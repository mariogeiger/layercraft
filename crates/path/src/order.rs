//! Grouping a layer's beads by island and putting them in printing order.

use crate::dense::best_hatch;
use crate::gap::{gap_fill, strips};
use crate::hatch::{bridge_angle, hatch};
use crate::link::{Fill, link};
use crate::overhang::{Below, overhang};
use crate::{Kind, Path, PathError, PathParams, Role, Support, check};
use layercraft_geom::{
    Contour, GRID_MM, Point, Region, bead_spacing, clip_lines, disc_area, edges, encloses, intersection,
    offset, polyline_length, signed_area, union,
};
use layercraft_region::RegionLayer;
use layercraft_tune::{Field, centre};
use std::rc::Rc;

/// The beads of one connected piece of a layer.
pub struct Island {
    /// The outer boundary that decides which beads belong here.
    outline: Contour,
    walls: Vec<(usize, Path)>,
    /// The beads along the strips too thin for infill lines.
    gaps: Vec<Path>,
    infill: Vec<Path>,
    /// The infill areas of the part of the layer the island belongs to, for joining its lines.
    fills: Rc<Vec<Fill>>,
}

/// The innermost of `outlines` holding `p`: an island may sit in another's hole.
fn holder<'a>(outlines: impl IntoIterator<Item = &'a Contour>, p: Point) -> Option<usize> {
    outlines
        .into_iter()
        .enumerate()
        .filter(|(_, o)| encloses(edges(o), p))
        .min_by(|(_, a), (_, b)| signed_area(a).abs().total_cmp(&signed_area(b).abs()))
        .map(|(i, _)| i)
}

/// The layer's islands grouped by the parameters at their centres, each group with the part of
/// the layer in its islands. An island is a shape of the outer wall, or of the infill where
/// there is no wall; every other shape belongs to the island holding it.
pub fn parts(
    layer: &RegionLayer,
    field: &Field<PathParams>,
) -> Result<Vec<(Rc<PathParams>, RegionLayer)>, PathError> {
    let outlines: Vec<&Contour> = layer.outline().into_iter().filter_map(|shape| shape.first()).collect();
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
    let mut empty = RegionLayer { walls: vec![Vec::new(); layer.walls.len()], ..layer.clone() };
    for region in empty.infill_mut() {
        region.clear();
    }
    let mut parts = vec![empty; groups.len()];
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
    for (j, region) in layer.infill().into_iter().enumerate() {
        for shape in region {
            let slot = group(shape, !walled)
                .and_then(|g| parts.get_mut(g))
                .and_then(|part| part.infill_mut().into_iter().nth(j));
            if let Some(slot) = slot {
                slot.push(shape.clone());
            }
        }
    }
    Ok(groups.into_iter().zip(parts).collect())
}

/// A region's shapes, each with its bounding box, for asking which points it holds.
struct Areas<'a> {
    shapes: Vec<([f64; 4], &'a [Contour])>,
}

impl<'a> Areas<'a> {
    fn new(region: &'a Region) -> Areas<'a> {
        let bound = |shape: &[Contour]| {
            shape.iter().flatten().fold([f64::MAX, f64::MAX, f64::MIN, f64::MIN], |b, p| {
                [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])]
            })
        };
        Areas { shapes: region.iter().map(|shape| (bound(shape), shape.as_slice())).collect() }
    }

    /// Whether `p` lies in one of the shapes, by the even-odd rule over its contours.
    fn hold(&self, p: Point) -> bool {
        self.shapes.iter().any(|(b, shape)| {
            b[0] <= p[0]
                && p[0] <= b[2]
                && b[1] <= p[1]
                && p[1] <= b[3]
                && encloses(shape.iter().flat_map(|c| edges(c)), p)
        })
    }
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
    let loop_path = |role: Role, width: f64, contour: &Contour, outline: &Contour| {
        Path::new(role, width, true, start_near(contour.clone(), back_right(outline)))
    };
    // The outer wall's shapes are the islands; without walls, the infill's shapes are.
    let walled = !layer.walls.is_empty();
    let mut islands: Vec<Island> = layer
        .outline()
        .into_iter()
        .filter_map(|shape| {
            let outline = shape.first()?.clone();
            let walls = shape
                .iter()
                .filter(|_| walled)
                .map(|c| (0, loop_path(Role::OUTER_WALL, w(p.outer_wall_width), c, &outline)))
                .collect();
            Some(Island { outline, walls, gaps: Vec::new(), infill: Vec::new(), fills: Rc::default() })
        })
        .collect();
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
                    // Only the stretches of the loop under the layer above; a loop wholly under
                    // it stays a loop, its seam where it was.
                    Some(within) => {
                        let mut closed = path.points.clone();
                        closed.extend(path.points.first().copied());
                        let pieces = clip_lines(std::slice::from_ref(&closed), within)?;
                        let whole = polyline_length(&closed);
                        if let [piece] = pieces.as_slice()
                            // Clipping rounds each point to the lattice: a step each, at most.
                            && polyline_length(piece) + closed.len() as f64 * GRID_MM >= whole
                        {
                            island.walls.push((k, path));
                            continue;
                        }
                        for piece in pieces {
                            if piece.len() >= 2 {
                                island.walls.push((k, Path { closed: false, points: piece, ..path.clone() }));
                            }
                        }
                    }
                }
            }
        }
    }
    // How far each wall hangs over the layer below, which motion slows for.
    if let Some(below) = &layer.support {
        let support = Below::new(below, w(p.outer_wall_width).max(w(p.inner_wall_width)));
        for (_, path) in islands.iter_mut().flat_map(|island| island.walls.iter_mut()) {
            (path.points, path.overhang) = overhang(&path.points, path.closed, &support, path.width);
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
    let resting = disc_area(p.bridge_width) / h;
    let fills = [
        (Role::TOP_SURFACE, &layer.top_surface, top, bead_spacing(top, h)),
        (if first { Role::BOTTOM_SURFACE } else { Role::SOLID }, &layer.solid, solid, bead_spacing(solid, h)),
        (Role::BRIDGE, &layer.bridge, p.bridge_width, p.bridge_width),
        (Role::INTERNAL_BRIDGE, &layer.bridge_on_sparse, p.bridge_width, resting),
        (Role::SPARSE, &layer.sparse, w(p.sparse_width), sparse_spacing),
    ];
    // A strip of the laid areas narrower than two dense lines takes one bead along it, whichever
    // areas it runs through, sparse infill's too, and each stretch of it the role of its area;
    // a bridge's strands hang across whatever they span.
    let s = bead_spacing(solid, h);
    let laid = union(&union(&layer.top_surface, &layer.solid)?, &layer.sparse)?;
    let (thin, wide) = strips(&laid, s)?;
    let (tops, solids) = (Areas::new(&layer.top_surface), Areas::new(&layer.solid));
    let dense = if first { Role::BOTTOM_SURFACE } else { Role::SOLID };
    let role_at = |q: Point| {
        if tops.hold(q) {
            Role::TOP_SURFACE
        } else if solids.hold(q) {
            dense
        } else {
            Role::SOLID
        }
    };
    let gaps = gap_fill(&thin, role_at, s, h, p.resolution)?;
    let mut areas = Vec::new();
    for (role, region, width, spacing) in fills {
        let region = if role.strand() { region.clone() } else { intersection(region, &wide)? };
        if region.is_empty() {
            continue;
        }
        let reach = offset(&region, p.infill_wall_overlap * width)?;
        // Each bridge crosses its own gap the short way; every other fill follows the layer.
        let mut lines = Vec::new();
        // Over air, from edge to edge; over sparse infill, across its lines, which the layer's
        // direction already crosses at right angles.
        if role.support == Support::Air {
            // An end hangs when its strand's last bead rests nowhere on the layer below.
            let below = layer.support.as_ref().map(|below| Below::new(below, width));
            let hangs = |end: Point| below.as_ref().is_none_or(|b| b.fraction(end, width) >= 1.0);
            for shape in &reach {
                let piece = vec![shape.clone()];
                let angle = bridge_angle(&piece, spacing, solid_angle, hangs)?;
                lines.extend(hatch(&piece, spacing, angle)?);
            }
        } else if role.kind == Kind::Solid && role.support == Support::Solid && p.resolution > 0.0 {
            lines = best_hatch(&reach, spacing, h, solid_angle, p.resolution)?;
        } else {
            lines = hatch(&reach, spacing, solid_angle)?;
        }
        for line in lines {
            let (Some(&a), Some(&b)) = (line.first(), line.last()) else { continue };
            let Some(i) = home(&islands, [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]) else { continue };
            if let Some(island) = islands.get_mut(i) {
                island.infill.push(Path::new(role, width, false, line));
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
    for gap in gaps {
        let Some(&start) = gap.points.first() else { continue };
        if let Some(island) = home(&islands, start).and_then(|i| islands.get_mut(i)) {
            island.gaps.push(gap);
        }
    }
    let fills = Rc::new(areas);
    for island in &mut islands {
        island.fills = fills.clone();
    }
    Ok(islands)
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
        if let Some(end) = path.end() {
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
    lines.sort_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    if let (Some(first), Some(last)) = (lines.first(), lines.last())
        && nearest(last) < nearest(first)
    {
        across = [-across[0], -across[1]];
        lines.sort_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    }
    let span = |l: &Path| {
        l.points
            .iter()
            .map(|p| project(along, p))
            .fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)))
    };
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
            if let Some(end) = line.end() {
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
/// walls, then the beads along its strips, against them, then each infill area's lines, in
/// sweeps for a monotone area and nearest first otherwise, joined where they can be.
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
        chain(island.gaps, at, &mut out);
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
