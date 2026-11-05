//! Grouping a layer's beads by island and putting them in printing order.

use crate::crossing::{Cover, Laid, Segment, segments};
use crate::families::{Beneath, Spacing, fall_line, fill};
use crate::frame::{hatch_in_frames, inside};
use crate::gap::{gap_fill, strips};
use crate::overhang::{Below, overhang};
use crate::seam::{Seams, farthest_along, seamed};
use crate::solids::Solids;
use crate::tour::tour;
use crate::{Crossing, Face, Kind, Path, PathError, PathParams, Role, SparsePattern, Support, check};
use layercraft_geom::{
    Contour, GRID_MM, Point, Polyline, Region, Relief, bead_spacing, bridge_angle, clip_lines, disc_area,
    edges, encloses, hatch, intersection, offset, phase_through, polyline_length, signed_area, union,
};
use layercraft_region::RegionLayer;
use layercraft_tune::{Field, centre};
use std::rc::Rc;

/// One infill area of a layer: its role, and how far apart its lines may be swept together.
pub struct Fill {
    pub role: Role,
    pub max_link: f64,
    /// Whether its lines are laid in sweeps, each against the one before.
    pub monotone: bool,
}

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

impl Island {
    /// An island without walls, bounded by `outline`, its beads to come.
    pub(crate) fn bare(outline: Contour) -> Island {
        Island { outline, walls: Vec::new(), gaps: Vec::new(), infill: Vec::new(), fills: Rc::default() }
    }

    /// Where the island's first bead starts: its outer wall's seam; without walls, the start of
    /// its first bead.
    fn first_start(&self) -> Point {
        let outer = self.walls.iter().find(|(k, _)| *k == 0).map(|(_, p)| p);
        let first = outer.or(self.gaps.first()).or(self.infill.first());
        first.and_then(|p| p.points.first()).or(self.outline.first()).copied().unwrap_or([0.0, 0.0])
    }

    pub(crate) fn outline(&self) -> &Contour {
        &self.outline
    }

    /// Adds an infill bead.
    pub(crate) fn push(&mut self, path: Path) {
        self.infill.push(path);
    }

    /// Joins the island's lines by the areas `fills`.
    pub(crate) fn share(&mut self, fills: Rc<Vec<Fill>>) {
        self.fills = fills;
    }
}

/// The innermost of `outlines` holding `p`: an island may sit in another's hole.
pub(crate) fn holder<'a>(outlines: impl IntoIterator<Item = &'a Contour>, p: Point) -> Option<usize> {
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

/// The width of a layer's sparse beads, and how far apart they lie for the requested density
/// by volume: beads `s` apart in effect, every `S`, fill `s/S` of the area.
pub fn sparse_bead(layer: &RegionLayer, p: &PathParams) -> (f64, f64) {
    let width = if layer.index == 0 { p.first_layer_width } else { p.sparse_width };
    let spacing = if p.sparse_density > 0.0 {
        bead_spacing(width, layer.height) / p.sparse_density
    } else {
        f64::INFINITY
    };
    (width, spacing)
}

/// A point on a line, for asking which island holds it: the middle of its middle segment.
pub(crate) fn on_line(line: &[Point]) -> Option<Point> {
    let k = line.len().checked_sub(2)? / 2;
    let (a, b) = (line.get(k)?, line.get(k + 1)?);
    Some([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0])
}

/// Where an island's loops start, and which way they run.
struct Mark {
    /// The island's outer seam: the reference of every loop of the island but its outer contour.
    seam: Point,
    /// Whether the island's solid has a right-handed frame; its loops run the other way if not.
    right_handed: bool,
}

/// What a layer's infill lines follow, the same for every part of the layer.
#[derive(Clone, Copy)]
pub struct Guides<'a> {
    /// The sparse lines of the rolls, where the layer has them: each part asking for rolls cuts
    /// its own from them.
    pub rolls: Option<&'a [Polyline]>,
    /// The dense beads of the layer below, which the dense lines cross.
    pub beneath: &'a Beneath,
    /// The layer's top over its plane, whose fall line a top surface's lines follow.
    pub relief: &'a Relief,
}

/// Splits a layer, or a part of one, into islands, each holding its walls, the infill inside
/// them and the part's infill areas for joining their lines, which follow `guides`.
pub fn islands(
    layer: &RegionLayer,
    p: &PathParams,
    guides: Guides,
    bodies: &Solids,
    pass: usize,
    seams: &mut Seams,
) -> Result<Vec<Island>, PathError> {
    let Guides { rolls, beneath, relief } = guides;
    let first = layer.index == 0;
    let w = |width: f64| if first { p.first_layer_width } else { width };
    let loop_path = |role: Role, width: f64, contour: &Contour, mark: &Mark| {
        let points = seamed(contour.clone(), mark.seam, width, p.resolution, mark.right_handed);
        Path::new(role, width, true, points)
    };
    // The outer wall's shapes are the islands; without walls, the infill's shapes are. Each
    // follows the seam of its solid on the layer below, or starts at the end of its frame's axis;
    // its holes' and inner walls' seams follow its own.
    let walled = !layer.walls.is_empty();
    let mut islands: Vec<Island> = Vec::new();
    let mut marks: Vec<Mark> = Vec::new();
    for shape in layer.outline() {
        let Some(outline) = shape.first().cloned() else { continue };
        let solid = bodies.solid_of(pass, &outline);
        let frame = solid.map(|s| bodies.frame(s)).unwrap_or_default();
        let reference = solid
            .and_then(|s| seams.follow(s, &outline, &frame, p.resolution))
            .unwrap_or_else(|| farthest_along(&outline, &frame, p.resolution));
        let mut mark = Mark { seam: reference, right_handed: frame.right_handed() };
        let mut walls = Vec::new();
        for (n, c) in shape.iter().enumerate().filter(|_| walled) {
            let path = loop_path(Role::OUTER_WALL, w(p.outer_wall_width), c, &mark);
            if n == 0 {
                mark.seam = path.points.first().copied().unwrap_or(mark.seam);
            }
            walls.push((0, path));
        }
        if let Some(s) = solid {
            seams.keep(s, mark.seam);
        }
        islands.push(Island { outline, walls, gaps: Vec::new(), infill: Vec::new(), fills: Rc::default() });
        marks.push(mark);
    }
    let home = |islands: &[Island], p: Point| -> Option<usize> {
        holder(islands.iter().map(|i| &i.outline), p).or(if islands.is_empty() { None } else { Some(0) })
    };
    for (k, wall) in layer.walls.iter().enumerate().skip(1) {
        for contour in wall.iter().flatten() {
            let Some(&first) = contour.first() else { continue };
            let Some(i) = home(&islands, first) else { continue };
            if let Some(island) = islands.get_mut(i) {
                let Some(mark) = marks.get(i) else { continue };
                let path = loop_path(Role::INNER_WALL, w(p.inner_wall_width), contour, mark);
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
    if let Some(below) = &layer.below {
        let support = Below::new(below, w(p.outer_wall_width).max(w(p.inner_wall_width)));
        for (_, path) in islands.iter_mut().flat_map(|island| island.walls.iter_mut()) {
            (path.points, path.overhang) = overhang(&path.points, path.closed, &support, path.width);
        }
    }
    let odd = layer.index % 2 == 1;
    let turn = p.infill_angle + if odd { 90.0 } else { 0.0 };
    let h = layer.height;
    // One direction per layer, turning a quarter turn each layer: two beads never cross on one
    // layer, where a crossing would lay plastic twice in one place.
    let (_, sparse_spacing) = sparse_bead(layer, p);
    let rolls = rolls.filter(|_| matches!(p.sparse_pattern, SparsePattern::Rolls { .. }));
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
    let crossing = |role: Role| {
        let Crossing { top, inside, first_layer } = p.crossing;
        if first {
            first_layer
        } else if role.face == Face::Top {
            top
        } else {
            inside
        }
    };
    // What the dense areas' insets leave, for the gap fill with the strips.
    let mut residue: Region = Vec::new();
    let mut areas = Vec::new();
    for (role, region, width, spacing) in fills {
        let region = if role.strand() { region.clone() } else { intersection(region, &wide)? };
        if region.is_empty() {
            continue;
        }
        let reach = offset(&region, p.infill_wall_overlap * width)?;
        // Each bridge crosses its own gap the short way; every other fill follows the layer.
        let mut lines = Vec::new();
        let mut along = Vec::new();
        // Over air, from edge to edge; over sparse infill, across its lines, which the layer's
        // direction already crosses at right angles.
        if role.support == Support::Air {
            // An end hangs when its strand's last bead rests nowhere on the layer below.
            let below = layer.below.as_ref().map(|below| Below::new(below, width));
            let hangs = |end: Point| below.as_ref().is_none_or(|b| b.fraction(end, width) >= 1.0);
            for shape in &reach {
                let piece = vec![shape.clone()];
                // Across the solid's own direction, on lines through its own origin.
                let frame = inside(shape).map(|q| bodies.frame_at(pass, q)).unwrap_or_default();
                let (preferred, origin) = (frame.turned(turn), frame.origin());
                let phase = phase_through(preferred, origin);
                let angle = bridge_angle(&piece, spacing, preferred, phase, hangs)?;
                lines.extend(hatch(&piece, spacing, angle, phase_through(angle, origin))?);
            }
        } else if role.kind == Kind::Solid && role.support == Support::Solid {
            let on_top = role.face == Face::Top;
            let fall = |piece: &Region| if on_top { fall_line(piece, relief) } else { None };
            let laid = fill(
                &reach,
                Spacing { spacing, width, crossing: crossing(role), shows: on_top },
                beneath,
                fall,
            )?;
            lines = laid.lines;
            along = laid.insets;
            residue = union(&residue, &laid.residue)?;
        } else if let Some(rolls) = rolls.filter(|_| role == Role::SPARSE) {
            // A stroke shorter than two beads would lay a blob.
            lines = clip_lines(rolls, &reach)?;
            lines.retain(|line| polyline_length(line) >= 2.0 * width);
        } else {
            lines = hatch_in_frames(&reach, spacing, turn, |q| bodies.frame_at(pass, q))?;
        }
        for line in lines {
            let Some(i) = on_line(&line).and_then(|q| home(&islands, q)) else { continue };
            if let Some(island) = islands.get_mut(i) {
                island.infill.push(Path::new(role, width, false, line));
            }
        }
        // The insets' beads run along their area, as the strips' do: with them, before the lines.
        for (points, closed) in along {
            let Some(i) = on_line(&points).and_then(|q| home(&islands, q)) else { continue };
            if let Some(island) = islands.get_mut(i) {
                island.gaps.push(Path::new(role, width, closed, points));
            }
        }
        // A role's areas join their lines as one.
        if !areas.iter().any(|f: &Fill| f.role == role) {
            areas.push(Fill {
                role,
                max_link: 3.0 * spacing,
                monotone: role.kind == Kind::Solid && p.monotone.contains(&role.face),
            });
        }
    }
    let axis_at = |q: Point| bodies.frame_at(pass, q).axis();
    let gaps = gap_fill(&union(&thin, &residue)?, role_at, &axis_at, s, h, p.resolution)?;
    for gap in gaps {
        let Some(&start) = gap.points.first() else { continue };
        if let Some(island) = home(&islands, start).and_then(|i| islands.get_mut(i)) {
            island.gaps.push(gap);
        }
    }
    let fills = Rc::new(areas);
    for (island, mark) in islands.iter_mut().zip(&marks) {
        island.fills = fills.clone();
        // The closed beads of the dense areas start at the island's seam too.
        for bead in island.gaps.iter_mut().filter(|g| g.closed) {
            bead.points = seamed(
                std::mem::take(&mut bead.points),
                mark.seam,
                bead.width,
                p.resolution,
                mark.right_handed,
            );
        }
    }
    Ok(islands)
}

/// Lays parallel lines in sweeps across their area, each line against the one before and all
/// sweeps the same way: from the side nearer the nozzle, each sweep takes the next line across
/// that overlaps the last one along their direction and lies within `reach`, and a new sweep
/// starts at the first line left. Each line starts where the last one ended.
fn sweep(mut lines: Vec<Path>, reach: f64, at: &mut Point, out: &mut Vec<Path>, cover: &Cover) {
    let Some((a, b)) = lines.first().and_then(|l| Some((*l.points.first()?, *l.points.last()?))) else {
        return tour(lines, at, out, cover);
    };
    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
    if length <= 0.0 {
        return tour(lines, at, out, cover);
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
                line.reverse();
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

/// Lines grouped by their direction, the direction laying the longest first, the one with an end
/// nearer the nozzle `at` where equally long: each is swept on
/// its own.
fn by_direction(lines: Vec<Path>, at: Point) -> Vec<Vec<Path>> {
    let direction = |l: &Path| match (l.points.first(), l.points.last()) {
        (Some(a), Some(b)) => (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees().rem_euclid(180.0),
        _ => 0.0,
    };
    let mut groups: Vec<(f64, f64, Vec<Path>)> = Vec::new();
    for line in lines {
        let (angle, length) = (direction(&line), polyline_length(&line.points));
        let near = |g: &&mut (f64, f64, Vec<Path>)| {
            let d = (g.0 - angle).rem_euclid(180.0);
            d.min(180.0 - d) < SAME_SWEEP
        };
        match groups.iter_mut().find(near) {
            Some(group) => {
                group.1 += length;
                group.2.push(line);
            }
            None => groups.push((angle, length, vec![line])),
        }
    }
    let nearest_end = |lines: &[Path]| {
        lines
            .iter()
            .flat_map(|l| [l.points.first(), l.points.last()])
            .flatten()
            .map(|p| distance2(at, *p))
            .fold(f64::MAX, f64::min)
    };
    groups.sort_by(|x, y| y.1.total_cmp(&x.1).then(nearest_end(&x.2).total_cmp(&nearest_end(&y.2))));
    groups.into_iter().map(|(_, _, lines)| lines).collect()
}

/// Lines closer in direction than this, in degrees, are swept together: parallel lines differ by
/// the lattice's rounding of their ends.
const SAME_SWEEP: f64 = 1e-3;

/// The layer's beads in printing order: island by island, nearest first; within an island the
/// walls, then the beads along its strips, against them, then each infill area's lines, in
/// sweeps for a monotone area and in the cheapest tour otherwise, joined where they can be.
/// Each tour keeps its moves off the beads laid before it, these and `earlier`, where it can.
pub fn sequence(mut islands: Vec<Island>, at: &mut Point, earlier: &[Segment]) -> Vec<Path> {
    let laid_by = |out: &[Path]| {
        let beads = earlier.iter().copied().chain(out.iter().flat_map(segments));
        Laid::new(&beads.map(|(a, b)| (a, b, 0)).collect::<Vec<_>>())
    };
    let mut out = Vec::new();
    while !islands.is_empty() {
        let near = islands
            .iter()
            .enumerate()
            .map(|(i, island)| (i, distance2(*at, island.first_start())))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
            .unwrap_or(0);
        let mut island = islands.swap_remove(near);
        island.walls.sort_by_key(|(k, _)| std::cmp::Reverse(*k));
        let mut walls = Vec::new();
        let mut depth = None;
        for (k, path) in island.walls {
            if depth != Some(k) && !walls.is_empty() {
                let laid = laid_by(&out);
                tour(std::mem::take(&mut walls), at, &mut out, &Cover { laid: &laid });
            }
            depth = Some(k);
            walls.push(path);
        }
        let laid = laid_by(&out);
        tour(walls, at, &mut out, &Cover { laid: &laid });
        let laid = laid_by(&out);
        tour(island.gaps, at, &mut out, &Cover { laid: &laid });
        let mut infill = island.infill;
        for fill in island.fills.iter() {
            let (mine, rest): (Vec<Path>, Vec<Path>) = infill.into_iter().partition(|p| p.role == fill.role);
            infill = rest;
            let mut ordered = Vec::new();
            let laid = laid_by(&out);
            let cover = Cover { laid: &laid };
            if fill.monotone {
                for lines in by_direction(mine, *at) {
                    sweep(lines, fill.max_link, at, &mut ordered, &cover);
                }
            } else {
                tour(mine, at, &mut ordered, &cover);
            }
            out.extend(ordered);
        }
        let laid = laid_by(&out);
        tour(infill, at, &mut out, &Cover { laid: &laid });
    }
    out
}
