//! A pass's beads, grouped by island. An island is a shape of the outer wall, or of the infill
//! where there is no wall, and holds every bead inside it. Its walls are closed loops started at
//! their seams (`seam`), each segment carrying the share of its bead that hangs past the layer
//! below (`overhang`). Inside them each infill area is laid by its own rule (`Lay`): dense areas
//! as families of curves (`families`), bridges as strands across their gap, sparse areas as the
//! rolls' lines (`rolls`) or as straight lines in the solid's frame, and the strips too thin for
//! lines as one bead along each (`gap`). A pass's islands are grouped by the parameters at their
//! centroids (`parts`).

use crate::families::{Beneath, Laying, fill};
use crate::frame::hatch_in_frames;
use crate::gap::{gap_fill, strips};
use crate::overhang::{Below, overhang};
use crate::rolls::RollLines;
use crate::seam::{Seams, farthest_along, seamed};
use crate::solids::Solids;
use crate::{Kind, Path, PathError, PathParams, Role, check};
use layercraft_geom::{
    Bands, Contour, Frame, GRID_MM, Moments, Point, Region, Relief, Rests, Shape, bead_spacing,
    bridge_strands, clip_lines, disc_area, edges, encloses, intersection, offset, polyline_length,
    segment_distance, signed_area, union,
};
use layercraft_region::RegionPass;
use layercraft_tune::Field;
use std::rc::Rc;

/// The farthest apart, in line spacings, two lines of one sweep lie: past it, the next line
/// starts a sweep of its own.
const SWEEP_REACH: f64 = 3.0;

/// How the lines of an infill area follow one another.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Order {
    /// The tour that crosses laid beads least often, then travels least.
    Tour,
    /// Sweeps across the area, each line against the one before, lines at most `reach` apart.
    Sweep { reach: f64 },
}

/// The lines of one infill area of an island, and how they follow one another.
pub struct Fill {
    pub order: Order,
    pub lines: Vec<Path>,
}

/// The beads of one connected piece of a pass.
pub struct Island {
    /// The outer boundary that decides which beads belong here.
    pub outline: Contour,
    /// Each wall's loops, with their depth: 0 for the outer wall.
    pub walls: Vec<(usize, Path)>,
    /// The beads that run along their area: the strips too thin for infill lines, the insets.
    pub gaps: Vec<Path>,
    /// The lines of each infill area, in the order the areas are laid.
    pub fills: Vec<Fill>,
}

impl Island {
    /// An island bounded by `outline`, its beads to come.
    pub fn new(outline: Contour) -> Island {
        Island { outline, walls: Vec::new(), gaps: Vec::new(), fills: Vec::new() }
    }

    /// Where the island's beads start: its outer wall's seam, which its other loops' follow;
    /// without walls, the start of its first bead.
    pub fn first_start(&self) -> Point {
        let outer = self.walls.iter().find(|(k, _)| *k == 0).map(|(_, p)| p);
        let first = outer.or(self.gaps.first()).or(self.fills.iter().find_map(|f| f.lines.first()));
        first.and_then(|p| p.points.first()).or(self.outline.first()).copied().unwrap_or([0.0, 0.0])
    }

    /// Every bead of the island.
    pub fn beads(&self) -> impl Iterator<Item = &Path> {
        let fills = self.fills.iter().flat_map(|f| &f.lines);
        self.walls.iter().map(|(_, p)| p).chain(&self.gaps).chain(fills)
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

/// The island holding `q`: the innermost whose outline holds it, or, for a point in none, as the
/// end of a line reaching into the walls past a corner, the one whose outline passes nearest.
pub(crate) fn island_of(islands: &[Island], q: Point) -> Option<usize> {
    holder(islands.iter().map(|i| &i.outline), q).or_else(|| {
        let gap =
            |o: &Contour| edges(o).map(|(a, b)| segment_distance(q, a, b)).fold(f64::INFINITY, f64::min);
        let gaps = islands.iter().map(|i| gap(&i.outline)).enumerate();
        gaps.min_by(|a, b| a.1.total_cmp(&b.1)).map(|(i, _)| i)
    })
}

/// Gives every island a fill laid in `order`, holding those of `lines` whose middle it holds.
pub(crate) fn share(islands: &mut [Island], order: Order, lines: impl IntoIterator<Item = Path>) {
    for island in islands.iter_mut() {
        island.fills.push(Fill { order, lines: Vec::new() });
    }
    for line in lines {
        let Some(i) = on_line(&line.points).and_then(|q| island_of(islands, q)) else { continue };
        if let Some(fill) = islands.get_mut(i).and_then(|island| island.fills.last_mut()) {
            fill.lines.push(line);
        }
    }
}

/// A point on a line, for asking which island holds it: the middle of its middle segment.
pub(crate) fn on_line(line: &[Point]) -> Option<Point> {
    let k = line.len().checked_sub(2)? / 2;
    let (a, b) = (line.get(k)?, line.get(k + 1)?);
    Some([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0])
}

/// Where the parameters of a shape are read, at height `z`: its centroid, which turns and moves
/// with it; its first point when it has no area.
fn centroid(shape: &Shape, z: f64) -> [f64; 3] {
    let at = Moments::of(std::iter::once((shape.as_slice(), 1.0))).map(|m| m.centroid);
    let [x, y] = at.or(shape.first().and_then(|c| c.first()).copied()).unwrap_or([0.0, 0.0]);
    [x, y, z]
}

/// The pass's islands grouped by the parameters at their centroids, each group with the part of
/// the pass in its islands. An island is a shape of the outer wall, or of the infill where there
/// is no wall; every other shape belongs to the island holding it.
pub fn parts(
    pass: &RegionPass,
    field: &Field<PathParams>,
) -> Result<Vec<(Rc<PathParams>, RegionPass)>, PathError> {
    let shapes: Vec<&Shape> = pass.outline().into_iter().filter(|shape| !shape.is_empty()).collect();
    let outlines: Vec<&Contour> = shapes.iter().filter_map(|shape| shape.first()).collect();
    let mut groups: Vec<Rc<PathParams>> = Vec::new();
    let mut group_of = Vec::with_capacity(outlines.len());
    for shape in &shapes {
        let p = field.at(centroid(shape, pass.top), check)?;
        group_of.push(match groups.iter().position(|q| Rc::ptr_eq(q, &p)) {
            Some(g) => g,
            None => {
                groups.push(p);
                groups.len() - 1
            }
        });
    }
    if groups.len() <= 1 {
        return Ok(groups.into_iter().map(|p| (p, pass.clone())).collect());
    }
    let mut empty = RegionPass { walls: vec![Vec::new(); pass.walls.len()], ..pass.clone() };
    for region in empty.infill_mut() {
        region.clear();
    }
    let mut parts = vec![empty; groups.len()];
    // An outline's own shape is its island's; any other lies strictly inside the one holding it.
    let held = |shape: &[Contour]| {
        let p = shape.first().and_then(|c| c.first()).copied().unwrap_or([0.0, 0.0]);
        holder(outlines.iter().copied(), p).and_then(|i| group_of.get(i).copied()).unwrap_or(0)
    };
    let walled = !pass.walls.is_empty();
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
    for (k, wall) in pass.walls.iter().enumerate() {
        for shape in wall {
            let slot =
                group(shape, k == 0).and_then(|g| parts.get_mut(g)).and_then(|part| part.walls.get_mut(k));
            if let Some(slot) = slot {
                slot.push(shape.clone());
            }
        }
    }
    for (j, region) in pass.infill().into_iter().enumerate() {
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

/// The width of a pass's sparse beads, and how far apart they lie for the requested density
/// by volume: beads `s` apart in effect, every `S`, fill `s/S` of the area.
pub fn sparse_bead(pass: &RegionPass, p: &PathParams) -> (f64, f64) {
    let width = p.width_on(pass.index, p.sparse_width);
    let spacing = if p.sparse_density > 0.0 {
        bead_spacing(width, pass.height) / p.sparse_density
    } else {
        f64::INFINITY
    };
    (width, spacing)
}

/// Where an island's loops start, and which way they run.
struct Mark {
    /// The island's outer seam: the reference of every loop of the island but its outer contour.
    seam: Point,
    /// Whether the island's solid has a right-handed frame; its loops run the other way if not.
    right_handed: bool,
}

/// What a pass's infill lines follow.
#[derive(Clone, Copy)]
pub struct Guides<'a> {
    /// The rolls' lines, where the part asks for them, and what of its area they leave.
    pub rolls: Option<&'a RollLines>,
    /// The dense beads of the layer below, which the dense lines cross.
    pub beneath: &'a Beneath,
    /// The layer's top over its plane, whose fall line a top surface's lines follow.
    pub relief: &'a Relief,
}

/// How an infill area's lines are laid.
#[derive(Clone, Copy)]
enum Lay<'a> {
    /// Strands across a gap over air, in the gap's own frame, held at their ends.
    Span,
    /// Families of curves, crossing the dense beads below where the area is wider than `crossing`;
    /// a top surface's follow its fall line and pay for the joins they show.
    Dense { crossing: f64, top: bool },
    /// The rolls' lines cut to the area.
    Rolls(&'a RollLines),
    /// Straight lines in the solid's frame, turning a quarter turn each layer.
    Hatch,
}

/// One infill area of a pass, its beads and how they are laid.
struct Area<'a> {
    role: Role,
    region: &'a Region,
    width: f64,
    spacing: f64,
    lay: Lay<'a>,
}

/// Splits a pass, or a part of one, into islands, each holding its walls, and the infill inside
/// them laid as `guides` lead; `index` is the pass's number in the print.
pub fn islands(
    pass: &RegionPass,
    p: &PathParams,
    guides: Guides,
    bodies: &Solids,
    index: usize,
    seams: &mut Seams,
) -> Result<Vec<Island>, PathError> {
    let (mut islands, marks) = walled(pass, p, bodies, index, seams)?;
    let Guides { rolls, beneath, relief } = guides;
    let (first, h) = (pass.index == 0, pass.height);
    let w = |width: f64| p.width_on(pass.index, width);
    // One direction per layer, turning a quarter turn each layer: two beads never cross on one
    // layer, where a crossing would lay plastic twice in one place.
    let turn = p.infill_angle + if pass.index % 2 == 1 { 90.0 } else { 0.0 };
    let (top, solid) = (w(p.top_surface_width), w(p.solid_width));
    let dense = Role::solid_on(pass.index);
    let crossing = |shows: bool| match (first, shows) {
        (true, _) => p.crossing.first_layer,
        (false, true) => p.crossing.top,
        (false, false) => p.crossing.inside,
    };
    // Bridge strands are round, a nozzle wide. Over air they hang, touching side by side. Over
    // sparse infill they rest on its lines, a solid layer like any other: spaced so that each
    // strand's section, π·w²/4, fills its spacing times the layer's height; and they cross those
    // lines, which the layer's direction already crosses at right angles.
    let resting = disc_area(p.bridge_width) / h;
    let (_, sparse_spacing) = sparse_bead(pass, p);
    let areas = [
        Area {
            role: Role::TOP_SURFACE,
            region: &pass.top_surface,
            width: top,
            spacing: bead_spacing(top, h),
            lay: Lay::Dense { crossing: crossing(true), top: true },
        },
        Area {
            role: dense,
            region: &pass.solid,
            width: solid,
            spacing: bead_spacing(solid, h),
            lay: Lay::Dense { crossing: crossing(false), top: false },
        },
        Area {
            role: Role::BRIDGE,
            region: &pass.bridge,
            width: p.bridge_width,
            spacing: p.bridge_width,
            lay: Lay::Span,
        },
        Area {
            role: Role::INTERNAL_BRIDGE,
            region: &pass.bridge_on_sparse,
            width: p.bridge_width,
            spacing: resting,
            lay: Lay::Hatch,
        },
        Area {
            role: Role::SPARSE,
            region: &pass.sparse,
            width: w(p.sparse_width),
            spacing: sparse_spacing,
            lay: rolls.map_or(Lay::Hatch, Lay::Rolls),
        },
    ];
    // A strip of the laid areas narrower than two dense lines takes one bead along it, whichever
    // areas it runs through, sparse infill's too, and each stretch of it the role of its area;
    // a bridge's strands hang across whatever they span.
    let s = bead_spacing(solid, h);
    let lined = union(&union(&pass.top_surface, &pass.solid)?, &pass.sparse)?;
    let (thin, wide) = strips(&lined, s)?;
    // What the dense areas' insets leave, for the gap fill with the strips.
    let mut residue: Region = Vec::new();
    for Area { role, region, width, spacing, lay } in areas {
        let region = if role.strand() { region.clone() } else { intersection(region, &wide)? };
        if region.is_empty() {
            continue;
        }
        let reach = offset(&region, p.infill_wall_overlap * width)?;
        let frame_at = |q: Point| bodies.frame_at(index, q);
        let lines = match lay {
            Lay::Span => {
                // Each piece in its own frame, as the region stage laid it to find what it needs
                // under it; an end hangs when its strand's last bead rests nowhere below.
                let rests = Rests::on(pass.below.as_ref().unwrap_or(&Vec::new()), width)?;
                let mut lines = Vec::new();
                for shape in &reach {
                    let piece = vec![shape.clone()];
                    let frame = Frame::of(std::iter::once((&piece, 1.0)));
                    lines.extend(bridge_strands(&piece, spacing, &frame, &rests)?.1);
                }
                lines
            }
            Lay::Dense { crossing, top } => {
                let laying = Laying { spacing, width, crossing };
                let dense = fill(&reach, laying, beneath, top.then_some(relief))?;
                residue = union(&residue, &dense.residue)?;
                // The insets' beads run along their area, as the strips' do: with them, before
                // the lines.
                for (points, closed) in dense.insets {
                    let Some(i) = on_line(&points).and_then(|q| island_of(&islands, q)) else { continue };
                    if let Some(island) = islands.get_mut(i) {
                        island.gaps.push(Path::new(role, width, closed, points));
                    }
                }
                dense.lines
            }
            Lay::Rolls(rolls) => {
                // A stroke shorter than two beads would lay a blob.
                let mut lines = clip_lines(&rolls.lines, &reach)?;
                lines.retain(|line| polyline_length(line) >= 2.0 * width);
                let left = intersection(&reach, &rolls.left)?;
                lines.extend(hatch_in_frames(&left, spacing, turn, frame_at)?);
                lines
            }
            Lay::Hatch => hatch_in_frames(&reach, spacing, turn, frame_at)?,
        };
        let order = if role.kind == Kind::Solid && p.monotone.contains(&role.face) {
            Order::Sweep { reach: SWEEP_REACH * spacing }
        } else {
            Order::Tour
        };
        share(&mut islands, order, lines.into_iter().map(|line| Path::new(role, width, false, line)));
    }
    let (tops, solids) = (Bands::new(&pass.top_surface), Bands::new(&pass.solid));
    let role_at = |q: Point| {
        if tops.encloses(q) {
            Role::TOP_SURFACE
        } else if solids.encloses(q) {
            dense
        } else {
            Role::SOLID
        }
    };
    let axis_at = |q: Point| bodies.frame_at(index, q).axis();
    for gap in gap_fill(&union(&thin, &residue)?, role_at, &axis_at, s, h, p.resolution)? {
        let Some(&start) = gap.points.first() else { continue };
        if let Some(island) = island_of(&islands, start).and_then(|i| islands.get_mut(i)) {
            island.gaps.push(gap);
        }
    }
    // The closed beads along the dense areas start at the island's seam too.
    for (island, mark) in islands.iter_mut().zip(&marks) {
        for bead in island.gaps.iter_mut().filter(|g| g.closed) {
            let points = std::mem::take(&mut bead.points);
            bead.points = seamed(points, mark.seam, bead.width, p.resolution, mark.right_handed);
        }
    }
    Ok(islands)
}

/// The islands of a pass with their walls, and where each island's loops start. The outer wall's
/// shapes are the islands; without walls, the infill's shapes are. Each follows the seam of its
/// solid on the pass before, or starts at the end of its frame's axis; its holes' and inner walls'
/// seams follow its own.
fn walled(
    pass: &RegionPass,
    p: &PathParams,
    bodies: &Solids,
    index: usize,
    seams: &mut Seams,
) -> Result<(Vec<Island>, Vec<Mark>), PathError> {
    let w = |width: f64| p.width_on(pass.index, width);
    let loop_path = |role: Role, width: f64, contour: &Contour, mark: &Mark| {
        let points = seamed(contour.clone(), mark.seam, width, p.resolution, mark.right_handed);
        Path::new(role, width, true, points)
    };
    let walled = !pass.walls.is_empty();
    let mut islands: Vec<Island> = Vec::new();
    let mut marks: Vec<Mark> = Vec::new();
    for shape in pass.outline() {
        let Some(outline) = shape.first().cloned() else { continue };
        let solid = bodies.solid_of(index, &outline);
        let frame = solid.map(|s| bodies.frame(s)).unwrap_or_default();
        let reference = solid
            .and_then(|s| seams.follow(s, &outline, &frame, p.resolution))
            .unwrap_or_else(|| farthest_along(&outline, &frame, p.resolution));
        let mut mark = Mark { seam: reference, right_handed: frame.right_handed() };
        let mut island = Island::new(outline);
        for (n, c) in shape.iter().enumerate().filter(|_| walled) {
            let path = loop_path(Role::OUTER_WALL, w(p.outer_wall_width), c, &mark);
            if n == 0 {
                mark.seam = path.points.first().copied().unwrap_or(mark.seam);
            }
            island.walls.push((0, path));
        }
        if let Some(s) = solid {
            seams.keep(s, mark.seam);
        }
        islands.push(island);
        marks.push(mark);
    }
    for (k, wall) in pass.walls.iter().enumerate().skip(1) {
        for contour in wall.iter().flatten() {
            let Some(&first) = contour.first() else { continue };
            let Some(i) = island_of(&islands, first) else { continue };
            let (Some(island), Some(mark)) = (islands.get_mut(i), marks.get(i)) else { continue };
            let path = loop_path(Role::INNER_WALL, w(p.inner_wall_width), contour, mark);
            let Some(within) = &pass.inner_walls_within else {
                island.walls.push((k, path));
                continue;
            };
            // Only the stretches of the loop under the layer above; a loop wholly under it stays
            // a loop, its seam where it was.
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
            for piece in pieces.into_iter().filter(|piece| piece.len() >= 2) {
                island.walls.push((k, Path { closed: false, points: piece, ..path.clone() }));
            }
        }
    }
    // How far each wall hangs over the layer below, which motion slows for.
    if let Some(below) = &pass.below {
        let support = Below::new(below, w(p.outer_wall_width).max(w(p.inner_wall_width)));
        for (_, path) in islands.iter_mut().flat_map(|island| island.walls.iter_mut()) {
            (path.points, path.overhang) = overhang(&path.points, path.closed, &support, path.width);
        }
    }
    Ok((islands, marks))
}
