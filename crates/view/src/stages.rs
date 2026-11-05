//! Each stage as an attributed complex, in the printer's frame, in millimetres.
//!
//! A bead's centre line lies half its height under the nozzle: at a layer's mid-height, where
//! the section cuts, or a strand's radius down for a bridge, which hangs round. So the stages of
//! one layer overlay, and a role has the same code in all of them. A bent layer is drawn on its
//! bend: sections and regions halfway between the layer's top and the top of the one below it,
//! beads half their height under the top; each edge and area is cut where the bend turns, so
//! that it lies on the bend everywhere. A comment names the values of each label,
//! `role: 0 outer wall, 1 inner wall, …`, for a viewer's legend.
//!
//! - mesh: the bodies' triangles, with their `body`;
//! - sections: each layer's section, triangulated, with its `layer`;
//! - regions: the walls' centre lines as segments, and the areas they enclose (top surface,
//!   solid, bridges, sparse) and the supports' triangulated, with `layer` and `role`;
//! - paths: each bead's centre line, with `layer`, `role`, `width`, `height` and `bead`, its
//!   place in the printing order;
//! - program: the nozzle's moves, each point with its time `t`, each segment with `layer`,
//!   `move`, `role`, `width`, `height`, `speed`, `e` and `fan`. A move that lays plastic is
//!   drawn along the bead it lays; the others where the nozzle goes, a helical lift as chords.
//!   After the machine's own G-code the nozzle may be anywhere: nothing is drawn until it is known.

use crate::ply::{Complex, ViewError, float, int};
use layercraft_geom::{Carrier, Point, Region, Relief, Warp, bounds, slice, triangulate};
use layercraft_mesh::Mesh;
use layercraft_motion::{Limits, Program, estimate};
use layercraft_path::{Paths, Role};
use layercraft_region::Regions;
use layercraft_section::Sections;
use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet};

/// A role's name in a legend.
pub fn role_name(role: Role) -> String {
    role.name().to_lowercase()
}

/// The code of a move that lays no plastic.
const NO_ROLE: f64 = -1.0;

fn role_code(role: Role) -> f64 {
    role.code() as f64
}

fn role_comment() -> String {
    let codes: Vec<String> =
        Role::NAMED.iter().enumerate().map(|(i, r)| format!("{i} {}", role_name(*r))).collect();
    format!("role: {}", codes.join(", "))
}

/// A bead's cross-section, width and height: a layer high, or round for a strand.
fn bead_section(role: Role, width: f64, layer_height: f64) -> [f64; 2] {
    if role.strand() { [width, width] } else { [width, layer_height] }
}

/// The farthest a chord of a drawn arc strays from it, in mm: a fortieth of a bead.
const SAG_MM: f64 = 0.01;

/// Chords for a full turn of radius `r`, each within `SAG_MM` of the arc.
fn chords(r: f64) -> usize {
    let n = (std::f64::consts::PI / (1.0 - SAG_MM / r).clamp(-1.0, 1.0).acos()).ceil();
    if n.is_finite() { n.clamp(3.0, 4096.0) as usize } else { 3 }
}

/// How far a layer's drawing lies over its plane at each point: a mean of the layer tops' reliefs.
struct Bend<'a> {
    parts: Vec<(&'a Relief, f64)>,
}

impl Bend<'_> {
    fn is_flat(&self) -> bool {
        self.parts.iter().all(|(relief, _)| relief.is_zero())
    }

    fn at(&self, p: Point) -> f64 {
        self.parts.iter().map(|(relief, share)| share * relief.at(p)).sum()
    }

    /// The fractions of the way from `a` to `b` where it may turn, in order.
    fn breaks(&self, a: Point, b: Point) -> Vec<f64> {
        let mut ts: Vec<f64> = self.parts.iter().flat_map(|(relief, _)| relief.breaks(a, b)).collect();
        ts.sort_by(f64::total_cmp);
        ts.dedup();
        ts
    }

    /// The edges of its triangles over the box from `lo` to `hi`, each once.
    fn edges(&self, lo: Point, hi: Point) -> Vec<[Point; 2]> {
        let mut seen = HashSet::new();
        let mut out = Vec::new();
        for (relief, _) in &self.parts {
            for ([a, b, c], _) in relief.facets_over(lo, hi) {
                for (p, q) in [(a, b), (b, c), (c, a)] {
                    let (u, v) = ([p[0].to_bits(), p[1].to_bits()], [q[0].to_bits(), q[1].to_bits()]);
                    if seen.insert(if u <= v { (u, v) } else { (v, u) }) {
                        out.push([p, q]);
                    }
                }
            }
        }
        out
    }
}

/// Each layer's top as a relief, and the position of the layer each stands on, the last pass of
/// the index before its own, for layers given by their indices and warps.
fn tops<'a>(
    carrier: &Carrier,
    layers: impl Iterator<Item = (usize, &'a Warp)>,
) -> Result<(Vec<Relief>, Vec<Option<usize>>), ViewError> {
    let (mut reliefs, mut below) = (Vec::new(), Vec::new());
    let (mut under, mut current): (Option<usize>, Option<(usize, usize)>) = (None, None);
    for (position, (index, warp)) in layers.enumerate() {
        reliefs.push(carrier.relief(warp).map_err(ViewError::Geometry)?);
        if let Some((last, at)) = current
            && last != index
        {
            under = Some(at);
        }
        below.push(under);
        current = Some((index, position));
    }
    Ok((reliefs, below))
}

/// Each layer's drawing halfway between its top and the top it stands on, the bed at first.
fn middles<'a>(reliefs: &'a [Relief], below: &[Option<usize>], bed: &'a Relief) -> Vec<Bend<'a>> {
    reliefs
        .iter()
        .zip(below)
        .map(|(own, under)| {
            let under = under.and_then(|k| reliefs.get(k)).unwrap_or(bed);
            Bend { parts: vec![(own, 0.5), (under, 0.5)] }
        })
        .collect()
}

/// Adds `region`'s triangles at height `z` on `bend`: each face between the edges of the bend's
/// triangles is linear, so that its own triangles lie on the bend.
fn fill<const E: usize, const F: usize>(
    c: &mut Complex<0, E, F>,
    region: &Region,
    z: f64,
    bend: &Bend,
    values: [f64; F],
) -> Result<(), ViewError> {
    let mut faces: Vec<Region> = vec![region.clone()];
    if !bend.is_flat()
        && let Some([lo, hi]) = bounds(region)
    {
        let edges = bend.edges(lo, hi);
        if !edges.is_empty() {
            faces =
                slice(region, &edges).map_err(ViewError::Geometry)?.into_iter().map(|s| vec![s]).collect();
        }
    }
    // The faces share their corners, on the lattice: a point of the plate, one point of the bend.
    let mut ids: HashMap<[u64; 2], u32> = HashMap::new();
    for face in &faces {
        let (points, triangles) = triangulate(face).map_err(ViewError::Geometry)?;
        let mut own = Vec::with_capacity(points.len());
        for p in &points {
            own.push(match ids.entry([p[0].to_bits(), p[1].to_bits()]) {
                Entry::Occupied(id) => *id.get(),
                Entry::Vacant(place) => *place.insert(c.point([p[0], p[1], z + bend.at(*p)], [])?),
            });
        }
        for t in triangles {
            if let [Some(&a), Some(&b), Some(&d)] = t.map(|i| own.get(i as usize)) {
                c.triangle([a, b, d], values);
            }
        }
    }
    Ok(())
}

/// Adds a polyline at height `z` on `bend`, back to its start when `closed`.
fn polyline<const E: usize, const F: usize>(
    c: &mut Complex<0, E, F>,
    points: &[Point],
    closed: bool,
    z: f64,
    bend: &Bend,
    values: [f64; E],
) -> Result<(), ViewError> {
    let closes = closed && points.len() >= 3;
    let mut ids = Vec::with_capacity(points.len());
    let mut add = |c: &mut Complex<0, E, F>, p: Point| -> Result<(), ViewError> {
        ids.push(c.point([p[0], p[1], z + bend.at(p)], [])?);
        Ok(())
    };
    let ends = points.iter().zip(points.iter().cycle().skip(1)).take(if closes {
        points.len()
    } else {
        points.len().saturating_sub(1)
    });
    for (&a, &b) in ends {
        add(c, a)?;
        for t in bend.breaks(a, b) {
            add(c, [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])])?;
        }
    }
    if closes {
        ids.extend(ids.first().copied());
    } else if let Some(&last) = points.last() {
        ids.push(c.point([last[0], last[1], z + bend.at(last)], [])?);
    }
    for w in ids.windows(2) {
        if let [a, b] = *w {
            c.segment([a, b], values);
        }
    }
    Ok(())
}

/// The bodies' triangles, with their `body`.
pub fn mesh(mesh: &Mesh) -> Result<Complex<0, 0, 1>, ViewError> {
    let mut c = Complex::new([], [], [int("body")]);
    let names: Vec<String> = mesh.bodies.iter().enumerate().map(|(i, b)| format!("{i} {}", b.name)).collect();
    c.comments.push(format!("body: {}", names.join(", ")));
    for (k, body) in mesh.bodies.iter().enumerate() {
        let first = c.points.rows.len() as u32;
        for v in &body.vertices {
            c.point(*v, [])?;
        }
        for t in &body.triangles {
            if t.iter().any(|&i| i as usize >= body.vertices.len()) {
                return Err(ViewError::Invalid(format!("a triangle of body {k} names a missing vertex")));
            }
            c.triangle(t.map(|i| first + i), [k as f64]);
        }
    }
    Ok(c)
}

/// Each layer's section, at mid-height, with its `layer`.
pub fn sections(sections: &Sections) -> Result<Complex<0, 0, 1>, ViewError> {
    let mut c = Complex::new([], [], [int("layer")]);
    let (reliefs, below) = tops(&sections.carrier, sections.layers.iter().map(|l| (l.index, &l.warp)))?;
    let bed = Relief::default();
    for (layer, bend) in sections.layers.iter().zip(middles(&reliefs, &below, &bed)) {
        fill(&mut c, &layer.region, layer.top - layer.height / 2.0, &bend, [layer.index as f64])?;
    }
    Ok(c)
}

/// The walls' centre lines and the areas inside them, with `layer` and `role`.
pub fn regions(regions: &Regions) -> Result<Complex<0, 2, 2>, ViewError> {
    let mut c = Complex::new([], [int("layer"), int("role")], [int("layer"), int("role")]);
    c.comments.push(role_comment());
    let (reliefs, below) = tops(&regions.carrier, regions.layers.iter().map(|l| (l.index, &l.warp)))?;
    let bed = Relief::default();
    for (layer, bend) in regions.layers.iter().zip(middles(&reliefs, &below, &bed)) {
        let (z, index) = (layer.top - layer.height / 2.0, layer.index as f64);
        for (k, wall) in layer.walls.iter().enumerate() {
            let role = role_code(if k == 0 { Role::OUTER_WALL } else { Role::INNER_WALL });
            for contour in wall.iter().flatten() {
                polyline(&mut c, contour, true, z, &bend, [index, role])?;
            }
        }
        let solid = if layer.index == 0 { Role::BOTTOM_SURFACE } else { Role::SOLID };
        let roles = [Role::TOP_SURFACE, solid, Role::BRIDGE, Role::INTERNAL_BRIDGE, Role::SPARSE];
        for (area, role) in layer.infill().into_iter().zip(roles) {
            fill(&mut c, area, z, &bend, [index, role_code(role)])?;
        }
        for (area, role) in
            [(&layer.supports, Role::SUPPORT), (&layer.support_interface, Role::SUPPORT_INTERFACE)]
        {
            fill(&mut c, area, z, &bend, [index, role_code(role)])?;
        }
    }
    Ok(c)
}

/// Each bead's centre line, with `layer`, `role`, `width`, `height` and `bead`.
pub fn paths(paths: &Paths) -> Result<Complex<0, 5, 0>, ViewError> {
    let properties = [int("layer"), int("role"), float("width"), float("height"), int("bead")];
    let mut c = Complex::new([], properties, []);
    c.comments.push(role_comment());
    let mut bead = 0usize;
    let (reliefs, _) = tops(&paths.carrier, paths.layers.iter().map(|l| (l.index, &l.warp)))?;
    for (layer, relief) in paths.layers.iter().zip(&reliefs) {
        let bend = Bend { parts: vec![(relief, 1.0)] };
        for path in &layer.paths {
            let [w, h] = bead_section(path.role, path.width, layer.height);
            let values = [layer.index as f64, role_code(path.role), w, h, bead as f64];
            polyline(&mut c, &path.points, path.closed, layer.top - h / 2.0, &bend, values)?;
            bead += 1;
        }
    }
    Ok(c)
}

/// The point at `at` and time `t`: the last one added when it is the same.
fn vertex<const E: usize>(
    c: &mut Complex<1, E, 0>,
    last: &mut Option<([f64; 3], f64, u32)>,
    at: [f64; 3],
    t: f64,
) -> Result<u32, ViewError> {
    if let Some((p, s, i)) = *last
        && p == at
        && s == t
    {
        return Ok(i);
    }
    let i = c.point(at, [t])?;
    *last = Some((at, t, i));
    Ok(i)
}

/// The nozzle's moves, with their times, `layer`, `move`, `role`, `width`, `height`, `speed`,
/// `e` and `fan`; `limits` time them as the printer runs them.
pub fn program(program: &Program, limits: &Limits) -> Result<Complex<1, 8, 0>, ViewError> {
    let properties = [
        int("layer"),
        int("move"),
        int("role"),
        float("width"),
        float("height"),
        float("speed"),
        float("e"),
        float("fan"),
    ];
    let mut c = Complex::new([float("t")], properties, []);
    // The move lays no plastic: no role.
    c.comments.push(role_comment() + ", -1 none");
    c.comments.push("move: 0 extrude, 1 travel, 2 wipe, 3 lift".into());
    c.comments.push("t: seconds since the program started, the machine's own G-code not counted".into());
    let times = estimate(program, limits);
    let mut last = None;
    let mut slot = 0usize;
    let mut t0 = 0.0;
    for layer in &program.layers {
        let index = layer.index as f64;
        // After the machine's own G-code the nozzle may be anywhere: nothing is drawn until it
        // is known.
        let mut at: Option<[f64; 3]> = None;
        for m in &layer.moves {
            let t1 = times.get(slot).copied().unwrap_or(t0);
            slot += 1;
            let (Some(from), Some(to)) = (at, m.to) else {
                at = m.to.or(at);
                t0 = t1;
                continue;
            };
            let kind = match (m.bead, m.e) {
                (Some(_), _) => 0.0,
                (None, e) if e < 0.0 => 2.0,
                _ if m.arc.is_some() || (to[0] == from[0] && to[1] == from[1]) => 3.0,
                _ => 1.0,
            };
            let (role, [w, h]) = m.bead.map_or((NO_ROLE, [0.0, 0.0]), |b| {
                (role_code(b.role), bead_section(b.role, b.width, layer.height))
            });
            let values = [index, kind, role, w, h, m.speed, m.e, m.fan];
            // A bead is drawn along its centre line, half its height under the nozzle.
            let down = h / 2.0;
            let n = if m.arc.is_some() {
                let turns = m.arc.map_or(0.0, |a| a.turn.abs() / std::f64::consts::TAU);
                ((chords(m.radius(from)) as f64 * turns).ceil() as usize).max(1)
            } else {
                1
            };
            let shift = |p: [f64; 3]| [p[0], p[1], p[2] - down];
            let mut a = vertex(&mut c, &mut last, shift(from), t0)?;
            for k in 1..=n {
                let s = k as f64 / n as f64;
                // The path ends exactly where the move says.
                let p = if k == n { to } else { m.point(from, s) };
                let b = vertex(&mut c, &mut last, shift(p), t0 + (t1 - t0) * s)?;
                c.segment([a, b], values);
                a = b;
            }
            at = Some(to);
            t0 = t1;
        }
    }
    Ok(c)
}
