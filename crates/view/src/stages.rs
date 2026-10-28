//! Each stage as an attributed complex, in the printer's frame, in millimetres.
//!
//! A bead's centre line lies half its height under the nozzle: at a layer's mid-height, where
//! the section cuts, or a strand's radius down for a bridge, which hangs round. So the stages of
//! one layer overlay, and a role has the same code in all of them. A comment names the values of
//! each label, `role: 0 outer wall, 1 inner wall, …`, for a viewer's legend.
//!
//! - mesh: the bodies' triangles, with their `body`;
//! - sections: each layer's section, triangulated, with its `layer`;
//! - regions: the walls' centre lines as segments, and the areas they enclose (top surface,
//!   solid, bridges, sparse) triangulated, with `layer` and `role`;
//! - paths: each bead's centre line, with `layer`, `role`, `width`, `height` and `bead`, its
//!   place in the printing order;
//! - program: the nozzle's moves, each point with its time `t`, each segment with `layer`,
//!   `move`, `role`, `width`, `height`, `speed`, `e` and `fan`. A move that lays plastic is
//!   drawn along the bead it lays; the others where the nozzle goes, a helical lift as chords.
//!   After the machine's own G-code the nozzle may be anywhere: nothing is drawn until it is known.

use crate::ply::{Complex, ViewError, float, int};
use layercraft_geom::{Point, Region, triangulate};
use layercraft_mesh::Mesh;
use layercraft_motion::{Limits, Program, estimate};
use layercraft_path::{Paths, Role};
use layercraft_region::Regions;
use layercraft_section::Sections;

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

/// Adds `region`'s triangles at height `z`.
fn fill<const E: usize, const F: usize>(
    c: &mut Complex<0, E, F>,
    region: &Region,
    z: f64,
    values: [f64; F],
) -> Result<(), ViewError> {
    let (points, triangles) = triangulate(region).map_err(ViewError::Geometry)?;
    let first = c.points.rows.len() as u32;
    for p in &points {
        c.point([p[0], p[1], z], [])?;
    }
    for t in triangles {
        c.triangle(t.map(|i| first + i), values);
    }
    Ok(())
}

/// Adds a polyline at height `z`, back to its start when `closed`.
fn polyline<const E: usize, const F: usize>(
    c: &mut Complex<0, E, F>,
    points: &[Point],
    closed: bool,
    z: f64,
    values: [f64; E],
) -> Result<(), ViewError> {
    let mut ids = Vec::with_capacity(points.len());
    for p in points {
        ids.push(c.point([p[0], p[1], z], [])?);
    }
    if closed && ids.len() >= 3 {
        ids.extend(ids.first().copied());
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
    for layer in &sections.layers {
        fill(&mut c, &layer.region, layer.top - layer.height / 2.0, [layer.index as f64])?;
    }
    Ok(c)
}

/// The walls' centre lines and the areas inside them, with `layer` and `role`.
pub fn regions(regions: &Regions) -> Result<Complex<0, 2, 2>, ViewError> {
    let mut c = Complex::new([], [int("layer"), int("role")], [int("layer"), int("role")]);
    c.comments.push(role_comment());
    for layer in &regions.layers {
        let (z, index) = (layer.top - layer.height / 2.0, layer.index as f64);
        for (k, wall) in layer.walls.iter().enumerate() {
            let role = role_code(if k == 0 { Role::OUTER_WALL } else { Role::INNER_WALL });
            for contour in wall.iter().flatten() {
                polyline(&mut c, contour, true, z, [index, role])?;
            }
        }
        let solid = if layer.index == 0 { Role::BOTTOM_SURFACE } else { Role::SOLID };
        let roles = [Role::TOP_SURFACE, solid, Role::BRIDGE, Role::INTERNAL_BRIDGE, Role::SPARSE];
        for (area, role) in layer.infill().into_iter().zip(roles) {
            fill(&mut c, area, z, [index, role_code(role)])?;
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
    for layer in &paths.layers {
        for path in &layer.paths {
            let [w, h] = bead_section(path.role, path.width, layer.height);
            let values = [layer.index as f64, role_code(path.role), w, h, bead as f64];
            polyline(&mut c, &path.points, path.closed, layer.top - h / 2.0, values)?;
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
