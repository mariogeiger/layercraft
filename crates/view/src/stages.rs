//! Each stage as an attributed complex, in the printer's frame, in millimetres.
//!
//! A bead's centre line lies half its height under the nozzle: at a layer's mid-height, where
//! the section cuts, or a strand's radius down for a bridge, which hangs round. So the stages of
//! one layer overlay, and a role has the same code in all of them.
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
use layercraft_motion::{Limits, Op, Program, estimate};
use layercraft_path::{Paths, Role};
use layercraft_region::Regions;
use layercraft_section::Sections;

/// The roles, in the order of their codes.
pub const ROLES: [Role; 6] =
    [Role::OuterWall, Role::InnerWall, Role::TopSurface, Role::Solid, Role::Bridge, Role::Sparse];

pub fn role_name(role: Role) -> &'static str {
    match role {
        Role::OuterWall => "outer wall",
        Role::InnerWall => "inner wall",
        Role::TopSurface => "top surface",
        Role::Solid => "solid infill",
        Role::Bridge => "bridge",
        Role::Sparse => "sparse infill",
    }
}

/// The code of a move that lays no plastic.
const NO_ROLE: f64 = -1.0;

fn role_code(role: Role) -> f64 {
    ROLES.iter().position(|r| *r == role).map_or(NO_ROLE, |i| i as f64)
}

fn role_comment() -> String {
    let codes: Vec<String> =
        ROLES.iter().enumerate().map(|(i, r)| format!("{i} {}", role_name(*r))).collect();
    format!("role: {}", codes.join(", "))
}

/// A bead's cross-section, width and height: a layer high, or round for a hanging strand.
fn bead_section(role: Role, width: f64, layer_height: f64) -> [f64; 2] {
    if role == Role::Bridge { [width, width] } else { [width, layer_height] }
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
            let role = role_code(if k == 0 { Role::OuterWall } else { Role::InnerWall });
            for contour in wall.iter().flatten() {
                polyline(&mut c, contour, true, z, [index, role])?;
            }
        }
        let areas = [
            (&layer.top_surface, Role::TopSurface),
            (&layer.solid, Role::Solid),
            (&layer.bridge, Role::Bridge),
            (&layer.bridge_on_sparse, Role::Bridge),
            (&layer.sparse, Role::Sparse),
        ];
        for (area, role) in areas {
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
    c.comments.push(role_comment() + ", -1 none: the move lays no plastic");
    c.comments.push("move: 0 extrude, 1 travel, 2 wipe, 3 lift".into());
    c.comments.push("t: seconds since the program started, the machine's own G-code not counted".into());
    let times = estimate(&program.ops, limits, [0.0; 3]);
    let (mut layer, mut layer_height, mut feature, mut fan) = (-1.0, 0.0, None, 0.0);
    let mut at: Option<[f64; 3]> = None;
    let mut last = None;
    for (i, op) in program.ops.iter().enumerate() {
        let t0 = times.get(i).copied().unwrap_or(0.0);
        let t1 = times.get(i + 1).copied().unwrap_or(t0);
        match op {
            Op::Layer { index, height, .. } => (layer, layer_height) = (*index as f64, *height),
            Op::Feature { role, width } => feature = Some((*role, *width)),
            Op::Fan { speed } => fan = *speed,
            Op::Raw { .. } => at = None,
            Op::Travel { to, speed } | Op::Wipe { to, speed, .. } | Op::Extrude { to, speed, .. } => {
                if let Some(from) = at {
                    let (values, down) = match (op, feature) {
                        (Op::Extrude { e, .. }, Some((role, width))) => {
                            let [w, h] = bead_section(role, width, layer_height);
                            ([layer, 0.0, role_code(role), w, h, *speed, *e, fan], h / 2.0)
                        }
                        (Op::Wipe { e, .. }, _) => ([layer, 2.0, NO_ROLE, 0.0, 0.0, *speed, *e, fan], 0.0),
                        (Op::Extrude { e, .. }, None) => {
                            ([layer, 0.0, NO_ROLE, 0.0, 0.0, *speed, *e, fan], 0.0)
                        }
                        _ => ([layer, 1.0, NO_ROLE, 0.0, 0.0, *speed, 0.0, fan], 0.0),
                    };
                    let a = vertex(&mut c, &mut last, [from[0], from[1], from[2] - down], t0)?;
                    let b = vertex(&mut c, &mut last, [to[0], to[1], to[2] - down], t1)?;
                    c.segment([a, b], values);
                }
                at = Some(*to);
            }
            Op::Lift { z, around, speed } => {
                if let Some(from) = at {
                    let values = [layer, 3.0, NO_ROLE, 0.0, 0.0, *speed, 0.0, fan];
                    let r = around.map_or(0.0, |o| (from[0] - o[0]).hypot(from[1] - o[1]));
                    let n = if r > 0.0 { chords(r) } else { 1 };
                    let start = around.map_or(0.0, |o| (from[1] - o[1]).atan2(from[0] - o[0]));
                    let mut a = vertex(&mut c, &mut last, from, t0)?;
                    for k in 1..=n {
                        let s = k as f64 / n as f64;
                        let height = from[2] + (z - from[2]) * s;
                        let p = match around {
                            // The turn ends exactly over where it started.
                            Some(o) if r > 0.0 && k < n => {
                                let angle = start + std::f64::consts::TAU * s;
                                [o[0] + r * angle.cos(), o[1] + r * angle.sin(), height]
                            }
                            _ => [from[0], from[1], height],
                        };
                        let b = vertex(&mut c, &mut last, p, t0 + (t1 - t0) * s)?;
                        c.segment([a, b], values);
                        a = b;
                    }
                }
                at = at.map(|p| [p[0], p[1], *z]);
            }
            Op::Retract { .. } | Op::Accel { .. } | Op::Progress { .. } => {}
        }
    }
    Ok(c)
}
