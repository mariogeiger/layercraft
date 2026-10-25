//! Pictures of the toolpaths: a SolidCraft script that draws them as sketches, and the
//! thumbnail shown on the printer's screen.
//!
//! The script holds one sketch per layer and role, at the layer's height and named
//! `L012 outer wall`, so layers and roles can be shown, hidden and measured in SolidCraft.
//! The thumbnail paints the beads from the bottom layer up, seen from above, each layer
//! lighter than the one below it.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use layercraft_package::Image;
use layercraft_path::{Paths, Role};
use serde_json::{Value, json};

fn role_name(role: Role) -> &'static str {
    match role {
        Role::OuterWall => "outer wall",
        Role::InnerWall => "inner wall",
        Role::TopSurface => "top surface",
        Role::Solid => "solid infill",
        Role::Bridge => "bridge",
        Role::Sparse => "sparse infill",
    }
}

const ROLES: [Role; 6] =
    [Role::OuterWall, Role::InnerWall, Role::TopSurface, Role::Solid, Role::Bridge, Role::Sparse];

/// A SolidCraft script drawing every bead's centre line, layers `from..to`.
pub fn solidcraft_script(paths: &Paths, layers: std::ops::Range<usize>) -> Value {
    let mut commands = Vec::new();
    for layer in paths.layers.iter().filter(|l| layers.contains(&l.index)) {
        for role in ROLES {
            let beads: Vec<_> =
                layer.paths.iter().filter(|p| p.role == role && p.points.len() >= 2).collect();
            if beads.is_empty() {
                continue;
            }
            let name = format!("L{:03} {}", layer.index, role_name(role));
            commands.push(json!({"command": "sketch.create", "params": {
                "plane": {"origin": [0.0, 0.0, layer.top], "x_dir": [1.0, 0.0, 0.0], "y_dir": [0.0, 1.0, 0.0]},
                "name": name,
            }}));
            for bead in beads {
                commands.push(json!({"command": "sketch.line", "params": {
                    "points": bead.points, "closed": bead.closed,
                }}));
            }
            commands.push(json!({"command": "sketch.finish", "params": {}}));
        }
    }
    json!({ "commands": commands })
}

fn colour(role: Role) -> [f64; 3] {
    match role {
        Role::OuterWall => [0.95, 0.55, 0.15],
        Role::InnerWall => [0.95, 0.80, 0.25],
        Role::TopSurface => [0.90, 0.25, 0.25],
        Role::Solid => [0.65, 0.30, 0.75],
        Role::Bridge => [0.25, 0.75, 0.45],
        Role::Sparse => [0.30, 0.60, 0.85],
    }
}

/// A `size` × `size` picture of the beads seen from above, framing the print.
pub fn thumbnail(paths: &Paths, size: u32) -> Image {
    let size = size.clamp(1, 4096);
    let mut rgba = vec![0u8; size as usize * size as usize * 4];
    let points = paths.layers.iter().flat_map(|l| l.paths.iter()).flat_map(|p| p.points.iter());
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in points {
        lo = [lo[0].min(p[0]), lo[1].min(p[1])];
        hi = [hi[0].max(p[0]), hi[1].max(p[1])];
    }
    if lo[0] > hi[0] {
        return Image { width: size, height: size, rgba };
    }
    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-3) * 1.1;
    let centre = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let scale = size as f64 / span;
    let pixel = |p: &[f64; 2]| {
        [(p[0] - centre[0]) * scale + size as f64 / 2.0, size as f64 / 2.0 - (p[1] - centre[1]) * scale]
    };
    let count = paths.layers.len().max(1) as f64;
    for (k, layer) in paths.layers.iter().enumerate() {
        let light = 0.55 + 0.45 * (k as f64 + 1.0) / count;
        for path in &layer.paths {
            let c = colour(path.role).map(|v| (v * light * 255.0).round().clamp(0.0, 255.0) as u8);
            let radius = (path.width * scale / 2.0).max(0.5);
            let mut pts: Vec<[f64; 2]> = path.points.iter().map(pixel).collect();
            if path.closed
                && let Some(&first) = pts.first()
            {
                pts.push(first);
            }
            for w in pts.windows(2) {
                if let [a, b] = w {
                    stroke(&mut rgba, size, *a, *b, radius, c);
                }
            }
        }
    }
    Image { width: size, height: size, rgba }
}

/// Paints the pixels within `radius` of the segment `a`–`b`.
fn stroke(rgba: &mut [u8], size: u32, a: [f64; 2], b: [f64; 2], radius: f64, c: [u8; 3]) {
    let (x0, x1) =
        ((a[0].min(b[0]) - radius).floor().max(0.0), (a[0].max(b[0]) + radius).ceil().min(size as f64 - 1.0));
    let (y0, y1) =
        ((a[1].min(b[1]) - radius).floor().max(0.0), (a[1].max(b[1]) + radius).ceil().min(size as f64 - 1.0));
    if !(x0 <= x1 && y0 <= y1) {
        return;
    }
    let d = [b[0] - a[0], b[1] - a[1]];
    let len2 = d[0] * d[0] + d[1] * d[1];
    for y in y0 as u32..=y1 as u32 {
        for x in x0 as u32..=x1 as u32 {
            let p = [x as f64 + 0.5 - a[0], y as f64 + 0.5 - a[1]];
            let t = if len2 > 0.0 { ((p[0] * d[0] + p[1] * d[1]) / len2).clamp(0.0, 1.0) } else { 0.0 };
            let q = [p[0] - t * d[0], p[1] - t * d[1]];
            if q[0] * q[0] + q[1] * q[1] <= radius * radius {
                let i = (y as usize * size as usize + x as usize) * 4;
                if let Some(px) = rgba.get_mut(i..i + 4) {
                    px.copy_from_slice(&[c[0], c[1], c[2], 255]);
                }
            }
        }
    }
}
