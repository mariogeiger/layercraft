//! Pictures of the stages: any stage as an attributed complex written as PLY, for SolidCraft or
//! any viewer to colour and filter by its properties; the toolpaths as a SolidCraft script
//! (`script`); and the thumbnail shown on the printer's screen.
//!
//! The thumbnail paints the beads from the bottom layer up, seen from above, each layer, all its
//! passes, lighter than the one below it; an arc as chords within `THUMBNAIL_SAG_MM` of it.

mod ply;
mod script;
mod stages;
#[cfg(test)]
mod tests;

pub use ply::{Complex, MAX_POINTS, Property, PropertyKind, Table, ViewError, float, int};
pub use script::solidcraft_script;
pub use stages::{mesh, paths, program, regions, role_name, sections};

use layercraft_motion::{Bead, Program};
use layercraft_package::Image;
use layercraft_path::Role;

/// The farthest a chord of an arc strays from it on the thumbnail, in mm: a tenth of a pixel of the
/// largest thumbnail across a 250 mm plate.
const THUMBNAIL_SAG_MM: f64 = 0.05;

/// A role's colour: strands green; walls orange outside, yellow inside; dense lines red on
/// top, purple elsewhere; sparse lines blue; supports grey, their tops lighter.
fn colour(role: Role) -> [f64; 3] {
    use layercraft_path::{Face, Kind};
    match (role.kind, role.face) {
        _ if role.strand() => [0.25, 0.75, 0.45],
        (Kind::Wall, Face::Inside) => [0.95, 0.80, 0.25],
        (Kind::Wall, _) => [0.95, 0.55, 0.15],
        (Kind::Solid, Face::Top) => [0.90, 0.25, 0.25],
        (Kind::Solid, _) => [0.65, 0.30, 0.75],
        (Kind::Sparse, _) => [0.30, 0.60, 0.85],
        (Kind::Support, Face::Top) => [0.80, 0.80, 0.80],
        (Kind::Support, _) => [0.55, 0.55, 0.55],
    }
}

/// A piece of bead across the plate: from, to, and the bead.
type Stroke = ([f64; 2], [f64; 2], Bead);

/// The beads of a program as segments across the plate, layer by layer, arcs as chords.
fn beads(program: &Program) -> Vec<Vec<Stroke>> {
    program
        .layers()
        .map(|layer| {
            let mut out = Vec::new();
            for pass in layer {
                // After the machine's own G-code the nozzle may be anywhere.
                let mut at: Option<[f64; 3]> = None;
                for m in &pass.moves {
                    if let (Some(from), Some(to), Some(bead)) = (at, m.to, m.bead) {
                        let n = m.chords(from, THUMBNAIL_SAG_MM);
                        let mut a = [from[0], from[1]];
                        for k in 1..=n {
                            let p = if k == n { to } else { m.point(from, k as f64 / n as f64) };
                            out.push((a, [p[0], p[1]], bead));
                            a = [p[0], p[1]];
                        }
                    }
                    at = m.to.or(at);
                }
            }
            out
        })
        .collect()
}

/// A `size` × `size` picture of the beads seen from above, framing the print.
pub fn thumbnail(program: &Program, size: u32) -> Image {
    let size = size.clamp(1, 4096);
    let mut rgba = vec![0u8; size as usize * size as usize * 4];
    let layers = beads(program);
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in layers.iter().flatten().flat_map(|(a, b, _)| [a, b]) {
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
    let count = layers.len().max(1) as f64;
    for (k, layer) in layers.iter().enumerate() {
        let light = 0.55 + 0.45 * (k as f64 + 1.0) / count;
        for (a, b, bead) in layer {
            let c = colour(bead.role).map(|v| (v * light * 255.0).round().clamp(0.0, 255.0) as u8);
            let radius = (bead.width * scale / 2.0).max(0.5);
            stroke(&mut rgba, size, pixel(a), pixel(b), radius, c);
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
