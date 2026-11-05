//! The triangles every leaf of a print is linear on.
//!
//! They cover the region where a leaf may bend: the patches' shadows, and around them as far as
//! the leaves need to come back to flat. Their edges hold every facet's edges, so that a leaf
//! following a patch is linear on each of its facets and bends exactly along its creases;
//! facets that overlap on the plate, one patch above another, are cut by each other's edges.
//! All those edges make one subdivision of the plate, cut into triangles about one size
//! (`triangulate_evenly`) that meet edge to edge; past the reach of the leaves, out to the convex
//! hull of the mesh's shadow, a few large triangles cover the plate where every leaf is flat, so
//! that each of the mesh's triangles lies over them (`refine`). Each corner knows the facets it
//! lies on.

use crate::patches::Patch;
use layercraft_geom::{
    GeomError, Point, Region, bounds, disc_offset, encloses, orientation, triangulate_evenly, union,
    union_contours, winding_region,
};
use std::cmp::Ordering;
use std::collections::HashMap;

/// The width of the squares facets are filed by, in triangles.
const FACET_CELLS: f64 = 8.0;

/// A facet, by its patch and its place in it.
pub type FacetId = (usize, usize);

/// The triangles the leaves bend on.
#[derive(Debug, Clone, Default)]
pub struct Triangulation {
    pub points: Vec<Point>,
    /// Counterclockwise index triples into `points`: those the leaves may bend on, then those out
    /// to the mesh's hull, where they are flat.
    pub triangles: Vec<[u32; 3]>,
    /// How many of the triangles the leaves may bend on.
    pub inner: usize,
    /// The facets each corner lies on, inside or on their edges.
    pub on: Vec<Vec<FacetId>>,
}

/// The convex hull of points, counterclockwise, without points on its edges.
pub fn hull(points: &[Point]) -> Vec<Point> {
    let mut sorted: Vec<Point> =
        points.iter().copied().filter(|p| p[0].is_finite() && p[1].is_finite()).collect();
    sorted.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    sorted.dedup();
    if sorted.len() < 3 {
        return sorted;
    }
    // Andrew's monotone chain: the lower hull left to right, the upper right to left.
    let mut out: Vec<Point> = Vec::with_capacity(sorted.len() + 1);
    for pass in [false, true] {
        let start = out.len();
        let mut chain: Box<dyn Iterator<Item = &Point>> =
            if pass { Box::new(sorted.iter().rev()) } else { Box::new(sorted.iter()) };
        for &p in chain.by_ref() {
            while out.len() >= start + 2 {
                let (Some(&a), Some(&b)) = (out.get(out.len() - 2), out.last()) else { break };
                if orientation(a, b, p) == Ordering::Greater {
                    break;
                }
                out.pop();
            }
            out.push(p);
        }
        out.pop();
    }
    out
}

/// A contour's corners, the first again at the end.
fn closed(ring: &[Point]) -> Vec<Point> {
    let mut out = ring.to_vec();
    out.extend(ring.first().copied());
    out
}

/// Whether `p` lies in the region.
fn holds(region: &Region, p: Point) -> bool {
    encloses(
        region
            .iter()
            .flatten()
            .flat_map(|ring| ring.iter().zip(ring.iter().cycle().skip(1)).map(|(a, b)| (*a, *b))),
        p,
    )
}

/// The facets over each of `points`.
fn facets_over(facets: &[(&Region, FacetId)], points: &[Point], width: f64) -> Vec<Vec<FacetId>> {
    let key = |p: Point| [(p[0] / width).floor() as i64, (p[1] / width).floor() as i64];
    let mut filed: HashMap<[i64; 2], Vec<usize>> = HashMap::new();
    for (k, (region, _)) in facets.iter().enumerate() {
        let Some([lo, hi]) = bounds(region) else { continue };
        let ([i0, j0], [i1, j1]) = (key(lo), key(hi));
        for i in i0..=i1 {
            for j in j0..=j1 {
                filed.entry([i, j]).or_default().push(k);
            }
        }
    }
    points
        .iter()
        .map(|&p| {
            filed
                .get(&key(p))
                .into_iter()
                .flatten()
                .filter_map(|&k| facets.get(k))
                .filter(|(region, _)| holds(region, p))
                .map(|(_, id)| *id)
                .collect()
        })
        .collect()
}

/// The triangles about `spacing` mm wide over the patches' facets and the plate around them out
/// to `margin` mm, each facet's edges among their edges, and past them out to `cover`, a convex
/// outline.
pub fn triangulation(
    patches: &[Patch],
    spacing: f64,
    margin: f64,
    cover: &[Point],
) -> Result<Triangulation, GeomError> {
    let facets: Vec<(&Region, FacetId)> = patches
        .iter()
        .enumerate()
        .flat_map(|(p, patch)| patch.facets.iter().enumerate().map(move |(f, facet)| (&facet.region, (p, f))))
        .collect();
    let contours: Vec<Vec<Point>> = facets.iter().flat_map(|(r, _)| r.iter().flatten().cloned()).collect();
    let shadows = winding_region(&contours)?;
    if shadows.is_empty() {
        return Ok(Triangulation::default());
    }
    let reach = disc_offset(&shadows, margin)?;
    // The cover grown by a spacing, so that rounding its corners to the lattice leaves the
    // whole mesh inside.
    let outline = union(&reach, &disc_offset(&union_contours(&[cover.to_vec()])?, spacing)?)?;
    let lines: Vec<Vec<Point>> =
        contours.iter().chain(reach.iter().flatten()).map(|ring| closed(ring)).collect();
    let mesh = triangulate_evenly(&outline, &lines, &|p| holds(&reach, p), spacing)?;
    let over = facets_over(&facets, &mesh.inside, FACET_CELLS * spacing);
    let mut on: Vec<Vec<FacetId>> = vec![Vec::new(); mesh.points.len()];
    for (t, &cell) in mesh.triangles.iter().zip(&mesh.cells) {
        let Some(ids) = over.get(cell as usize) else { continue };
        for &corner in t {
            if let Some(list) = on.get_mut(corner as usize) {
                for id in ids {
                    if !list.contains(id) {
                        list.push(*id);
                    }
                }
            }
        }
    }
    Ok(Triangulation { points: mesh.points, triangles: mesh.triangles, inner: mesh.filled, on })
}
