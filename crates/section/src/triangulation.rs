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
    Buckets, GeomError, Point, Region, bounds, disc_offset, edges, encloses, triangulate_evenly, union,
    union_contours, winding_region,
};

/// The width of the buckets facets are filed by, in triangles.
const FACET_BUCKET: f64 = 8.0;

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

/// A contour's corners, the first again at the end.
fn closed(ring: &[Point]) -> Vec<Point> {
    let mut out = ring.to_vec();
    out.extend(ring.first().copied());
    out
}

/// Whether `p` lies in the region.
fn holds(region: &Region, p: Point) -> bool {
    encloses(region.iter().flatten().flat_map(|ring| edges(ring)), p)
}

/// The facets holding each of `points`, found through buckets `width` wide.
fn facets_holding(facets: &[(&Region, FacetId)], points: &[Point], width: f64) -> Vec<Vec<FacetId>> {
    let boxes: Vec<Option<[Point; 2]>> = facets.iter().map(|(region, _)| bounds(region)).collect();
    let filed = Buckets::of_boxes(&boxes, width);
    points
        .iter()
        .map(|&p| {
            filed
                .near(p)
                .iter()
                .filter_map(|&k| facets.get(k as usize))
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
    let over = facets_holding(&facets, &mesh.inside, FACET_BUCKET * spacing);
    let mut on: Vec<Vec<FacetId>> = vec![Vec::new(); mesh.points.len()];
    for (t, &face) in mesh.triangles.iter().zip(&mesh.faces) {
        let Some(ids) = over.get(face as usize) else { continue };
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
