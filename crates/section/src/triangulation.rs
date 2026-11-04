//! The triangles every leaf of a print is linear on.
//!
//! They cover the region where a leaf may bend: the patches' shadows, and around them as far as
//! the leaves need to come back to flat. Their edges hold every facet's edges, so that a leaf
//! following a patch is linear on each of its facets and bends exactly along its creases;
//! facets that overlap on the plate, one patch above another, are cut by each other's edges.
//! Each piece is cut into triangles about one size (`triangulate_evenly`), with the same corners
//! along shared edges, so that they meet edge to edge; past that, out to the convex hull of the
//! mesh's shadow, a few large triangles cover the plate where every leaf is flat, so that each
//! of the mesh's triangles lies over them (`refine`). Each corner knows the facets it lies on.

use crate::patches::Patch;
use layercraft_geom::{
    GeomError, Point, Region, area, bounds, difference, disc_offset, intersection, orientation,
    triangulate_evenly, union_contours, winding_region,
};
use std::cmp::Ordering;

/// Pieces thinner than this, in mm², are the rounding of shared edges, not overlaps.
const SLIVER_MM2: f64 = 1e-9;

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

fn overlap(a: &Region, b: &Region) -> bool {
    match (bounds(a), bounds(b)) {
        (Some([alo, ahi]), Some([blo, bhi])) => {
            alo[0] <= bhi[0] && blo[0] <= ahi[0] && alo[1] <= bhi[1] && blo[1] <= ahi[1]
        }
        _ => false,
    }
}

/// The pieces the facets cut each other into where they overlap, each with the facets over it:
/// the facets themselves when their areas add up to their union's, none overlapping another.
fn pieces(patches: &[Patch]) -> Result<Vec<(Region, Vec<FacetId>)>, GeomError> {
    let facets: Vec<(&Region, FacetId)> = patches
        .iter()
        .enumerate()
        .flat_map(|(p, patch)| patch.facets.iter().enumerate().map(move |(f, facet)| (&facet.region, (p, f))))
        .collect();
    let total: f64 = facets.iter().map(|(r, _)| area(r)).sum();
    let contours: Vec<Vec<Point>> = facets.iter().flat_map(|(r, _)| r.iter().flatten().cloned()).collect();
    if total - area(&winding_region(&contours)?) <= SLIVER_MM2 * facets.len() as f64 {
        return Ok(facets.into_iter().map(|(r, id)| (r.clone(), vec![id])).collect());
    }
    let mut out: Vec<(Region, Vec<FacetId>)> = Vec::new();
    for (region, id) in facets {
        let mut rest = region.clone();
        let mut next = Vec::with_capacity(out.len() + 1);
        for (piece, ids) in out {
            if rest.is_empty() || !overlap(&piece, &rest) {
                next.push((piece, ids));
                continue;
            }
            let common = intersection(&piece, &rest)?;
            if area(&common) <= SLIVER_MM2 {
                next.push((piece, ids));
                continue;
            }
            let outside = difference(&piece, &rest)?;
            rest = difference(&rest, &piece)?;
            if area(&outside) > SLIVER_MM2 {
                next.push((outside, ids.clone()));
            }
            let mut both = ids;
            both.push(id);
            next.push((common, both));
        }
        if area(&rest) > SLIVER_MM2 {
            next.push((rest, vec![id]));
        }
        out = next;
    }
    Ok(out)
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
    let pieces = pieces(patches)?;
    let contours: Vec<Vec<Point>> = pieces.iter().flat_map(|(r, _)| r.iter().flatten().cloned()).collect();
    let shadows = winding_region(&contours)?;
    if shadows.is_empty() {
        return Ok(Triangulation::default());
    }
    let reach = disc_offset(&shadows, margin)?;
    let around = difference(&reach, &shadows)?;
    // The cover grown by a spacing, so that rounding its corners to the lattice leaves the
    // whole mesh inside.
    let beyond = difference(&disc_offset(&union_contours(&[cover.to_vec()])?, spacing)?, &reach)?;
    let mut cells: Vec<(Region, bool)> = pieces.iter().map(|(r, _)| (r.clone(), true)).collect();
    cells.push((around, true));
    let outer = cells.len() as u32;
    cells.push((beyond, false));
    let mesh = triangulate_evenly(&cells, spacing)?;
    let inner = mesh.cells.iter().take_while(|&&c| c < outer).count();
    let mut on: Vec<Vec<FacetId>> = vec![Vec::new(); mesh.points.len()];
    for (t, &cell) in mesh.triangles.iter().zip(&mesh.cells) {
        let Some((_, ids)) = pieces.get(cell as usize) else { continue };
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
    Ok(Triangulation { points: mesh.points, triangles: mesh.triangles, inner, on })
}
