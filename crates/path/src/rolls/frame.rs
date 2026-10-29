//! A part's own frame: the centroid of its areas and their principal axes, which turn and move
//! with the part. The axis of greatest spread is the first, pointing the way its areas skew
//! (their third moment along it positive); the second is a quarter turn from it. Areas spread
//! alike every way keep the plate's axes, between whose two ways skew alone decides.
//!
//! The moments are Green's sums over each contour's edges, outer contours counting positive and
//! holes negative: `∫∫ x^p y^q` from the vertices exactly.

use layercraft_geom::{Point, Region};

/// The frame: where its origin lies on the plate, and its first axis, a unit vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Frame {
    origin: Point,
    axis: [f64; 2],
}

/// The moments of an area about a point: `[∫1, ∫x, ∫y, ∫x², ∫xy, ∫y²]`.
fn moments(region: &Region, about: Point, into: &mut [f64; 6], weight: f64) {
    for contour in region.iter().flatten() {
        for (i, a) in contour.iter().enumerate() {
            let Some(b) = contour.get((i + 1) % contour.len()) else { continue };
            let (x0, y0, x1, y1) = (a[0] - about[0], a[1] - about[1], b[0] - about[0], b[1] - about[1]);
            let cross = weight * (x0 * y1 - x1 * y0);
            let sums = [
                cross / 2.0,
                cross * (x0 + x1) / 6.0,
                cross * (y0 + y1) / 6.0,
                cross * (x0 * x0 + x0 * x1 + x1 * x1) / 12.0,
                cross * (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) / 24.0,
                cross * (y0 * y0 + y0 * y1 + y1 * y1) / 12.0,
            ];
            for (m, s) in into.iter_mut().zip(sums) {
                *m += s;
            }
        }
    }
}

/// `∫∫ x³` over an area, in the frame of `origin` and `axis`.
fn skew(region: &Region, origin: Point, axis: [f64; 2], weight: f64) -> f64 {
    let along = |p: &Point| (p[0] - origin[0]) * axis[0] + (p[1] - origin[1]) * axis[1];
    let across = |p: &Point| -(p[0] - origin[0]) * axis[1] + (p[1] - origin[1]) * axis[0];
    let mut sum = 0.0;
    for contour in region.iter().flatten() {
        for (i, a) in contour.iter().enumerate() {
            let Some(b) = contour.get((i + 1) % contour.len()) else { continue };
            let (x0, y0, x1, y1) = (along(a), across(a), along(b), across(b));
            sum += weight * (x0 * y1 - x1 * y0) * (x0 + x1) * (x0 * x0 + x1 * x1) / 20.0;
        }
    }
    sum
}

impl Frame {
    /// The frame of areas, each weighing what is given with it (a layer's height); the plate's
    /// own for no area.
    pub fn of<'a>(areas: impl Iterator<Item = (&'a Region, f64)> + Clone) -> Frame {
        let about = areas.clone().find_map(|(r, _)| r.iter().flatten().flatten().next().copied());
        let Some(about) = about else { return Frame { origin: [0.0, 0.0], axis: [1.0, 0.0] } };
        let mut m = [0.0; 6];
        for (region, weight) in areas.clone() {
            moments(region, about, &mut m, weight);
        }
        let [a, sx, sy, sxx, sxy, syy] = m;
        if a.is_nan() || a <= 0.0 {
            return Frame { origin: about, axis: [1.0, 0.0] };
        }
        let (cx, cy) = (sx / a, sy / a);
        let (vxx, vxy, vyy) = (sxx / a - cx * cx, sxy / a - cx * cy, syy / a - cy * cy);
        let origin = [about[0] + cx, about[1] + cy];
        // The axis of the greater spread; the plate's when the spread is alike every way.
        let anisotropy = (vxx - vyy).hypot(2.0 * vxy);
        let angle = if anisotropy > 1e-9 * (vxx + vyy) { 0.5 * (2.0 * vxy).atan2(vxx - vyy) } else { 0.0 };
        let axis = [angle.cos(), angle.sin()];
        let skewed: f64 = areas.map(|(region, weight)| skew(region, origin, axis, weight)).sum();
        let axis = if skewed < 0.0 { [-axis[0], -axis[1]] } else { axis };
        Frame { origin, axis }
    }

    /// A point of the plate in the frame.
    pub fn local(&self, p: Point) -> Point {
        let (x, y) = (p[0] - self.origin[0], p[1] - self.origin[1]);
        [x * self.axis[0] + y * self.axis[1], -x * self.axis[1] + y * self.axis[0]]
    }

    /// A point of the frame on the plate.
    pub fn plate(&self, q: Point) -> Point {
        [
            self.origin[0] + q[0] * self.axis[0] - q[1] * self.axis[1],
            self.origin[1] + q[0] * self.axis[1] + q[1] * self.axis[0],
        ]
    }
}
