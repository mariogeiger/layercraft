//! The convex hull of points in the plane, every turn decided exactly (`orientation`).
//!
//! Andrew's monotone chain: the points sorted by their coordinates, the lower hull walked one way
//! and the upper the other, each point dropped while it and the two before do not turn left.

use crate::{Point, orientation};
use std::cmp::Ordering;

/// The convex hull of the finite points, counterclockwise, without points on its edges: one point
/// or two where they are one or on a line.
pub fn convex_hull(points: &[Point]) -> Vec<Point> {
    let mut sorted: Vec<Point> =
        points.iter().copied().filter(|p| p[0].is_finite() && p[1].is_finite()).collect();
    sorted.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    sorted.dedup();
    if sorted.len() < 3 {
        return sorted;
    }
    let half = |points: &mut dyn Iterator<Item = &Point>| {
        let mut chain: Vec<Point> = Vec::new();
        for &p in points {
            while let [.., a, b] = chain.as_slice()
                && orientation(*a, *b, p) != Ordering::Greater
            {
                chain.pop();
            }
            chain.push(p);
        }
        chain.pop();
        chain
    };
    let mut hull = half(&mut sorted.iter());
    hull.extend(half(&mut sorted.iter().rev()));
    hull
}
