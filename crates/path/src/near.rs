//! The point of a set nearest another, and every point within a distance of it: a k-d tree, each
//! node splitting its points at their median along the longer side of their bounds. Points can
//! be taken out, and a search looks only into the parts that still hold some. Of points as near,
//! the lower number comes first, so every answer is the same however the tree splits.

use layercraft_geom::Point;

fn distance2(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// Numbered points.
pub(crate) struct Near {
    /// The points and their numbers in the tree's order: the node of the range `lo..hi` lies at
    /// its middle, the half of the range below it before, the half above after.
    points: Vec<(Point, usize)>,
    /// Whether each node splits across y rather than x.
    across_y: Vec<bool>,
    /// How many points each node's range still holds.
    held: Vec<usize>,
    /// Whether each node's own point is still held.
    here: Vec<bool>,
    /// Where each number's point lies in `points`.
    place: Vec<Option<usize>>,
}

impl Near {
    /// The tree of `points`, each with its number; points with no finite place are left out.
    pub fn new(points: impl IntoIterator<Item = (Point, usize)>) -> Near {
        let mut points: Vec<(Point, usize)> =
            points.into_iter().filter(|(p, _)| p[0].is_finite() && p[1].is_finite()).collect();
        let n = points.len();
        let (mut across_y, mut held) = (vec![false; n], vec![0; n]);
        let mut ranges = vec![(0, n)];
        while let Some((lo, hi)) = ranges.pop() {
            let mid = lo + (hi - lo) / 2;
            let Some(range) = points.get_mut(lo..hi).filter(|r| !r.is_empty()) else { continue };
            let (low, high) =
                range.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(l, h), (p, _)| {
                    ([l[0].min(p[0]), l[1].min(p[1])], [h[0].max(p[0]), h[1].max(p[1])])
                });
            let y = high[1] - low[1] > high[0] - low[0];
            let key = |p: &(Point, usize)| if y { p.0[1] } else { p.0[0] };
            range.select_nth_unstable_by(mid - lo, |a, b| key(a).total_cmp(&key(b)).then(a.1.cmp(&b.1)));
            if let (Some(s), Some(h)) = (across_y.get_mut(mid), held.get_mut(mid)) {
                (*s, *h) = (y, hi - lo);
            }
            ranges.extend([(lo, mid), (mid + 1, hi)]);
        }
        let mut place = vec![None; points.iter().map(|(_, k)| k + 1).max().unwrap_or(0)];
        for (at, (_, k)) in points.iter().enumerate() {
            if let Some(slot) = place.get_mut(*k) {
                *slot = Some(at);
            }
        }
        Near { points, across_y, held, here: vec![true; n], place }
    }

    /// Takes out the point numbered `number`.
    pub fn take(&mut self, number: usize) {
        let Some(target) = self.place.get_mut(number).and_then(Option::take) else { return };
        let (mut lo, mut hi) = (0, self.points.len());
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if let Some(h) = self.held.get_mut(mid) {
                *h = h.saturating_sub(1);
            }
            match target.cmp(&mid) {
                std::cmp::Ordering::Less => hi = mid,
                std::cmp::Ordering::Greater => lo = mid + 1,
                std::cmp::Ordering::Equal => break,
            }
        }
        if let Some(here) = self.here.get_mut(target) {
            *here = false;
        }
    }

    /// Calls `visit` with each node, its range and its point, in an order that searches near
    /// `p` first, looking into a range only when `visit` returns a squared distance no farther
    /// than the range lies from `p`.
    fn search(&self, p: Point, mut visit: impl FnMut(usize, Point) -> f64) {
        let mut ranges = vec![(0.0, 0, self.points.len())];
        while let Some((bound, lo, hi)) = ranges.pop() {
            let mid = lo + (hi - lo) / 2;
            let (Some(&(q, number)), Some(&held)) = (self.points.get(mid), self.held.get(mid)) else {
                continue;
            };
            if lo >= hi || held == 0 {
                continue;
            }
            let reach =
                if self.here.get(mid) == Some(&true) { visit(number, q) } else { visit(usize::MAX, q) };
            if bound > reach {
                continue;
            }
            let y = self.across_y.get(mid) == Some(&true);
            let delta = if y { p[1] - q[1] } else { p[0] - q[0] };
            let (near, far) =
                if delta < 0.0 { ((lo, mid), (mid + 1, hi)) } else { ((mid + 1, hi), (lo, mid)) };
            if delta * delta <= reach {
                ranges.push((delta * delta, far.0, far.1));
            }
            ranges.push((bound, near.0, near.1));
        }
    }

    /// The number of the point still held nearest `p`, the lowest of those as near.
    pub fn nearest(&self, p: Point) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        self.search(p, |number, q| {
            let d = distance2(p, q);
            if number != usize::MAX && best.is_none_or(|(b, k)| d < b || (d == b && number < k)) {
                best = Some((d, number));
            }
            best.map_or(f64::INFINITY, |(b, _)| b)
        });
        best.map(|(_, number)| number)
    }

    /// The squared distances and numbers of the points still held no farther than `r` from
    /// `p`, nearest first, the lower number first of those as near.
    pub fn within(&self, p: Point, r: f64, found: &mut Vec<(f64, usize)>) {
        found.clear();
        let r2 = r * r;
        self.search(p, |number, q| {
            let d = distance2(p, q);
            if number != usize::MAX && d <= r2 {
                found.push((d, number));
            }
            r2
        });
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    }
}
