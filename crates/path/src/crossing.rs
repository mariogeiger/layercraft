//! Which moves cross a bead already laid at their height. The nozzle travels at the height of the
//! layer's tops, so such a move strikes the bead's shoulder side on; running along a bead it
//! rides on its top, as it does laying the next one. Only crossing a bead's centre line counts:
//! a move touching one where it starts or ends, as from a line that overlaps its wall, crosses
//! nothing.

use crate::Path;
use layercraft_geom::{Point, crosses};

/// One segment of a bead.
pub type Segment = (Point, Point);

/// The segments of `path`'s bead, a loop's closing one with them.
pub fn segments(path: &Path) -> Vec<Segment> {
    let closing = path.points.first().filter(|_| path.closed);
    let ends = path.points.iter().copied().chain(closing.copied());
    ends.clone().zip(ends.skip(1)).collect()
}

/// Laid segments filed by the horizontal bands their heights span, about the square root of them
/// in each band; each carries when it was laid. The bands only find the segments near a move: what
/// crosses what does not depend on them.
pub struct Laid {
    foot: f64,
    height: f64,
    bands: Vec<Vec<(Point, Point, usize)>>,
}

impl Laid {
    /// Files `segments`, each with when it was laid.
    pub fn new(segments: &[(Point, Point, usize)]) -> Laid {
        let (low, high) = segments
            .iter()
            .flat_map(|(a, b, _)| [a[1], b[1]])
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), y| (low.min(y), high.max(y)));
        let count = (segments.len() as f64).sqrt().ceil().max(1.0);
        let height = if high > low { (high - low) / count } else { 1.0 };
        let foot = if low.is_finite() { low } else { 0.0 };
        let mut laid = Laid { foot, height, bands: vec![Vec::new(); count as usize] };
        for &s in segments {
            let (first, last) = (laid.band(s.0[1].min(s.1[1])), laid.band(s.0[1].max(s.1[1])));
            laid.bands.iter_mut().take(last + 1).skip(first).for_each(|band| band.push(s));
        }
        laid
    }

    fn band(&self, y: f64) -> usize {
        let k = ((y - self.foot) / self.height).floor();
        if k >= 1.0 { (k as usize).min(self.bands.len().saturating_sub(1)) } else { 0 }
    }

    /// Whether the move from `a` to `b` crosses a segment laid before `now`.
    pub fn crossed(&self, a: Point, b: Point, now: usize) -> bool {
        let (first, last) = (self.band(a[1].min(b[1])), self.band(a[1].max(b[1])));
        let (left, right) = (a[0].min(b[0]), a[0].max(b[0]));
        self.bands.get(first..=last).into_iter().flatten().flatten().any(|&(c, d, when)| {
            when < now && c[0].max(d[0]) >= left && c[0].min(d[0]) <= right && crosses(a, b, c, d)
        })
    }
}

/// What a tour's moves stay off: the beads laid before it.
pub struct Cover<'a> {
    pub laid: &'a Laid,
}

impl Cover<'_> {
    /// Whether a move from `a` to `b` crosses none of them.
    pub fn stays(&self, a: Point, b: Point) -> bool {
        !self.laid.crossed(a, b, usize::MAX)
    }

    /// How long a move from `a` that stays can be: any length, nothing forbids a length.
    pub fn reach(&self, _a: Point) -> f64 {
        f64::INFINITY
    }
}

/// Marks the beads of a layer, in printing order, reached by a move that crosses a bead laid
/// before it: one of `paths`, or one of `earlier`, laid at this height by the passes before.
pub fn mark(mut paths: Vec<Path>, earlier: &[Segment]) -> Vec<Path> {
    let mut all: Vec<(Point, Point, usize)> = earlier.iter().map(|&(a, b)| (a, b, 0)).collect();
    for (k, path) in paths.iter().enumerate() {
        all.extend(segments(path).into_iter().map(|(a, b)| (a, b, k + 1)));
    }
    let laid = Laid::new(&all);
    let mut end: Option<Point> = None;
    for (k, path) in paths.iter_mut().enumerate() {
        if let (Some(a), Some(&b)) = (end, path.points.first()) {
            path.travel_crosses = laid.crossed(a, b, k + 1);
        }
        end = path.end();
    }
    paths
}
