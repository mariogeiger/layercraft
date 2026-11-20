//! Which moves cross a bead already laid at their height. The nozzle travels at the height of the
//! layer's tops, so such a move strikes the bead's shoulder side on; running along a bead it
//! rides on its top, as it does laying the next one. Only crossing a bead's centre line counts:
//! a move touching one where it starts or ends, as from a line that overlaps its wall, crosses
//! nothing. The beads are filed as they are laid, by the horizontal bands their heights span
//! (`Banded`), and each move is marked as its bead is laid.

use crate::Path;
use layercraft_geom::{Banded, Point, crosses};

/// One segment of a bead.
pub type Segment = (Point, Point);

/// The segments of `path`'s bead, a loop's closing one with them.
pub fn segments(path: &Path) -> Vec<Segment> {
    let closing = path.points.first().filter(|_| path.closed);
    let ends = path.points.iter().copied().chain(closing.copied());
    ends.clone().zip(ends.skip(1)).collect()
}

/// The beads laid so far at a height, and where the last one laid left the nozzle.
pub struct Laid {
    segments: Banded<Segment>,
    end: Option<Point>,
}

impl Laid {
    /// No bead laid yet, filed for about `count` segments between the heights `low` and `high`.
    pub fn new(low: f64, high: f64, count: usize) -> Laid {
        Laid { segments: Banded::new(low, high, count), end: None }
    }

    /// Files the segment `a`–`b` of a bead laid before.
    pub fn add(&mut self, a: Point, b: Point) {
        self.segments.insert(a[1].min(b[1]), a[1].max(b[1]), (a, b));
    }

    /// Lays `path`: marks it when the move to it from the last bead laid crosses a bead, then
    /// files it.
    pub fn lay(&mut self, path: &mut Path) {
        if let (Some(a), Some(&b)) = (self.end, path.points.first()) {
            path.travel_crosses = self.crosses(a, b);
        }
        self.end = path.end();
        for (a, b) in segments(path) {
            self.add(a, b);
        }
    }

    /// Whether the move from `a` to `b` crosses a bead laid.
    pub fn crosses(&self, a: Point, b: Point) -> bool {
        let (left, right) = (a[0].min(b[0]), a[0].max(b[0]));
        self.segments
            .between(a[1].min(b[1]), a[1].max(b[1]))
            .any(|&(c, d)| c[0].max(d[0]) >= left && c[0].min(d[0]) <= right && crosses(a, b, c, d))
    }
}
