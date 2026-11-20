//! Finding the boxes of a set near a point, in a box or along a segment, through buckets.
//!
//! Square buckets cover the boxes, each listing the boxes that meet it, so that a point is looked
//! for among the few boxes of its bucket, a box among those of the buckets it meets and a segment
//! among those of the buckets it passes, walked in order. The buckets only narrow the search:
//! every answer is decided on the items themselves, so it does not depend on how the buckets lie.

use crate::{Lists, Point, bounds_of};

/// Triangles per bucket, about, over triangles: a bucket as wide as a couple of them.
const PER_BUCKET: f64 = 2.0;
/// The most buckets per box, so that a few huge or far apart boxes cannot ask for many.
const MAX_BUCKETS_PER_BOX: usize = 4;

/// Buckets over a set of boxes, each named by its index.
#[derive(Debug, Clone, Default)]
pub struct Buckets {
    origin: Point,
    size: f64,
    columns: usize,
    rows: usize,
    /// Bucket `(i, j)` is list `j · columns + i`.
    lists: Lists<u32>,
}

impl Buckets {
    /// Buckets over the triangles with corners `corners(t)`, `count` of them, about `PER_BUCKET`
    /// triangles' boxes to a bucket.
    pub fn over(count: usize, corners: impl Fn(usize) -> Option<[Point; 3]>) -> Buckets {
        let boxes: Vec<Option<[Point; 2]>> = (0..count).map(|t| bounds_of(&corners(t)?)).collect();
        let area: f64 = boxes.iter().flatten().map(|[lo, hi]| (hi[0] - lo[0]) * (hi[1] - lo[1])).sum();
        Buckets::of_boxes(&boxes, (PER_BUCKET * area / count.max(1) as f64).sqrt())
    }

    /// Buckets about `size` wide over `boxes`, those that are there: wider where that would make
    /// more than `MAX_BUCKETS_PER_BOX` buckets a box.
    pub fn of_boxes(boxes: &[Option<[Point; 2]>], size: f64) -> Buckets {
        let Some([lo, hi]) = bounds_of(boxes.iter().flatten().flatten()) else { return Buckets::default() };
        if !(lo[0] <= hi[0] && lo[1] <= hi[1]) {
            return Buckets::default();
        }
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
        let budget = (MAX_BUCKETS_PER_BOX * boxes.len()).max(1) as f64;
        let floor = ((hi[0] - lo[0]) * (hi[1] - lo[1]) / budget).sqrt().max(span / budget);
        let size = size.max(floor).max(f64::MIN_POSITIVE);
        let side = |extent: f64| ((extent / size).floor() as usize).saturating_add(1);
        let (columns, rows) = (side(hi[0] - lo[0]), side(hi[1] - lo[1]));
        let mut buckets = Buckets { origin: lo, size, columns, rows, lists: Lists::default() };
        let ranges: Vec<Option<[usize; 4]>> =
            boxes.iter().map(|b| b.map(|[lo, hi]| buckets.range(lo, hi))).collect();
        let pairs = ranges.iter().enumerate().flat_map(|(t, range)| {
            range.iter().copied().flat_map(move |[i0, i1, j0, j1]| {
                (j0..=j1).flat_map(move |j| (i0..=i1).map(move |i| (j * columns + i, t as u32)))
            })
        });
        buckets.lists = Lists::from_pairs(columns.saturating_mul(rows), pairs);
        buckets
    }

    /// The bucket column and row holding `p`, clamped into the grid.
    fn cell(&self, p: Point) -> [usize; 2] {
        let at = |v: f64, o: f64, n: usize| {
            let k = ((v - o) / self.size).floor();
            if k.is_nan() || k < 0.0 { 0 } else { (k as usize).min(n.saturating_sub(1)) }
        };
        [at(p[0], self.origin[0], self.columns), at(p[1], self.origin[1], self.rows)]
    }

    /// The columns and rows of buckets the box from `lo` to `hi` meets.
    fn range(&self, lo: Point, hi: Point) -> [usize; 4] {
        let ([i0, j0], [i1, j1]) = (self.cell(lo), self.cell(hi));
        [i0, i1, j0, j1]
    }

    /// The boxes listed in bucket `(i, j)`.
    fn bucket(&self, i: usize, j: usize) -> &[u32] {
        self.lists.get(j * self.columns + i)
    }

    /// The boxes that may hold `p`.
    pub fn near(&self, p: Point) -> &[u32] {
        if !self.meets(p, p) {
            return &[];
        }
        let [i, j] = self.cell(p);
        self.bucket(i, j)
    }

    /// Whether the box from `lo` to `hi` meets the buckets' box.
    fn meets(&self, lo: Point, hi: Point) -> bool {
        let far =
            [self.origin[0] + self.size * self.columns as f64, self.origin[1] + self.size * self.rows as f64];
        !self.lists.items().is_empty()
            && hi[0] >= self.origin[0]
            && hi[1] >= self.origin[1]
            && lo[0] <= far[0]
            && lo[1] <= far[1]
    }

    /// Calls `f` with each box listed in the buckets the box from `lo` to `hi` meets, row after
    /// row: a box in several of them, once for each.
    pub fn visit(&self, lo: Point, hi: Point, mut f: impl FnMut(u32)) {
        if !self.meets(lo, hi) {
            return;
        }
        let [i0, i1, j0, j1] = self.range(lo, hi);
        for j in j0..=j1 {
            self.lists.span(j * self.columns + i0, j * self.columns + i1).iter().for_each(|&t| f(t));
        }
    }

    /// The boxes that may meet the box from `lo` to `hi`, each once, in order.
    pub fn within(&self, lo: Point, hi: Point) -> Vec<u32> {
        let mut out: Vec<u32> = Vec::new();
        self.visit(lo, hi, |t| out.push(t));
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The boxes that may meet the segment from `a` to `b`, each once, in order: in each column
    /// of buckets the segment crosses, the rows it spans there and one more each side, which
    /// rounding cannot cross.
    pub fn along(&self, a: Point, b: Point) -> Vec<u32> {
        let (lo, hi) = ([a[0].min(b[0]), a[1].min(b[1])], [a[0].max(b[0]), a[1].max(b[1])]);
        let [i0, i1, j0, j1] = self.range(lo, hi);
        if i0 == i1 || j0 == j1 || !self.meets(lo, hi) {
            return self.within(lo, hi);
        }
        let d = [b[0] - a[0], b[1] - a[1]];
        let mut out: Vec<u32> = Vec::new();
        for i in i0..=i1 {
            let x0 = (self.origin[0] + self.size * i as f64).max(lo[0]);
            let x1 = (self.origin[0] + self.size * (i + 1) as f64).min(hi[0]);
            let y = |x: f64| a[1] + (x - a[0]) * d[1] / d[0];
            let (ya, yb) = (y(x0), y(x1));
            let [_, ja] = self.cell([x0, ya.min(yb)]);
            let [_, jb] = self.cell([x0, ya.max(yb)]);
            for j in ja.saturating_sub(1).max(j0)..=(jb + 1).min(j1) {
                out.extend_from_slice(self.bucket(i, j));
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}
