//! Finding the triangles of a set near a point or along a segment, through buckets.
//!
//! Square buckets cover the triangles' box, each listing the triangles whose box meets it, so
//! that a point is looked for among the few triangles of its bucket and a segment among those
//! of the buckets it passes, walked in order. The buckets only narrow the search: every answer
//! is decided on the triangles themselves, so it does not depend on how the buckets lie.

use crate::Point;

/// Triangles per bucket, about: a bucket as wide as a couple of triangles.
const PER_BUCKET: f64 = 2.0;
/// The most buckets per triangle, so that a few huge triangles cannot ask for many.
const MAX_BUCKETS_PER_TRIANGLE: usize = 4;

/// Buckets over a set of triangles.
#[derive(Debug, Clone, Default)]
pub struct Buckets {
    origin: Point,
    size: f64,
    columns: usize,
    rows: usize,
    /// Bucket `k` lists `items[starts[k]..starts[k + 1]]`.
    starts: Vec<usize>,
    items: Vec<u32>,
}

impl Buckets {
    /// Buckets over the triangles with corners `corners(t)`, `count` of them.
    pub fn over(count: usize, corners: impl Fn(usize) -> Option<[Point; 3]>) -> Buckets {
        let boxes: Vec<Option<[Point; 2]>> = (0..count)
            .map(|t| {
                let c = corners(t)?;
                let lo = [c[0][0].min(c[1][0]).min(c[2][0]), c[0][1].min(c[1][1]).min(c[2][1])];
                let hi = [c[0][0].max(c[1][0]).max(c[2][0]), c[0][1].max(c[1][1]).max(c[2][1])];
                Some([lo, hi])
            })
            .collect();
        let mut bounds = [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]];
        let mut area = 0.0;
        for [lo, hi] in boxes.iter().flatten() {
            bounds = [
                [bounds[0][0].min(lo[0]), bounds[0][1].min(lo[1])],
                [bounds[1][0].max(hi[0]), bounds[1][1].max(hi[1])],
            ];
            area += (hi[0] - lo[0]) * (hi[1] - lo[1]);
        }
        let [lo, hi] = bounds;
        if !(lo[0] <= hi[0] && lo[1] <= hi[1]) {
            return Buckets::default();
        }
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
        let budget = (MAX_BUCKETS_PER_TRIANGLE * count).max(1) as f64;
        // About `PER_BUCKET` triangles' boxes per bucket, and no more buckets than the budget.
        let wanted = (PER_BUCKET * area / count.max(1) as f64).sqrt();
        let floor = ((hi[0] - lo[0]) * (hi[1] - lo[1]) / budget).sqrt().max(span / budget);
        let size = wanted.max(floor).max(f64::MIN_POSITIVE);
        let side = |extent: f64| ((extent / size).floor() as usize).saturating_add(1);
        let (columns, rows) = (side(hi[0] - lo[0]), side(hi[1] - lo[1]));
        let mut buckets = Buckets { origin: lo, size, columns, rows, starts: Vec::new(), items: Vec::new() };
        let ranges: Vec<Option<[usize; 4]>> =
            boxes.iter().map(|b| b.map(|[lo, hi]| buckets.range(lo, hi))).collect();
        let mut counts = vec![0usize; columns * rows + 1];
        for [i0, i1, j0, j1] in ranges.iter().flatten() {
            for j in *j0..=*j1 {
                for i in *i0..=*i1 {
                    if let Some(c) = counts.get_mut(j * columns + i) {
                        *c += 1;
                    }
                }
            }
        }
        let mut starts = Vec::with_capacity(counts.len() + 1);
        let mut total = 0;
        for c in &counts {
            starts.push(total);
            total += c;
        }
        starts.push(total);
        let mut fill = starts.clone();
        let mut items = vec![0u32; total];
        for (t, range) in ranges.iter().enumerate() {
            let Some([i0, i1, j0, j1]) = range else { continue };
            for j in *j0..=*j1 {
                for i in *i0..=*i1 {
                    if let Some(slot) = fill.get_mut(j * columns + i)
                        && let Some(item) = items.get_mut(*slot)
                    {
                        *item = t as u32;
                        *slot += 1;
                    }
                }
            }
        }
        buckets.starts = starts;
        buckets.items = items;
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

    /// The triangles listed in bucket `(i, j)`.
    fn bucket(&self, i: usize, j: usize) -> &[u32] {
        let k = j * self.columns + i;
        match (self.starts.get(k), self.starts.get(k + 1)) {
            (Some(&a), Some(&b)) => self.items.get(a..b).unwrap_or(&[]),
            _ => &[],
        }
    }

    /// The triangles whose box may hold `p`.
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
        !self.items.is_empty()
            && hi[0] >= self.origin[0]
            && hi[1] >= self.origin[1]
            && lo[0] <= far[0]
            && lo[1] <= far[1]
    }

    /// The triangles whose box may meet the box from `lo` to `hi`, each once, in order.
    pub fn within(&self, lo: Point, hi: Point) -> Vec<u32> {
        if !self.meets(lo, hi) {
            return Vec::new();
        }
        let [i0, i1, j0, j1] = self.range(lo, hi);
        let mut out: Vec<u32> = Vec::new();
        for j in j0..=j1 {
            for i in i0..=i1 {
                out.extend_from_slice(self.bucket(i, j));
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The triangles whose box may meet the segment from `a` to `b`, each once, in order: in
    /// each column of buckets the segment crosses, the rows it spans there and one more each
    /// side, which rounding cannot cross.
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
