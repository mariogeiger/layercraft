//! Where a nozzle waits clear of a layer: the point nearest it that lies a given clearance from
//! every bead, inside the printable area. The beads are taken by their convex hull, the part of
//! the plate a hot end must stay off; nothing here names a plate axis, so a layer turned or
//! moved on the plate has its waiting point turned or moved with it. The printable area is the
//! machine's own box and may cut the choice.

use layercraft_geom::Point;

/// How far a distance may fall short of the clearance and still count as reaching it, in mm: the
/// rounding of the candidates, which lie on the clearance exactly in theory.
const ROUNDING_MM: f64 = 1e-9;

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

fn add_scaled(a: Point, d: Point, t: f64) -> Point {
    [a[0] + t * d[0], a[1] + t * d[1]]
}

fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn length(a: Point) -> f64 {
    a[0].hypot(a[1])
}

/// The convex hull of `points`, anticlockwise, without repeated points: one point or two where
/// the points are one or collinear.
fn hull(points: &[Point]) -> Vec<Point> {
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    sorted.dedup();
    if sorted.len() < 3 {
        return sorted;
    }
    let chain = |sorted: &mut dyn Iterator<Item = &Point>| {
        let mut half: Vec<Point> = Vec::new();
        for &p in sorted {
            while let [.., a, b] = half.as_slice()
                && cross(sub(*b, *a), sub(p, *b)) <= 0.0
            {
                half.pop();
            }
            half.push(p);
        }
        half.pop();
        half
    };
    let mut around = chain(&mut sorted.iter());
    around.extend(chain(&mut sorted.iter().rev()));
    around
}

/// The edges of a hull as pairs of ends; for one point, none; for two, the segment.
fn edges(hull: &[Point]) -> Vec<(Point, Point)> {
    match hull {
        [] | [_] => Vec::new(),
        [a, b] => vec![(*a, *b)],
        _ => hull.iter().copied().zip(hull.iter().copied().cycle().skip(1)).collect(),
    }
}

/// How far `p` lies from the segment `a`–`b`.
fn from_segment(p: Point, (a, b): (Point, Point)) -> f64 {
    let d = sub(b, a);
    let t = if dot(d, d) > 0.0 { (dot(sub(p, a), d) / dot(d, d)).clamp(0.0, 1.0) } else { 0.0 };
    length(sub(p, add_scaled(a, d, t)))
}

/// How far `p` lies from the hull and what it holds: 0 inside.
fn from_hull(hull: &[Point], p: Point) -> f64 {
    let sides = edges(hull);
    let inside = hull.len() >= 3 && sides.iter().all(|&(a, b)| cross(sub(b, a), sub(p, a)) >= 0.0);
    if inside {
        return 0.0;
    }
    let near = |q: &Point| length(sub(p, *q));
    sides.iter().map(|&s| from_segment(p, s)).chain(hull.iter().map(near)).fold(f64::INFINITY, f64::min)
}

/// Where the segment `a`–`b` meets the segment `c`–`d`, as its parameter along `a`–`b`.
fn crossing(a: Point, b: Point, c: Point, d: Point) -> Option<f64> {
    let (u, v) = (sub(b, a), sub(d, c));
    let det = cross(u, v);
    if det == 0.0 {
        return None;
    }
    let (t, s) = (cross(sub(c, a), v) / det, cross(sub(c, a), u) / det);
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&s)).then_some(t)
}

/// The points where the segment `a`–`b` meets the circle of `radius` about `centre`.
fn circle_crossings(a: Point, b: Point, centre: Point, radius: f64) -> Vec<Point> {
    let (d, f) = (sub(b, a), sub(a, centre));
    let (qa, qb, qc) = (dot(d, d), 2.0 * dot(f, d), dot(f, f) - radius * radius);
    let discriminant = qb * qb - 4.0 * qa * qc;
    if qa <= 0.0 || discriminant < 0.0 {
        return Vec::new();
    }
    let root = discriminant.sqrt();
    [(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)]
        .into_iter()
        .filter(|t| (0.0..=1.0).contains(t))
        .map(|t| add_scaled(a, d, t))
        .collect()
}

/// The point of the printable `area`, `[min, max]`, nearest `from` that lies at least `clearance`
/// from the hull of `beads`; `from` itself where it does. Of equally near points, the one
/// farthest ahead along `heading` (a unit vector, or zero), the way the nozzle was going. With no
/// beads, `from`; with no such point, none.
pub fn clear_of(
    beads: &[Point],
    from: Point,
    heading: Point,
    clearance: f64,
    area: [[f64; 2]; 2],
) -> Option<Point> {
    let hull = hull(beads);
    if hull.is_empty() {
        return Some(from);
    }
    let [lo, hi] = area;
    let in_area = |p: Point| {
        (lo[0] - ROUNDING_MM..=hi[0] + ROUNDING_MM).contains(&p[0])
            && (lo[1] - ROUNDING_MM..=hi[1] + ROUNDING_MM).contains(&p[1])
    };
    let clear = |p: Point| in_area(p) && from_hull(&hull, p) >= clearance - ROUNDING_MM;
    if clear(from) {
        return Some(from);
    }
    // The boundary of the hull grown by the clearance: the hull's edges moved out, and arcs of
    // the clearance about its corners.
    let sides = edges(&hull);
    let moved: Vec<(Point, Point)> = sides
        .iter()
        .flat_map(|&(a, b)| {
            let d = sub(b, a);
            let n = [d[1] / length(d), -d[0] / length(d)];
            let both = if hull.len() == 2 { vec![n, [-n[0], -n[1]]] } else { vec![n] };
            both.into_iter().map(move |n| (add_scaled(a, n, clearance), add_scaled(b, n, clearance)))
        })
        .collect();
    let corners = [lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]];
    let walls: Vec<(Point, Point)> =
        corners.iter().copied().zip(corners.iter().copied().cycle().skip(1)).collect();
    let mut candidates: Vec<Point> = corners.to_vec();
    // Nearest points of each piece of the boundary, and of the area's walls.
    for &(a, b) in moved.iter().chain(&walls) {
        let d = sub(b, a);
        let t = if dot(d, d) > 0.0 { (dot(sub(from, a), d) / dot(d, d)).clamp(0.0, 1.0) } else { 0.0 };
        candidates.push(add_scaled(a, d, t));
    }
    for &v in &hull {
        let away = sub(from, v);
        let u = if length(away) > ROUNDING_MM {
            [away[0] / length(away), away[1] / length(away)]
        } else {
            heading
        };
        candidates.push(add_scaled(v, u, clearance));
    }
    // Where the area's walls meet the boundary.
    for &(a, b) in &walls {
        candidates.extend(
            moved.iter().filter_map(|&(c, d)| crossing(a, b, c, d).map(|t| add_scaled(a, sub(b, a), t))),
        );
        candidates.extend(hull.iter().flat_map(|&v| circle_crossings(a, b, v, clearance)));
    }
    let near = |p: &Point| length(sub(from, *p));
    candidates.into_iter().filter(|&p| clear(p)).min_by(|a, b| {
        let (da, db) = (near(a), near(b));
        if (da - db).abs() > ROUNDING_MM {
            da.total_cmp(&db)
        } else {
            dot(heading, sub(*b, from)).total_cmp(&dot(heading, sub(*a, from)))
        }
    })
}
