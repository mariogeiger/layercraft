//! A pass's beads in printing order: island by island, nearest first; within an island its walls
//! from the innermost out, then the beads along its strips, then each infill area's lines, in
//! sweeps across the area where its face asks for one look (`Order::Sweep`) and in the cheapest
//! tour otherwise (`tour`). Each move is marked as it is laid when it crosses a bead laid before
//! it at its height (`crossing`), and the tours keep their moves off those beads where they can.

use crate::Path;
use crate::crossing::{Laid, Segment};
use crate::families::apart;
use crate::island::{Island, Order};
use crate::tour::tour;
use layercraft_geom::{Point, distance2, height_range, polyline_length};

/// Lines closer in direction than this, in degrees, are swept together: parallel lines differ by
/// the lattice's rounding of their ends.
const SAME_SWEEP: f64 = 1e-3;

/// Lays parallel lines in sweeps across their area, each line against the one before and all
/// sweeps the same way: from the side nearer the nozzle, each sweep takes the next line across
/// that overlaps the last one along their direction and lies within `reach`, and a new sweep
/// starts at the first line left. Each line starts where the last one ended.
fn sweep(mut lines: Vec<Path>, reach: f64, at: &mut Point, out: &mut Vec<Path>, laid: &Laid) {
    let Some((a, b)) = lines.first().and_then(|l| Some((*l.points.first()?, *l.points.last()?))) else {
        return tour(lines, at, out, laid);
    };
    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
    if length <= 0.0 {
        return tour(lines, at, out, laid);
    }
    let along = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
    let mut across = [-along[1], along[0]];
    let project = |d: [f64; 2], p: &Point| p[0] * d[0] + p[1] * d[1];
    let offset_of = |d: [f64; 2], l: &Path| {
        l.points.iter().map(|p| project(d, p)).sum::<f64>() / l.points.len().max(1) as f64
    };
    let nearest = |l: &Path| l.points.iter().map(|p| distance2(*at, *p)).fold(f64::MAX, f64::min);
    lines.sort_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    if let (Some(first), Some(last)) = (lines.first(), lines.last())
        && nearest(last) < nearest(first)
    {
        across = [-across[0], -across[1]];
        lines.sort_by(|x, y| offset_of(across, x).total_cmp(&offset_of(across, y)));
    }
    let span = |l: &Path| {
        l.points
            .iter()
            .map(|p| project(along, p))
            .fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)))
    };
    let mut left: Vec<Option<Path>> = lines.into_iter().map(Some).collect();
    while let Some(start) = left.iter().position(Option::is_some) {
        let mut next = Some(start);
        while let Some((current, mut line)) = next.and_then(|i| Some((i, left.get_mut(i)?.take()?))) {
            if let (Some(&s), Some(&e)) = (line.points.first(), line.points.last())
                && distance2(*at, e) < distance2(*at, s)
            {
                line.reverse();
            }
            let (offset, (lo, hi)) = (offset_of(across, &line), span(&line));
            if let Some(end) = line.end() {
                *at = end;
            }
            out.push(line);
            next = left.iter().enumerate().skip(current + 1).find_map(|(i, l)| {
                let l = l.as_ref()?;
                let (o, (a, b)) = (offset_of(across, l), span(l));
                (o - offset > 1e-9 && o - offset <= reach && a <= hi && lo <= b).then_some(i)
            });
        }
    }
}

/// Lines grouped by their direction, the direction laying the longest first, the one with an end
/// nearer the nozzle `at` where equally long: each is swept on its own.
fn by_direction(lines: Vec<Path>, at: Point) -> Vec<Vec<Path>> {
    let direction = |l: &Path| match (l.points.first(), l.points.last()) {
        (Some(a), Some(b)) => (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees().rem_euclid(180.0),
        _ => 0.0,
    };
    let mut groups: Vec<(f64, f64, Vec<Path>)> = Vec::new();
    for line in lines {
        let (angle, length) = (direction(&line), polyline_length(&line.points));
        let near = |g: &&mut (f64, f64, Vec<Path>)| apart(g.0, angle) < SAME_SWEEP;
        match groups.iter_mut().find(near) {
            Some(group) => {
                group.1 += length;
                group.2.push(line);
            }
            None => groups.push((angle, length, vec![line])),
        }
    }
    let nearest_end = |lines: &[Path]| {
        lines
            .iter()
            .flat_map(|l| [l.points.first(), l.points.last()])
            .flatten()
            .map(|p| distance2(at, *p))
            .fold(f64::MAX, f64::min)
    };
    groups.sort_by(|x, y| y.1.total_cmp(&x.1).then(nearest_end(&x.2).total_cmp(&nearest_end(&y.2))));
    groups.into_iter().map(|(_, _, lines)| lines).collect()
}

/// The pass's beads in printing order, each marked when the move to it crosses a bead laid before
/// it: one of these, or one of `earlier`, laid at this height by the passes before. Each tour
/// keeps its moves off the beads laid before it where it can.
pub fn sequence(mut islands: Vec<Island>, at: &mut Point, earlier: &[Segment]) -> Vec<Path> {
    let beads = || islands.iter().flat_map(Island::beads);
    let ends = earlier.iter().flat_map(|(a, b)| [a[1], b[1]]);
    let (low, high) = height_range(ends.chain(beads().flat_map(|p| p.points.iter().map(|q| q[1]))));
    let count = earlier.len() + beads().map(|p| p.points.len()).sum::<usize>();
    let mut laid = Laid::new(low, high, count);
    for &(a, b) in earlier {
        laid.add(a, b);
    }
    let mut out = Vec::new();
    while !islands.is_empty() {
        let near = islands
            .iter()
            .enumerate()
            .map(|(i, island)| (i, distance2(*at, island.first_start())))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
            .unwrap_or(0);
        let Island { mut walls, gaps, fills, .. } = islands.swap_remove(near);
        walls.sort_by_key(|(k, _)| std::cmp::Reverse(*k));
        let mut depth = Vec::new();
        let mut last = None;
        for (k, path) in walls {
            if last != Some(k) && !depth.is_empty() {
                lay(std::mem::take(&mut depth), Order::Tour, at, &mut laid, &mut out);
            }
            last = Some(k);
            depth.push(path);
        }
        lay(depth, Order::Tour, at, &mut laid, &mut out);
        lay(gaps, Order::Tour, at, &mut laid, &mut out);
        for fill in fills {
            lay(fill.lines, fill.order, at, &mut laid, &mut out);
        }
    }
    out
}

/// Puts `paths` in `order` from the nozzle `at`, their moves kept off the beads `laid` where they
/// can be, then lays them after `out`.
fn lay(paths: Vec<Path>, order: Order, at: &mut Point, laid: &mut Laid, out: &mut Vec<Path>) {
    let mut ordered = Vec::new();
    match order {
        Order::Sweep { reach } => {
            for lines in by_direction(paths, *at) {
                sweep(lines, reach, at, &mut ordered, laid);
            }
        }
        Order::Tour => tour(paths, at, &mut ordered, laid),
    }
    for mut path in ordered {
        laid.lay(&mut path);
        out.push(path);
    }
}
