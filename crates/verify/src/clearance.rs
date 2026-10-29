//! The toolhead against what is printed: no part of the head comes down into a bead laid before.
//!
//! Seen from below, the head is a solid of revolution about the nozzle's axis whose underside
//! lies `floor(r)` above the tip at distance `r` from it: nothing on the nozzle's flat face, then
//! its flank up to its column, and past the column the plane of the rest of the head, the other
//! nozzle and the gantry, taken `margin` lower, for beads that curl up and a plate not quite flat.
//! The plane reaches everywhere: no bead may stand higher than the tip by more than its
//! clearance. Within the column the beads are met exactly (`beads`): on a bent layer the face
//! irons the beads of its own layer it laid just before, up the slope, by up to `ironing`, and
//! no others. The tip passes the points of each move a step apart; about each, out to half a step
//! either way, it lies within half a step of it across the plate and no lower than the lower end
//! of that stretch, its height changing evenly along a move. The beads are tested against the
//! nozzle so widened and lowered; where that fails, against the tip at the point itself, a
//! failure there a true one, and the stretch is halved about it, down to `MIN_BLUR`.

use crate::Toolhead;
use crate::beads::{Allowed, Beads};
use layercraft_motion::Move;

/// The step between the points of a move tested, in mm.
const STEP: f64 = 0.1;
/// The most points of one move: a plate 400 mm across at a step each.
const MAX_STEPS: usize = 1 << 13;
/// How finely a stretch is halved before its bound stands for it, in mm across the plate and up.
const MIN_BLUR: f64 = 1e-6;
/// The most stretches one move is tested over, its halves counted, before the bounds stand.
const MAX_STRETCHES: usize = 1 << 16;
/// How closely the greatest intrusion of a move is sought, in mm.
const PRECISION: f64 = 1e-4;

impl Toolhead {
    /// How high the head's underside stands above its tip at distance `r` from its axis.
    pub fn floor(&self, r: f64) -> f64 {
        if r <= self.face_radius {
            0.0
        } else if r <= self.column_radius {
            ((r - self.face_radius) * self.flank_slope).min(self.protrusion)
        } else {
            self.clearance()
        }
    }

    /// The most a printed point may stand above the tip past the nozzle: under the head's plane,
    /// its margin kept.
    pub fn clearance(&self) -> f64 {
        (self.protrusion - self.margin).max(0.0)
    }
}

/// How far a move from `from` comes down into what `beads` holds beyond what is allowed, if at
/// all, and whether into the beads of its own layer: the head's plane first, then the nozzle.
pub fn intrusion(beads: &Beads, allowed: &Allowed, m: &Move, from: [f64; 3]) -> Option<(f64, bool)> {
    let to = m.to?;
    let plane = beads.highest - from[2].min(to[2]) - allowed.lift - allowed.head.clearance();
    if plane > allowed.written {
        return Some((plane, false));
    }
    let length = m.length(from);
    let steps = ((length / STEP).ceil() as usize).clamp(1, MAX_STEPS);
    let mut search = Search { beads, allowed, m, from, length, worst: None, budget: MAX_STRETCHES };
    for k in 0..steps {
        search.stretch([k as f64 / steps as f64, (k + 1) as f64 / steps as f64]);
    }
    search.worst
}

/// The greatest intrusion of a move, sought stretch by stretch: a stretch whose bound over the
/// whole exceeds neither nothing nor the worst found, to `PRECISION`, is passed; another is
/// tested at its middle, then halved, while `budget` lasts, its bound standing for it where it
/// runs out or the stretch is too fine to halve.
struct Search<'a> {
    beads: &'a Beads,
    allowed: &'a Allowed<'a>,
    m: &'a Move,
    from: [f64; 3],
    length: f64,
    worst: Option<(f64, bool)>,
    budget: usize,
}

impl Search<'_> {
    fn stretch(&mut self, [s0, s1]: [f64; 2]) {
        self.budget = self.budget.saturating_sub(1);
        let (m, from) = (self.m, self.from);
        let mid = (s0 + s1) / 2.0;
        let tip = m.point(from, mid);
        let low = m.point(from, s0)[2].min(m.point(from, s1)[2]);
        let (a, b) = (m.point(from, s0), m.point(from, s1));
        // Across the plate the tip stays within half the stretch's length of its middle, and
        // within half its span when it runs straight.
        let span = (b[0] - a[0]).hypot(b[1] - a[1]);
        let blur = if m.arc.is_some() { self.length * (s1 - s0) / 2.0 } else { span / 2.0 };
        let enough = self.worst.map_or(0.0, |w| w.0 + PRECISION);
        let bound = self.beads.contact(self.allowed, [tip[0], tip[1], low], blur);
        if bound.0 <= enough {
            return;
        }
        let point = self.beads.contact(self.allowed, tip, 0.0);
        if point.0 > self.worst.map_or(0.0, |w| w.0) {
            self.worst = Some(point);
        }
        if blur.max((a[2] - b[2]).abs()) <= MIN_BLUR || self.budget == 0 {
            if self.worst.is_none() {
                self.worst = Some(bound);
            }
            return;
        }
        self.stretch([s0, mid]);
        self.stretch([mid, s1]);
    }
}
