//! The solids of a print, each with its own frame. A solid is what holds together: its islands
//! on one layer touch one another's, and those of the layer below or above. Its frame is that of
//! all its layers at once (`Frame`), each island weighing its layer's height: the lines it
//! settles, where its seams go and the way its loops turn, turn, move and mirror with the solid,
//! stay the same from layer to layer, and change with nothing else on the plate.

use crate::frame::{Frame, inside};
use crate::order::holder;
use layercraft_geom::{
    Contour, GeomError, Point, RESOLUTION_MM, Region, Shape, bounds, intersection, offset,
};
use layercraft_region::RegionLayer;

/// The solids of a print: each pass's islands, by outer contour, with their solid, and each
/// solid's frame.
pub(crate) struct Solids {
    passes: Vec<Vec<(Contour, usize)>>,
    frames: Vec<Frame>,
}

/// One island while the solids are found: its shape, its box and its place in the union-find.
struct Island<'a> {
    shape: &'a Shape,
    box_: [Point; 2],
    id: usize,
}

/// Whether two boxes come within `gap` of each other.
fn near([lo, hi]: [Point; 2], [a, b]: [Point; 2], gap: f64) -> bool {
    a[0] <= hi[0] + gap && lo[0] <= b[0] + gap && a[1] <= hi[1] + gap && lo[1] <= b[1] + gap
}

/// The root of `i` in the union-find `up`, halving the path there.
fn root(up: &mut [usize], mut i: usize) -> usize {
    for _ in 0..up.len() {
        let next = up.get(i).copied().unwrap_or(i);
        if next == i {
            break;
        }
        let skip = up.get(next).copied().unwrap_or(next);
        if let Some(slot) = up.get_mut(i) {
            *slot = skip;
        }
        i = skip;
    }
    i
}

impl Solids {
    /// The solids the islands `shapes` gives each pass make, the passes of one height side by
    /// side, each height on the one before.
    pub fn of<'a>(
        layers: &'a [RegionLayer],
        shapes: impl Fn(&'a RegionLayer) -> Vec<&'a Shape>,
    ) -> Result<Solids, GeomError> {
        let mut count = 0;
        let islands: Vec<Vec<Island>> = layers
            .iter()
            .map(|layer| {
                let own = shapes(layer).into_iter().filter_map(|shape| {
                    let box_ = bounds(&vec![vec![shape.first()?.clone()]])?;
                    count += 1;
                    Some(Island { shape, box_, id: count - 1 })
                });
                own.collect()
            })
            .collect();
        let mut up: Vec<usize> = (0..count).collect();
        let touch = |a: &Island, b: &Island| -> Result<bool, GeomError> {
            if !near(a.box_, b.box_, 2.0 * RESOLUTION_MM) {
                return Ok(false);
            }
            let grown = offset(&vec![a.shape.clone()], RESOLUTION_MM)?;
            Ok(!intersection(&grown, &vec![b.shape.clone()])?.is_empty())
        };
        // The passes of each height, in order.
        let mut heights: Vec<Vec<usize>> = Vec::new();
        for (i, layer) in layers.iter().enumerate() {
            let same = heights
                .last()
                .and_then(|h| h.first())
                .and_then(|&j| layers.get(j))
                .is_some_and(|l| l.top == layer.top);
            match heights.last_mut() {
                Some(h) if same => h.push(i),
                _ => heights.push(vec![i]),
            }
        }
        let pass = |i: usize| islands.get(i).map_or(&[][..], |v| v.as_slice());
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for (k, height) in heights.iter().enumerate() {
            let above = heights.get(k + 1).map_or(&[][..], |h| h.as_slice());
            for (n, &a) in height.iter().enumerate() {
                let others = height.iter().skip(n + 1).chain(above);
                for &b in others {
                    pairs.push((a, b));
                }
            }
        }
        for (a, b) in pairs {
            for x in pass(a) {
                for y in pass(b) {
                    if touch(x, y)? {
                        let (rx, ry) = (root(&mut up, x.id), root(&mut up, y.id));
                        if let Some(slot) = up.get_mut(rx.max(ry)) {
                            *slot = rx.min(ry);
                        }
                    }
                }
            }
        }
        // Each solid's areas, by root, and its frame.
        let mut solid_of: Vec<Option<usize>> = vec![None; count];
        let mut areas: Vec<Vec<(Region, f64)>> = Vec::new();
        let mut passes = Vec::with_capacity(layers.len());
        for (layer, own) in layers.iter().zip(&islands) {
            let mut here = Vec::with_capacity(own.len());
            for island in own {
                let r = root(&mut up, island.id);
                let solid = match solid_of.get(r).copied().flatten() {
                    Some(s) => s,
                    None => {
                        areas.push(Vec::new());
                        if let Some(slot) = solid_of.get_mut(r) {
                            *slot = Some(areas.len() - 1);
                        }
                        areas.len() - 1
                    }
                };
                if let Some(list) = areas.get_mut(solid) {
                    list.push((vec![island.shape.clone()], layer.height));
                }
                if let Some(outer) = island.shape.first() {
                    here.push((outer.clone(), solid));
                }
            }
            passes.push(here);
        }
        let frames = areas.iter().map(|list| Frame::of(list.iter().map(|(r, w)| (r, *w)))).collect();
        Ok(Solids { passes, frames })
    }

    /// The solid whose island on pass `pass` holds `p`, the innermost.
    pub fn solid_at(&self, pass: usize, p: Point) -> Option<usize> {
        let islands = self.passes.get(pass)?;
        let i = holder(islands.iter().map(|(c, _)| c), p)?;
        islands.get(i).map(|(_, s)| *s)
    }

    /// The solid whose island on pass `pass` has the outer contour `outline`.
    pub fn solid_of(&self, pass: usize, outline: &Contour) -> Option<usize> {
        self.passes.get(pass)?.iter().find(|(c, _)| c == outline).map(|(_, s)| *s)
    }

    /// How many solids there are.
    pub fn count(&self) -> usize {
        self.frames.len()
    }

    /// The solid whose island on pass `pass` holds `shape`, found by a point inside it.
    pub fn solid_of_shape(&self, pass: usize, shape: &Shape) -> Option<usize> {
        self.solid_at(pass, inside(shape)?)
    }

    /// The frame of solid `solid`; the plate's where there is none.
    pub fn frame(&self, solid: usize) -> Frame {
        self.frames.get(solid).copied().unwrap_or_default()
    }

    /// The frame of the solid holding `p` on pass `pass`; the plate's where none does.
    pub fn frame_at(&self, pass: usize, p: Point) -> Frame {
        self.solid_at(pass, p).and_then(|s| self.frames.get(s)).copied().unwrap_or_default()
    }
}
