//! The solids of a print, each with its own frame. A solid is what holds together: its islands
//! on one layer touch one another's, and those of the layer below or above. Its frame is that of
//! all its layers at once (`Frame`), each island weighing its layer's height: the lines it
//! settles, where its seams go and the way its loops turn, turn, move and mirror with the solid,
//! stay the same from layer to layer, and change with nothing else on the plate.

use crate::frame::point_inside;
use crate::island::holder;
use layercraft_geom::{
    Contour, DisjointSets, Frame, GeomError, Point, RESOLUTION_MM, Region, Shape, bounds, intersection,
    offset,
};
use layercraft_region::RegionPass;

/// The solids of a print: each pass's islands, by outer contour, with their solid, and each
/// solid's frame.
pub(crate) struct Solids {
    passes: Vec<Vec<(Contour, usize)>>,
    frames: Vec<Frame>,
}

/// One island while the solids are found: its shape, that grown by `RESOLUTION_MM`, which meets
/// the islands it touches, its box and its number among the sets.
struct Piece<'a> {
    shape: &'a Shape,
    grown: Region,
    box_: [Point; 2],
    id: usize,
}

/// Whether two boxes come within `gap` of each other.
fn boxes_meet([lo, hi]: [Point; 2], [a, b]: [Point; 2], gap: f64) -> bool {
    a[0] <= hi[0] + gap && lo[0] <= b[0] + gap && a[1] <= hi[1] + gap && lo[1] <= b[1] + gap
}

/// Joins in `sets` each island of `a` with those of `b` it touches.
fn join(sets: &mut DisjointSets, a: &[Piece], b: &[Piece]) -> Result<(), GeomError> {
    for x in a {
        for y in b {
            if boxes_meet(x.box_, y.box_, 2.0 * RESOLUTION_MM)
                && !intersection(&x.grown, &vec![y.shape.clone()])?.is_empty()
            {
                sets.unite(x.id, y.id);
            }
        }
    }
    Ok(())
}

impl Solids {
    /// The solids the islands `shapes` gives each pass make, the passes of one height side by
    /// side, each height on the one before.
    pub fn of<'a>(
        layers: &'a [RegionPass],
        shapes: impl Fn(&'a RegionPass) -> Vec<&'a Shape>,
    ) -> Result<Solids, GeomError> {
        let mut count = 0;
        let mut islands: Vec<Vec<Piece>> = Vec::with_capacity(layers.len());
        for layer in layers {
            let mut own = Vec::new();
            for shape in shapes(layer) {
                let Some(box_) = shape.first().and_then(|outer| bounds(&vec![vec![outer.clone()]])) else {
                    continue;
                };
                let grown = offset(&vec![shape.clone()], RESOLUTION_MM)?;
                own.push(Piece { shape, grown, box_, id: count });
                count += 1;
            }
            islands.push(own);
        }
        let mut sets = DisjointSets::new(count);
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
        // Each pass against the others of its height and those of the height above.
        for (k, height) in heights.iter().enumerate() {
            let above = heights.get(k + 1).map_or(&[][..], |h| h.as_slice());
            for (n, &a) in height.iter().enumerate() {
                for &b in height.iter().skip(n + 1).chain(above) {
                    join(&mut sets, pass(a), pass(b))?;
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
                let r = sets.root(island.id);
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
        self.solid_at(pass, point_inside(shape)?)
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
