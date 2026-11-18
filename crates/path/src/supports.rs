//! The supports' lines, inside the supports' areas, which keep their gap to the part. A column
//! is a loop along its edge, where a bead's centre can reach (`disc_offset`), stacked layer on layer into a thin shell: all of
//! its body. Its interface is laid in straight lines across it the short way, `interface_gap`
//! apart, a quarter turn from the column's axis (its greatest spread) through its origin, each
//! running from the loop to the loop, its ends on the loop's centre line, so that no line of the
//! supports stands inside a column but its top: the first layer of the interface spans the column
//! over air from one side of the shell to the other (`Support::Air`), slowly, as a strand on
//! air hangs; the next rest on it. Each piece of the supports is an island; its loop is laid
//! first, then its lines, neighbouring lines joined inside it without retracting. On the first
//! layer the lines take its width, to grip.

use crate::frame::hatch_in_frames;
use crate::order::Fill;
use crate::order::{Island, holder, on_line};
use crate::solids::Solids;
use crate::{Path, PathError, PathParams, Role, Support};
use layercraft_geom::{Region, bead_spacing, difference, disc_offset, intersection, polyline_length, union};
use layercraft_region::RegionLayer;
use std::rc::Rc;

/// The islands of the layer's supports, their loops and the lines inside them, over the
/// interface of the layer below (`interface_below`).
pub fn islands(
    layer: &RegionLayer,
    p: &PathParams,
    columns: &Solids,
    pass: usize,
    interface_below: &Region,
) -> Result<Vec<Island>, PathError> {
    if layer.supports.is_empty() && layer.support_interface.is_empty() {
        return Ok(Vec::new());
    }
    let s = &p.support;
    let width = if layer.index == 0 { p.first_layer_width } else { s.width };
    let pitch = bead_spacing(width, layer.height) + s.interface_gap;
    // The loops' centre lines: what every line inside runs to.
    let rim = disc_offset(&union(&layer.supports, &layer.support_interface)?, -width / 2.0)?;
    let mut islands = Vec::new();
    for shape in &rim {
        let Some(outer) = shape.first() else { continue };
        let mut island = Island::bare(outer.clone());
        for ring in shape {
            island.push_wall(0, Path::new(Role::SUPPORT, width, true, ring.clone()));
        }
        islands.push(island);
    }
    let spanning = difference(&layer.support_interface, interface_below)?;
    let resting = intersection(&layer.support_interface, interface_below)?;
    let on_interface = Role { support: Support::Solid, ..Role::SUPPORT_INTERFACE };
    let mut fills = Vec::new();
    for (role, region) in [(Role::SUPPORT_INTERFACE, &spanning), (on_interface, &resting)] {
        if region.is_empty() {
            continue;
        }
        let inside = intersection(region, &rim)?;
        for line in hatch_in_frames(&inside, pitch, 90.0, |q| columns.frame_at(pass, q))? {
            // A line shorter than its bead is wide would lay a blob.
            if polyline_length(&line) < width {
                continue;
            }
            let home = on_line(&line).and_then(|q| holder(islands.iter().map(Island::outline), q));
            if let Some(island) = home.and_then(|i| islands.get_mut(i)) {
                island.push(Path::new(role, width, false, line));
            }
        }
        fills.push(Fill { role, max_link: 3.0 * pitch, monotone: false });
    }
    let fills = Rc::new(fills);
    for island in &mut islands {
        island.share(fills.clone());
    }
    Ok(islands)
}
