//! The supports' lines, inside the supports' areas, which keep their gap to the part. A column
//! is a loop along its edge, where a bead's centre can reach (`disc_offset`), stacked layer on
//! layer into a thin shell: all of its body. Its interface is laid in straight lines across it
//! the short way, a quarter turn from the column's axis (its greatest spread) through its origin,
//! `interface_gap` farther apart than beads that tile. Each line runs from the loop to the loop,
//! its ends on the loop's centre line, so that no line of the supports stands inside a column but
//! its top: the first layer of the interface spans the column over air from one side of the shell
//! to the other (`Rest::Air`), slowly, as a strand on air hangs; the next rest on it. Each piece
//! of the supports is an island: its loop is laid first, then its lines, in the cheapest tour. On
//! the first layer the lines take its width, to grip.

use crate::frame::hatch_in_frames;
use crate::island::{Island, Order, share};
use crate::solids::Solids;
use crate::{Path, PathError, PathParams, Role};
use layercraft_geom::{Region, bead_spacing, difference, disc_offset, intersection, polyline_length, union};
use layercraft_region::RegionPass;

/// The islands of the pass's supports, their loops and the lines inside them, over the
/// interface of the layer below (`interface_below`); `index` is the pass's number in the print.
pub fn islands(
    pass: &RegionPass,
    p: &PathParams,
    columns: &Solids,
    index: usize,
    interface_below: &Region,
) -> Result<Vec<Island>, PathError> {
    if pass.supports.is_empty() && pass.support_interface.is_empty() {
        return Ok(Vec::new());
    }
    let width = p.width_on(pass.index, p.support.width);
    let pitch = bead_spacing(width, pass.height) + p.support.interface_gap;
    // The loops' centre lines: what every line inside runs to.
    let rim = disc_offset(&union(&pass.supports, &pass.support_interface)?, -width / 2.0)?;
    let mut islands = Vec::new();
    for shape in &rim {
        let Some(outer) = shape.first() else { continue };
        let mut island = Island::new(outer.clone());
        for ring in shape {
            island.walls.push((0, Path::new(Role::SUPPORT, width, true, ring.clone())));
        }
        islands.push(island);
    }
    let spanning = difference(&pass.support_interface, interface_below)?;
    let resting = intersection(&pass.support_interface, interface_below)?;
    for (role, region) in [(Role::SUPPORT_INTERFACE, &spanning), (Role::SUPPORT_INTERFACE_RESTING, &resting)]
    {
        if region.is_empty() {
            continue;
        }
        let inside = intersection(region, &rim)?;
        let lines = hatch_in_frames(&inside, pitch, 90.0, |q| columns.frame_at(index, q))?;
        // A line shorter than its bead is wide would lay a blob.
        let lines = lines.into_iter().filter(|line| polyline_length(line) >= width);
        share(&mut islands, Order::Tour, lines.map(|line| Path::new(role, width, false, line)));
    }
    Ok(islands)
}
