//! The supports' lines, each column's in its own frame, inside the supports' areas, which keep
//! their gap to the part: each column's body in straight lines `gap` apart past their tiling
//! spacing, at the layer's angle from the column's axis through its origin, the same way on
//! every layer, so each line stands on the one below and the lines stack into thin walls; its
//! interface across them, `interface_gap` apart, dense enough for the part to rest on. Each
//! piece of the supports is an island without walls; neighbouring lines join inside it without
//! retracting. On the first layer the lines take its width, to grip.

use crate::frame::hatch_in_frames;
use crate::link::Fill;
use crate::order::{Island, holder, on_line};
use crate::solids::Solids;
use crate::{Path, PathError, PathParams, Role};
use layercraft_geom::{bead_spacing, offset, polyline_length, union};
use layercraft_region::RegionLayer;
use std::rc::Rc;

/// The islands of the layer's supports, their lines inside them.
pub fn islands(
    layer: &RegionLayer,
    p: &PathParams,
    columns: &Solids,
    pass: usize,
) -> Result<Vec<Island>, PathError> {
    if layer.supports.is_empty() && layer.support_interface.is_empty() {
        return Ok(Vec::new());
    }
    let Some(s) = &p.support else {
        return Err(PathError::BadParams("the regions hold supports, but no support lines are set".into()));
    };
    let width = if layer.index == 0 { p.first_layer_width } else { s.width };
    let spacing = bead_spacing(width, layer.height);
    let all = union(&layer.supports, &layer.support_interface)?;
    let mut islands: Vec<Island> =
        all.iter().filter_map(|shape| Some(Island::bare(shape.first()?.clone()))).collect();
    let mut fills = Vec::new();
    let areas = [
        (Role::SUPPORT, &layer.supports, spacing + s.gap, p.infill_angle),
        (Role::SUPPORT_INTERFACE, &layer.support_interface, spacing + s.interface_gap, p.infill_angle + 90.0),
    ];
    for (role, region, pitch, angle) in areas {
        if region.is_empty() {
            continue;
        }
        for line in
            hatch_in_frames(&offset(region, -width / 2.0)?, pitch, angle, |q| columns.frame_at(pass, q))?
        {
            // A line shorter than its bead is wide would lay a blob.
            if polyline_length(&line) < width {
                continue;
            }
            let home = on_line(&line).and_then(|q| holder(islands.iter().map(Island::outline), q));
            if let Some(island) = home.and_then(|i| islands.get_mut(i)) {
                island.push(Path::new(role, width, false, line));
            }
        }
        fills.push(Fill { role, reach: region.clone(), max_link: 3.0 * pitch, monotone: false });
    }
    let fills = Rc::new(fills);
    for island in &mut islands {
        island.share(fills.clone());
    }
    Ok(islands)
}
