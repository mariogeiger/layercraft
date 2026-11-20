//! The filament each support column is laid in. A column is one body over its whole height: its
//! islands on one layer, with those they overlap on the layers next to it. Unless the parameters
//! force one (`column_material`), it takes the material the part prints on the most of the
//! layers it lays the column on, the least material on a tie: there the column adds no pass of
//! its own. Those layers are its body's, and its interface's too when the interface has no
//! material of its own (`interface_material`) and follows it. Under a part of one material,
//! every column takes it.

use crate::SupportParams;
use crate::supports::Columns;
use layercraft_geom::{DisjointSets, GeomError, Point, Region, Shape, bounds, intersection, union};
use std::cmp::Reverse;
use std::collections::BTreeMap;

/// One layer's columns, by the material that lays them, in order of material.
pub type Laid = Vec<(usize, Columns)>;

/// Each layer's `columns` split by the filament that lays them, the part printing the
/// materials `printed` on each layer.
pub fn by_filament(
    columns: Vec<Columns>,
    printed: &[Vec<usize>],
    p: &SupportParams,
) -> Result<Vec<Laid>, GeomError> {
    // Each layer's columns, body and interface as one, island by island.
    let wholes: Vec<Region> = columns
        .iter()
        .map(|c| match (c.body.is_empty(), c.interface.is_empty()) {
            (_, true) => Ok(c.body.clone()),
            (true, false) => Ok(c.interface.clone()),
            (false, false) => union(&c.body, &c.interface),
        })
        .collect::<Result<_, _>>()?;
    let materials = match p.column_material {
        Some(k) => wholes.iter().map(|whole| vec![k; whole.len()]).collect(),
        None => elected(&columns, &wholes, printed, p.interface_material.is_none())?,
    };
    columns
        .into_iter()
        .zip(&wholes)
        .zip(&materials)
        .map(|((column, whole), materials)| laid(column, whole, materials, p.interface_material))
        .collect()
}

/// A layer's `column` laid by the materials of the islands of `whole`, its interface in
/// `interface_material` when it has one.
fn laid(
    column: Columns,
    whole: &Region,
    materials: &[usize],
    interface_material: Option<usize>,
) -> Result<Laid, GeomError> {
    let Columns { body, interface } = column;
    let (follow, own) = match interface_material {
        Some(k) => (Vec::new(), Some((k, interface))),
        None => (interface, None),
    };
    let mut distinct = materials.to_vec();
    distinct.sort_unstable();
    distinct.dedup();
    let mut by: BTreeMap<usize, Columns> = BTreeMap::new();
    match distinct.as_slice() {
        [] => {}
        [m] => {
            by.insert(*m, Columns { body, interface: follow });
        }
        _ => {
            for &m in &distinct {
                let mask: Region = whole
                    .iter()
                    .zip(materials)
                    .filter(|&(_, &of)| of == m)
                    .map(|(shape, _)| shape.clone())
                    .collect();
                let body = intersection(&body, &mask)?;
                let interface = if follow.is_empty() { Vec::new() } else { intersection(&follow, &mask)? };
                by.insert(m, Columns { body, interface });
            }
        }
    }
    if let Some((k, interface)) = own {
        by.entry(k).or_default().interface = interface;
    }
    Ok(by.into_iter().filter(|(_, c)| !(c.body.is_empty() && c.interface.is_empty())).collect())
}

/// One island of a layer's columns while the columns are found: its number among the sets, its
/// shape and its box.
struct Island<'a> {
    id: usize,
    shape: &'a Shape,
    box_: Option<[Point; 2]>,
}

/// Whether two islands overlap.
fn overlap(a: &Island, b: &Island) -> Result<bool, GeomError> {
    let (Some([lo, hi]), Some([c, d])) = (a.box_, b.box_) else { return Ok(false) };
    if c[0] > hi[0] || lo[0] > d[0] || c[1] > hi[1] || lo[1] > d[1] {
        return Ok(false);
    }
    Ok(!intersection(&vec![a.shape.clone()], &vec![b.shape.clone()])?.is_empty())
}

/// The material each island of each layer's `wholes` takes, its column's: the one `printed` on
/// the most of the layers the column's filament lays it on, the least on a tie.
fn elected(
    columns: &[Columns],
    wholes: &[Region],
    printed: &[Vec<usize>],
    follows: bool,
) -> Result<Vec<Vec<usize>>, GeomError> {
    let mut layers: Vec<Vec<Island>> = Vec::with_capacity(wholes.len());
    // By island, the layer the column's filament lays it on: none for an island of the
    // interface alone, in a filament of its own.
    let mut laid_on: Vec<Option<usize>> = Vec::new();
    for (layer, (whole, column)) in wholes.iter().zip(columns).enumerate() {
        let mut here = Vec::with_capacity(whole.len());
        for shape in whole {
            let own = follows
                || column.interface.is_empty()
                || !intersection(&column.body, &vec![shape.clone()])?.is_empty();
            here.push(Island { id: laid_on.len(), shape, box_: bounds(&vec![shape.clone()]) });
            laid_on.push(own.then_some(layer));
        }
        layers.push(here);
    }
    // One column: the islands that overlap, layer on layer.
    let mut sets = DisjointSets::new(laid_on.len());
    for (here, above) in layers.iter().zip(layers.iter().skip(1)) {
        for a in here {
            for b in above {
                if overlap(a, b)? {
                    sets.unite(a.id, b.id);
                }
            }
        }
    }
    let mut material = vec![0; laid_on.len()];
    for class in sets.classes() {
        // The layers its filament lays the column on, each once: its members come layer by layer.
        let mut own: Vec<usize> = class.iter().filter_map(|&i| laid_on.get(i).copied().flatten()).collect();
        own.dedup();
        let mut tally: BTreeMap<usize, usize> = BTreeMap::new();
        for layer in own {
            let mut there = printed.get(layer).cloned().unwrap_or_default();
            there.sort_unstable();
            there.dedup();
            for m in there {
                *tally.entry(m).or_default() += 1;
            }
        }
        let chosen = tally.into_iter().max_by_key(|&(m, n)| (n, Reverse(m))).map_or(0, |(m, _)| m);
        for &i in &class {
            if let Some(slot) = material.get_mut(i) {
                *slot = chosen;
            }
        }
    }
    Ok(layers
        .iter()
        .map(|here| here.iter().map(|island| material.get(island.id).copied().unwrap_or(0)).collect())
        .collect())
}
