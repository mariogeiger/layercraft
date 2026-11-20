//! From the defaults (`defaults`) to each stage's parameters: a printer printing a quality in a
//! material on a plate (`read`), and the variables of the printer's G-code templates for a
//! print's filaments (`Filaments::scope`). What follows from the values is derived here, once:
//! the bending layers' bounds, how far apart sparse lines lie, how wide a band a bead crosses,
//! the toolhead's shape and the firmware's limits.

use crate::defaults::{
    BENDING, MATERIALS, Material, Plate, Printer, QUALITIES, Quality, ROLLS_PLY_LAYERS, ROLLS_TILT,
};
use layercraft_motion::{Heat, Limits, MotionParams, Templates};
use layercraft_path::{Crossing, PathParams, SparsePattern};
use layercraft_region::RegionParams;
use layercraft_section::{Foliation, Layering};
use layercraft_template::{Scope, Value, render_number};
use layercraft_verify::{Envelope, Toolhead};

/// Everything the stages need for one filament, read once.
pub struct Settings {
    pub layering: Layering,
    pub region: RegionParams,
    pub path: PathParams,
    pub motion: MotionParams,
    pub envelope: Envelope,
    /// The temperatures of the filament and the bed.
    pub heat: Heat,
    pub printer: &'static Printer,
    pub material: &'static Material,
    pub plate: Plate,
}

/// The entry named `name` among `entries`, or why there is none.
fn named<T>(entries: &'static [(&'static str, T)], name: &str, what: &str) -> Result<&'static T, String> {
    entries.iter().find(|(n, _)| *n == name).map(|(_, entry)| entry).ok_or_else(|| {
        let known: Vec<&str> = entries.iter().map(|(n, _)| *n).collect();
        format!("not supported: the {what} `{name}`; there are {}", known.join(", "))
    })
}

pub fn quality(name: &str) -> Result<&'static Quality, String> {
    named(&QUALITIES, name, "quality")
}

pub fn material(name: &str) -> Result<&'static Material, String> {
    named(&MATERIALS, name, "material")
}

/// The plate the printer's files name `name`.
pub fn plate(name: &str) -> Result<Plate, String> {
    Plate::ALL.into_iter().find(|p| p.name() == name).ok_or_else(|| {
        let known: Vec<&str> = Plate::ALL.iter().map(|p| p.name()).collect();
        format!("not supported: the plate `{name}`; there are {}", known.join(", "))
    })
}

/// Convection rolls for sparse infill, the default (`defaults::ROLLS_TILT`).
pub fn rolls(layer_height: f64) -> SparsePattern {
    SparsePattern::Rolls { tilt: ROLLS_TILT, ply: ROLLS_PLY_LAYERS * layer_height }
}

/// The firmware's motion limits as it takes them: accelerations (`M201`), speeds (`M203`),
/// print, retract and travel accelerations (`M204`) and jerks (`M205`); then the fans off.
fn limits_gcode(p: &Printer) -> String {
    let f = &p.firmware;
    let [ax, ay, az, ae] = f.max_accel.map(render_number);
    let [vx, vy, vz, ve] = f.max_speed.map(render_number);
    let [jx, jy, jz, je] = f.jerk;
    let (print, retract, travel) =
        (render_number(f.print_accel), render_number(f.retract_accel), render_number(f.travel_accel));
    format!(
        "M201 X{ax} Y{ay} Z{az} E{ae}\nM203 X{vx} Y{vy} Z{vz} E{ve}\nM204 P{print} R{retract} T{travel}\n\
         M205 X{jx:.2} Y{jy:.2} Z{jz:.2} E{je:.2}\nM106 S0\nM106 P2 S0\n"
    )
}

/// The stages' parameters for `printer` printing `quality` in `material` on `plate`.
pub fn read(
    printer: &'static Printer,
    quality: &Quality,
    material: &'static Material,
    plate: Plate,
) -> Result<Settings, String> {
    let bed = material.beds.iter().find(|(p, _)| *p == plate).map(|(_, t)| *t).ok_or_else(|| {
        format!("not supported: {} on the {}, which it does not hold to", material.kind, plate.name())
    })?;
    let nozzle = printer.nozzle_diameter;
    let q = quality;
    let w = &q.widths;
    let m = &q.motion;
    let cooling = &material.cooling;
    let shape = &printer.nozzle;
    // Bridges are laid a nozzle wide.
    let widest = [w.outer_wall, w.inner_wall, w.sparse, w.top_surface, w.solid, nozzle]
        .into_iter()
        .fold(0.0, f64::max);
    // A bead across a band narrower than `v²/a` never runs at its speed: it speeds up and stops.
    let run = |speed: f64, accel: f64| if accel > 0.0 { speed * speed / accel } else { 0.0 };
    let crossing = Crossing {
        top: run(m.top_surface.speed, m.top_surface.accel),
        inside: run(m.solid.speed, m.solid.accel),
        first_layer: run(m.first_layer_infill_speed, m.first_layer_accel),
    };
    let max_speed = printer.firmware.max_speed[0].min(printer.firmware.max_speed[1]);
    let max_speed_z = printer.firmware.max_speed[2];
    let gcode = &printer.gcode;
    Ok(Settings {
        layering: Layering {
            first_layer_height: q.first_layer_height,
            layer_height: q.layer_height,
            // The rise a layer may take: what the nozzle reaches below the rest of the head,
            // less the room kept under it.
            foliation: Some(Foliation {
                max_slope: BENDING.max_slope,
                min_layer_height: q.layer_height * (1.0 - BENDING.slab_spread),
                max_layer_height: q.layer_height * (1.0 + BENDING.slab_spread),
                max_rise: (shape.protrusion - printer.head_margin).max(0.0),
                parallel_layers: BENDING.parallel_layers,
                cell: BENDING.cell * nozzle,
                face_reach: (shape.face_diameter + widest) / 2.0,
                ironing: BENDING.ironing,
            }),
        },
        region: RegionParams {
            walls: q.walls,
            outer_wall_width: w.outer_wall,
            inner_wall_width: w.inner_wall,
            top_layers: q.top_layers,
            bottom_layers: q.bottom_layers,
            first_layer_shrink: q.first_layer_shrink,
            first_layer_width: w.first_layer,
            one_wall_on_top: q.one_wall_on_top,
            // The path stage lays sparse lines this far apart, one direction per layer.
            sparse_gap: if q.sparse_density > 0.0 {
                layercraft_geom::bead_spacing(w.sparse, q.layer_height) / q.sparse_density
            } else {
                0.0
            },
            bridge_width: nozzle,
            min_sparse_area: q.min_sparse_area,
            supports: q.supports.clone(),
        },
        path: PathParams {
            outer_wall_width: w.outer_wall,
            inner_wall_width: w.inner_wall,
            top_surface_width: w.top_surface,
            solid_width: w.solid,
            sparse_width: w.sparse,
            sparse_density: q.sparse_density,
            infill_wall_overlap: q.infill_wall_overlap,
            infill_angle: q.infill_angle,
            first_layer_width: w.first_layer,
            bridge_width: nozzle,
            monotone: q.monotone.to_vec(),
            crossing,
            resolution: q.resolution,
            sparse_pattern: rolls(q.layer_height),
            support: q.support_lines.clone(),
        },
        motion: MotionParams {
            filament_diameter: material.diameter,
            flow_ratio: material.flow_ratio,
            max_volumetric_speed: material.max_volumetric_speed,
            outer_wall: m.outer_wall.clone(),
            inner_wall: m.inner_wall.clone(),
            top_surface: m.top_surface.clone(),
            solid: m.solid.clone(),
            sparse: m.sparse.clone(),
            bridge: m.bridge.clone(),
            support: m.support.clone(),
            support_interface: m.support_interface.clone(),
            overhang_speeds: m.overhang_speeds.to_vec(),
            first_layer_wall_speed: m.first_layer_wall_speed,
            first_layer_infill_speed: m.first_layer_infill_speed,
            first_layer_accel: m.first_layer_accel,
            travel: m.travel.clone(),
            retract_length: material.retract_length,
            retract_speed: printer.retract_speed,
            deretract_speed: printer.deretract_speed,
            retract_min_travel: printer.retract_min_travel,
            seam_gap: q.seam_gap,
            wipe_distance: material.wipe_distance,
            retract_before_wipe: printer.retract_before_wipe,
            z_hop: printer.z_hop,
            lift_radius: printer.spiral_lift_radius,
            face_radius: shape.face_diameter / 2.0,
            // Every layer takes its time to cool: one laid faster stays soft under the next,
            // and sags or smears.
            slow_down_layer_time: cooling.layer_time,
            slow_down_min_speed: cooling.min_speed,
            fan_speed: cooling.fan,
            overhang_fan_speed: cooling.overhang_fan,
            overhang_fan_share: cooling.overhang_share,
            fan_off_layers: cooling.fan_off_layers,
            resolution: q.resolution,
            arc_fitting: q.arc_fitting,
            limits: Limits {
                max_accel: printer.firmware.max_accel[0].min(printer.firmware.max_accel[1]),
                jerk: printer.firmware.jerk[0].min(printer.firmware.jerk[1]),
                max_speed_xy: max_speed,
                max_speed_z,
            },
            templates: Templates {
                limits: limits_gcode(printer),
                start: gcode.start.to_string(),
                end: gcode.end.to_string(),
                layer_change: gcode.layer_change.to_string(),
                prologue: gcode.prologue.to_string(),
                change_filament: gcode.filament_change.to_string(),
            },
        },
        envelope: Envelope {
            area: printer.area,
            max_z: printer.max_z,
            max_speed,
            max_speed_z,
            filament_diameter: material.diameter,
            max_volumetric_speed: material.max_volumetric_speed,
            toolhead: Toolhead {
                face_radius: shape.face_diameter / 2.0,
                flank_slope: shape.flank_degrees.to_radians().tan(),
                column_radius: shape.column_diameter / 2.0,
                protrusion: shape.protrusion,
                margin: printer.head_margin,
            },
            ironing: BENDING.ironing,
            resolution: q.resolution,
        },
        heat: Heat { nozzle: vec![material.nozzle], bed: [bed, bed] },
        printer,
        material,
        plate,
    })
}

/// A print's filaments by material, each with the settings of its own kind: the first's shape
/// the part, and the last given stands for every material past them.
pub struct Filaments<'a> {
    pub first: &'a Settings,
    pub more: &'a [Settings],
}

impl Filaments<'_> {
    /// Material `k`'s settings.
    pub fn of(&self, k: usize) -> &Settings {
        match k.checked_sub(1) {
            Some(j) => self.more.get(j).or(self.more.last()).unwrap_or(self.first),
            None => self.first,
        }
    }

    /// The temperatures of `n` filaments, by material, and the first's bed.
    pub fn heat(&self, n: usize) -> Heat {
        let nozzle =
            (0..n.max(1)).map(|k| self.of(k).heat.nozzle.first().copied().unwrap_or_default()).collect();
        Heat { nozzle, bed: self.first.heat.bed }
    }

    /// The variables of the printer's templates, and the settings the printed file declares, for
    /// the filaments coloured `colours`, by material, every one fed to the first extruder: the
    /// print's, then one item per filament, its own, named as the printer's maker names them. The
    /// program's own (its layers, its extent, the filaments a change swaps) are the frame's.
    pub fn scope(&self, colours: &[String]) -> Scope {
        let n = colours.len().max(1);
        let s = self.first;
        let printer = s.printer;
        let materials: Vec<&Material> = (0..n).map(|k| self.of(k).material).collect();
        let num = Value::Num;
        let each =
            |item: &dyn Fn(&Material) -> Value| Value::List(materials.iter().map(|m| item(m)).collect());
        let pulled = |d: Option<f64>| (Value::Bool(d.is_some()), num(d.unwrap_or(0.0)));
        let (cut, cut_by): (Vec<Value>, Vec<Value>) =
            materials.iter().map(|m| pulled(m.withdraw_at_cut)).unzip();
        let (exit, exit_by): (Vec<Value>, Vec<Value>) =
            materials.iter().map(|m| pulled(m.withdraw_at_exit)).unzip();
        let softest = materials.iter().map(|m| m.softening).fold(f64::INFINITY, f64::min);
        let strongest = materials.iter().map(|m| m.cooling.aux_full).fold(0.0, f64::max);
        let entries = [
            ("printer_model", Value::Str(printer.name.into())),
            ("printable_height", num(printer.max_z)),
            ("nozzle_diameter", Value::List(vec![num(printer.nozzle_diameter)])),
            ("layer_height", num(s.layering.layer_height)),
            ("initial_layer_print_height", num(s.layering.first_layer_height)),
            ("initial_layer_acceleration", num(s.motion.first_layer_accel)),
            ("travel_acceleration", num(s.motion.travel.accel)),
            ("curr_bed_type", Value::Str(s.plate.name().into())),
            ("bed_temperature_initial_layer_single", num(s.heat.bed[0])),
            ("flush_length", num(printer.flush)),
            ("retract_length_toolchange", Value::List(printer.toolchange_retraction.map(num).to_vec())),
            // The nozzle printed with: the first extruder's, the printer's own, no other hot end.
            ("nozzle_diameter_at_nozzle_id", Value::List(vec![num(printer.nozzle_diameter)])),
            ("initial_nozzle_id", num(0.0)),
            ("current_nozzle_id", num(0.0)),
            ("next_nozzle_id", num(0.0)),
            ("initial_no_support_hotend", num(-1.0)),
            ("current_hotend", num(-1.0)),
            ("next_hotend", num(-1.0)),
            ("filament_map", each(&|_| num(1.0))),
            ("filament_colour", Value::List(colours.iter().map(|c| Value::Str(c.clone())).collect())),
            ("filament_type", each(&|m| Value::Str(m.kind.into()))),
            ("filament_ids", each(&|m| Value::Str(m.code.into()))),
            ("nozzle_temperature_initial_layer", each(&|m| num(m.nozzle[0]))),
            ("nozzle_temperature", each(&|m| num(m.nozzle[1]))),
            ("filament_max_volumetric_speed", each(&|m| num(m.max_volumetric_speed))),
            ("flush_volumetric_speeds", each(&|m| num(m.max_volumetric_speed))),
            ("flush_temperatures", each(&|m| num(m.flush_temperature))),
            ("filament_cooling_before_tower", each(&|m| num(m.cooling_after_change))),
            ("long_retractions_when_cut", Value::List(cut)),
            ("retraction_distances_when_cut", Value::List(cut_by)),
            ("long_retractions_when_ec", Value::List(exit)),
            ("retraction_distances_when_ec", Value::List(exit_by)),
            ("close_additional_fan_first_x_layers", each(&|m| num(m.cooling.aux_first_layers))),
            ("first_x_layer_fan_speed", each(&|m| num(m.cooling.aux_first))),
            ("additional_cooling_fan_speed", each(&|m| num(m.cooling.aux_full))),
            ("additional_fan_full_speed_layer", each(&|m| num(m.cooling.aux_full_layer))),
            // Over the print's filaments: the chamber cooled for the one softening first, at
            // the strongest auxiliary fan any asks.
            ("min_vitrification_temperature", num(softest)),
            ("max_additional_fan", num(strongest)),
        ];
        entries.into_iter().map(|(k, v)| (k.to_string(), v)).collect()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::defaults::X2D;

    fn of(quality_name: &str, material_name: &str) -> Settings {
        read(&X2D, quality(quality_name).unwrap(), material(material_name).unwrap(), X2D.plate).unwrap()
    }

    #[test]
    fn every_quality_material_and_plate_it_holds_to_reads() {
        for (q, _) in &QUALITIES {
            for (m, own) in &MATERIALS {
                for plate in Plate::ALL {
                    let read = read(&X2D, quality(q).unwrap(), material(m).unwrap(), plate);
                    assert_eq!(read.is_ok(), own.beds.iter().any(|(p, _)| *p == plate), "{q} {m} {plate:?}");
                }
            }
        }
        assert!(quality("0.16").err().unwrap_or_default().contains("0.20, 0.12"));
        assert!(material("wood").err().unwrap_or_default().contains("pla, petg-tr"));
        assert!(plate("Smooth PEI Plate").is_err());
        assert!(read(&X2D, quality("0.20").unwrap(), material("petg-tr").unwrap(), Plate::Cool).is_err());
    }

    #[test]
    fn the_firmware_takes_the_printers_limits() {
        assert_eq!(
            limits_gcode(&X2D),
            "M201 X20000 Y20000 Z500 E30000\nM203 X1000 Y1000 Z20 E30\nM204 P20000 R30000 T9000\n\
             M205 X9.00 Y9.00 Z3.00 E2.50\nM106 S0\nM106 P2 S0\n"
        );
    }

    #[test]
    fn a_band_is_crossed_where_a_bead_reaches_its_speed_and_sparse_lines_follow_the_density() {
        let s = of("0.20", "pla");
        assert_eq!(s.path.crossing, Crossing { top: 20.0, inside: 6.25, first_layer: 22.05 });
        let gap = layercraft_geom::bead_spacing(0.45, 0.2) / 0.15;
        assert!((s.region.sparse_gap - gap).abs() < 1e-12);
        let fine = of("0.12", "pla");
        let foliation = fine.layering.foliation.unwrap();
        assert!((foliation.min_layer_height - 0.06).abs() < 1e-12);
        assert!((foliation.max_layer_height - 0.18).abs() < 1e-12);
        assert!((foliation.max_rise - 2.74).abs() < 1e-12);
    }

    #[test]
    fn a_print_of_petg_and_pla_gives_each_filament_its_own_settings() {
        // PETG for the part, PLA for the supports' interface, the last standing for the third:
        // each its temperatures and its own item in the templates' per-filament lists, the
        // chamber cooled for the PLA.
        let (petg, more) = (of("0.20", "petg-tr"), [of("0.20", "pla")]);
        let filaments = Filaments { first: &petg, more: &more };
        let heat = filaments.heat(3);
        assert_eq!(heat.nozzle, [[250.0, 245.0], [220.0, 220.0], [220.0, 220.0]]);
        let grey = "#808080".to_string();
        let scope = filaments.scope(&[grey.clone(), grey]);
        let list = |items: &[Value]| Some(Value::List(items.to_vec()));
        let kinds = [Value::Str("PETG".into()), Value::Str("PLA".into())];
        assert_eq!(scope.get("filament_type").cloned(), list(&kinds));
        assert_eq!(scope.get("nozzle_temperature").cloned(), list(&[Value::Num(245.0), Value::Num(220.0)]));
        assert_eq!(scope.get("min_vitrification_temperature"), Some(&Value::Num(45.0)));
        assert_eq!(scope.get("max_additional_fan"), Some(&Value::Num(70.0)));
        assert_eq!(scope.get("flush_length"), Some(&Value::Num(0.0)));
        let pulled = [Value::Bool(true), Value::Bool(true)];
        assert_eq!(scope.get("long_retractions_when_cut").cloned(), list(&pulled));
        assert_eq!(
            scope.get("retraction_distances_when_cut").cloned(),
            list(&[Value::Num(10.0), Value::Num(10.0)])
        );
    }
}
