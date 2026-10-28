//! The stages' parameters read from a Bambu Lab machine description.
//!
//! Bambu's settings follow conventions of their own: a line width of 0 means the default
//! width, a percentage is of the nozzle diameter; an acceleration of 0 means the default one,
//! a percentage is of it; a filament's `nil` leaves the printer's value. This file holds those
//! conventions so the stages see plain numbers.

use layercraft_machine::{Kind, Machine, MachineError, Scope, Value};
use layercraft_motion::{Limits, MotionParams, RoleMotion, Templates, placeholders};
use layercraft_path::{Face, PathParams};
use layercraft_region::RegionParams;
use layercraft_section::Layering;
use layercraft_verify::{Envelope, Toolhead};

/// Everything the stages need, read once.
pub struct Settings {
    pub layering: Layering,
    pub region: RegionParams,
    pub path: PathParams,
    pub motion: MotionParams,
    pub envelope: Envelope,
    pub scope: Scope,
    pub bed_type: String,
    pub filament_kind: String,
    pub filament_code: String,
    pub filament_density: f64,
    pub printer_model: String,
    pub nozzle_diameter: f64,
}

fn fraction(m: &Machine, kind: Kind, key: &str) -> Result<f64, MachineError> {
    Ok(m.number(kind, key)? / 100.0)
}

/// A setting that may be a percentage of `whole`; a plain number stands for itself.
fn share_of(m: &Machine, kind: Kind, key: &str, whole: f64) -> Result<f64, MachineError> {
    let v = m.number(kind, key)?;
    Ok(if m.text(kind, key)?.trim().ends_with('%') { v / 100.0 * whole } else { v })
}

/// A filament's own value of a setting, or the printer's where the filament has none (`nil`).
fn filament_or(m: &Machine, filament_key: &str, machine_key: &str) -> Result<f64, MachineError> {
    match m.get(Kind::Filament, filament_key).ok().and_then(|v| v.number()) {
        Some(v) => Ok(v),
        None => m.number(Kind::Machine, machine_key),
    }
}

fn width(m: &Machine, key: &str, nozzle: f64) -> Result<f64, MachineError> {
    let w = share_of(m, Kind::Process, key, nozzle)?;
    if w > 0.0 { Ok(w) } else { m.number(Kind::Process, "line_width") }
}

fn accel(m: &Machine, key: &str) -> Result<f64, MachineError> {
    let default = m.number(Kind::Process, "default_acceleration")?;
    // A role without an acceleration of its own moves at the default one.
    let a = share_of(m, Kind::Process, key, default).unwrap_or(default);
    Ok(if a > 0.0 { a } else { default })
}

fn role(m: &Machine, speed: &str, acceleration: &str) -> Result<RoleMotion, MachineError> {
    Ok(RoleMotion { speed: m.number(Kind::Process, speed)?, accel: accel(m, acceleration)?, flow: 1.0 })
}

/// The radius of a spiral lift's helix, in mm: the smallest that broke the string at the end of
/// a print. Measured on the X2D with Bambu PLA Basic, one 10 mm cube per radius, the print ending
/// on its lift: at 0 the nozzle pulled up a 3 mm string, at 0.6 a 10 mm one, at 1.2 and 2.4 none.
/// A larger turn only costs time, about `2π·√(r/a)` per lift at its centripetal limit.
const SPIRAL_LIFT_RADIUS: f64 = 1.2;

/// How much wider the nozzle grows per mm up from its tip, `tan 60°`: a cone wider than any
/// nozzle's, until the X2D's is measured; the machine description does not give it.
const NOZZLE_FLARE: f64 = 1.732_050_807_568_877_2;

/// The room kept under the head past the nozzle's tip, in mm: for overhangs that curl up and a
/// plate not quite flat.
const HEAD_MARGIN: f64 = 1.0;

/// The hop and the radius of the helix a lift turns; a filament's own lift wins over the
/// printer's. Bambu's automatic lift is taken as the spiral one.
fn lift(m: &Machine) -> Result<(f64, f64), MachineError> {
    use Kind::{Filament, Machine as M};
    let z_hop = filament_or(m, "filament_z_hop", "z_hop")?;
    let kind = match m.text(Filament, "filament_z_hop_types").ok().filter(|t| t != "nil") {
        Some(t) => t,
        None => m.text(M, "z_hop_types").unwrap_or_else(|_| "Normal Lift".into()),
    };
    let radius = match kind.as_str() {
        "Normal Lift" => 0.0,
        "Spiral Lift" | "Auto Lift" => SPIRAL_LIFT_RADIUS,
        other => return Err(MachineError::Unsupported(format!("the lift `{other}`"))),
    };
    Ok((z_hop, radius))
}

/// The bed's first-layer and other-layer temperature keys for a plate type.
fn plate_keys(bed_type: &str) -> (&'static str, &'static str) {
    match bed_type {
        "Cool Plate" => ("cool_plate_temp_initial_layer", "cool_plate_temp"),
        "Engineering Plate" => ("eng_plate_temp_initial_layer", "eng_plate_temp"),
        "High Temp Plate" => ("hot_plate_temp_initial_layer", "hot_plate_temp"),
        "Smooth PEI Plate" => ("supertack_plate_temp_initial_layer", "supertack_plate_temp"),
        _ => ("textured_plate_temp_initial_layer", "textured_plate_temp"),
    }
}

/// The short plate name of `slice_info`'s bed type.
pub fn plate_code(bed_type: &str) -> &'static str {
    match bed_type {
        "Cool Plate" => "cool_plate",
        "Engineering Plate" => "eng_plate",
        "High Temp Plate" => "hot_plate",
        _ => "textured_plate",
    }
}

fn area(m: &Machine) -> Result<[[f64; 2]; 2], MachineError> {
    let Value::List(corners) = m.get_all(Kind::Machine, "printable_area")? else {
        return Err(MachineError::Missing("printable_area".into()));
    };
    let mut lo = [f64::MAX; 2];
    let mut hi = [f64::MIN; 2];
    for c in corners {
        let text = c.render();
        let mut xy = text.trim().split('x').map(|v| v.trim().parse::<f64>());
        if let (Some(Ok(x)), Some(Ok(y))) = (xy.next(), xy.next()) {
            lo = [lo[0].min(x), lo[1].min(y)];
            hi = [hi[0].max(x), hi[1].max(y)];
        }
    }
    if lo[0] > hi[0] {
        return Err(MachineError::NotANumber("printable_area".into(), String::new()));
    }
    Ok([lo, hi])
}

/// The machine's motion limits as the firmware takes them: accelerations (`M201`), speeds
/// (`M203`), print, retract and travel accelerations (`M204`) and jerks (`M205`).
fn limits_gcode(m: &Machine) -> Result<String, MachineError> {
    let n = |key: &str| m.number(Kind::Machine, key);
    let f = |v: f64| layercraft_machine::render_number(v);
    Ok(format!(
        "M201 X{} Y{} Z{} E{}\nM203 X{} Y{} Z{} E{}\nM204 P{} R{} T{}\nM205 X{:.2} Y{:.2} Z{:.2} E{:.2}\nM106 S0\nM106 P2 S0\n",
        f(n("machine_max_acceleration_x")?),
        f(n("machine_max_acceleration_y")?),
        f(n("machine_max_acceleration_z")?),
        f(n("machine_max_acceleration_e")?),
        f(n("machine_max_speed_x")?),
        f(n("machine_max_speed_y")?),
        f(n("machine_max_speed_z")?),
        f(n("machine_max_speed_e")?),
        f(n("machine_max_acceleration_extruding")?),
        f(n("machine_max_acceleration_retracting")?),
        f(n("machine_max_acceleration_travel")?),
        n("machine_max_jerk_x")?,
        n("machine_max_jerk_y")?,
        n("machine_max_jerk_z")?,
        n("machine_max_jerk_e")?,
    ))
}

/// Reads every stage's parameters from the machine description.
pub fn read(m: &Machine, bed_type: Option<&str>) -> Result<Settings, MachineError> {
    use Kind::{Filament, Machine as M, Process};
    let nozzle = m.number(M, "nozzle_diameter")?;
    let bed_type = match bed_type {
        Some(b) => b.to_string(),
        None => m.text(M, "default_bed_type").unwrap_or_else(|_| "Textured PEI Plate".into()),
    };
    let layer_height = m.number(Process, "layer_height")?;
    let shell = |layers: &str, thickness: &str| -> Result<usize, MachineError> {
        let n = m.number(Process, layers)?;
        let t = m.number(Process, thickness).unwrap_or(0.0);
        Ok(n.max((t / layer_height).ceil()).max(0.0) as usize)
    };
    let outer = width(m, "outer_wall_line_width", nozzle)?;
    let inner = width(m, "inner_wall_line_width", nozzle)?;
    let sparse_width = width(m, "sparse_infill_line_width", nozzle)?;
    let (first_bed_key, _) = plate_keys(&bed_type);
    let bed_first = m.number(Filament, first_bed_key)?;
    let template = |kind: Kind, key: &str| m.text(kind, key).unwrap_or_default();
    let filament_diameter = m.number(Filament, "filament_diameter")?;
    let max_volumetric_speed = m.number(Filament, "filament_max_volumetric_speed")?;
    let mut scope = m.scope();
    scope.extend(placeholders(&bed_type, bed_first, nozzle));
    // A filament's flush speed and temperature of 0 stand for its melt limit and its highest
    // printing temperature.
    let or = |key: &str, fallback: f64| match m.number(Filament, key) {
        Ok(v) if v > 0.0 => v,
        _ => fallback,
    };
    let hottest = m.number(Filament, "nozzle_temperature_range_high")?;
    scope.insert(
        "flush_volumetric_speeds".into(),
        Value::List(vec![Value::Num(or("filament_flush_volumetric_speed", max_volumetric_speed))]),
    );
    scope.insert(
        "flush_temperatures".into(),
        Value::List(vec![Value::Num(or("filament_flush_temp", hottest))]),
    );
    // Over the print's filaments (here one): the lowest softening point, the strongest extra fan.
    scope.insert(
        "min_vitrification_temperature".into(),
        Value::Num(m.number(Filament, "temperature_vitrification")?),
    );
    scope
        .insert("max_additional_fan".into(), Value::Num(m.number(Filament, "additional_cooling_fan_speed")?));
    let max_z = m.number(M, "printable_height")?;
    let max_speed = m.number(M, "machine_max_speed_x")?.min(m.number(M, "machine_max_speed_y")?);
    let (z_hop, lift_radius) = lift(m)?;
    // The filament's overhang fan, when on: over bridges, and walls past its threshold.
    let (overhang_fan_speed, overhang_fan_share) =
        if m.number(Filament, "enable_overhang_bridge_fan").unwrap_or(0.0) > 0.0 {
            let share = fraction(m, Filament, "overhang_fan_threshold")?;
            (fraction(m, Filament, "overhang_fan_speed")?.clamp(0.0, 1.0), share.clamp(0.0, 1.0))
        } else {
            (0.0, 0.0)
        };
    Ok(Settings {
        layering: Layering {
            first_layer_height: m.number(Process, "initial_layer_print_height")?,
            layer_height,
        },
        region: RegionParams {
            walls: m.number(Process, "wall_loops")?.max(0.0) as usize,
            outer_wall_width: outer,
            inner_wall_width: inner,
            top_layers: shell("top_shell_layers", "top_shell_thickness")?,
            bottom_layers: shell("bottom_shell_layers", "bottom_shell_thickness")?,
            first_layer_shrink: m.number(Process, "elefant_foot_compensation").unwrap_or(0.0),
            first_layer_width: width(m, "initial_layer_line_width", nozzle)?,
            one_wall_on_top: m.number(Process, "only_one_wall_top").unwrap_or(0.0) > 0.0,
            // The path stage lays sparse lines this far apart, one direction per layer.
            sparse_gap: match fraction(m, Process, "sparse_infill_density")? {
                d if d > 0.0 => layercraft_geom::bead_spacing(sparse_width, layer_height) / d,
                _ => 0.0,
            },
            bridge_width: nozzle,
            min_sparse_area: m.number(Process, "minimum_sparse_infill_area").unwrap_or(0.0),
        },
        path: PathParams {
            outer_wall_width: outer,
            inner_wall_width: inner,
            top_surface_width: width(m, "top_surface_line_width", nozzle)?,
            solid_width: width(m, "internal_solid_infill_line_width", nozzle)?,
            sparse_width,
            sparse_density: fraction(m, Process, "sparse_infill_density")?,
            infill_wall_overlap: fraction(m, Process, "infill_wall_overlap")?,
            infill_angle: m.number(Process, "infill_direction")?,
            first_layer_width: width(m, "initial_layer_line_width", nozzle)?,
            bridge_width: nozzle,
            // Bambu's monotonic patterns, for the faces they name.
            monotone: [("top_surface_pattern", Face::Top), ("bottom_surface_pattern", Face::Bottom)]
                .into_iter()
                .filter(|(key, _)| m.text(Process, key).is_ok_and(|p| p.starts_with("monotonic")))
                .map(|(_, face)| face)
                .collect(),
            resolution: m.number(Process, "resolution")?,
        },
        motion: MotionParams {
            filament_diameter,
            flow_ratio: m.number(Filament, "filament_flow_ratio")?,
            max_volumetric_speed,
            outer_wall: role(m, "outer_wall_speed", "outer_wall_acceleration")?,
            inner_wall: role(m, "inner_wall_speed", "inner_wall_acceleration")?,
            top_surface: RoleMotion {
                flow: m.number(Process, "top_solid_infill_flow_ratio").unwrap_or(1.0),
                ..role(m, "top_surface_speed", "top_surface_acceleration")?
            },
            solid: role(m, "internal_solid_infill_speed", "internal_solid_infill_acceleration")?,
            sparse: role(m, "sparse_infill_speed", "sparse_infill_acceleration")?,
            bridge: RoleMotion {
                flow: m.number(Process, "bridge_flow").unwrap_or(1.0),
                ..role(m, "bridge_speed", "bridge_acceleration")?
            },
            // Bambu's quarters of a bead's width over air, then a bead wholly over it.
            overhang_speeds: if m.number(Process, "enable_overhang_speed").unwrap_or(0.0) > 0.0 {
                ["overhang_1_4_speed", "overhang_2_4_speed", "overhang_3_4_speed", "overhang_4_4_speed", "overhang_totally_speed"]
                    .iter()
                    .map(|key| m.number(Process, key))
                    .collect::<Result<_, _>>()?
            } else {
                Vec::new()
            },
            first_layer_wall_speed: m.number(Process, "initial_layer_speed")?,
            first_layer_infill_speed: m.number(Process, "initial_layer_infill_speed")?,
            first_layer_accel: accel(m, "initial_layer_acceleration")?,
            travel: role(m, "travel_speed", "travel_acceleration")?,
            retract_length: filament_or(m, "filament_retraction_length", "retraction_length")?,
            retract_speed: m.number(M, "retraction_speed")?,
            deretract_speed: m.number(M, "deretraction_speed")?,
            retract_min_travel: m.number(M, "retraction_minimum_travel")?,
            // The process's seam gap, a percentage of the nozzle; 15 % when the profile is silent,
            // the gap Bambu Studio leaves on the X2D.
            seam_gap: share_of(m, Process, "seam_gap", nozzle).unwrap_or(0.15 * nozzle),
            // A filament's own wipe settings win over the printer's; `nil` leaves the printer's.
            wipe_distance: if m.number(M, "wipe").unwrap_or(0.0) > 0.0 {
                filament_or(m, "filament_wipe_distance", "wipe_distance")?
            } else {
                0.0
            },
            retract_before_wipe: filament_or(m, "filament_retract_before_wipe", "retract_before_wipe")
                .unwrap_or(0.0)
                / 100.0,
            z_hop,
            lift_radius,
            // Every layer takes its time to cool, whatever `slow_down_for_layer_cooling` says: one
            // laid faster stays soft under the next, and sags or smears.
            slow_down_layer_time: m.number(Filament, "slow_down_layer_time")?,
            slow_down_min_speed: m.number(Filament, "slow_down_min_speed")?,
            fan_speed: fraction(m, Filament, "fan_max_speed")?.clamp(0.0, 1.0),
            overhang_fan_speed,
            overhang_fan_share,
            fan_off_layers: m.number(Filament, "close_fan_the_first_x_layers")?.max(0.0) as usize,
            resolution: m.number(Process, "resolution")?,
            arc_fitting: m.number(Process, "enable_arc_fitting")? > 0.0,
            limits: Limits {
                max_accel: m.number(M, "machine_max_acceleration_x")?,
                jerk: m.number(M, "machine_max_jerk_x")?,
                max_speed_xy: max_speed,
                max_speed_z: m.number(M, "machine_max_speed_z")?,
            },
            templates: Templates {
                limits: limits_gcode(m)?,
                start: template(M, "machine_start_gcode"),
                end: template(M, "machine_end_gcode"),
                layer_change: template(M, "layer_change_gcode"),
                filament_start: template(Filament, "filament_start_gcode"),
                // Bambu Lab printers expect the filament's virtual tool named, and their
                // spaghetti detector turned on, before the first layer.
                prologue: ";VT[initial_filament_id] H[initial_no_support_hotend]\nM981 S1 P20000 ;open spaghetti detector\n".into(),
                filament_end: template(Filament, "filament_end_gcode"),
            },
        },
        envelope: Envelope {
            area: area(m)?,
            max_z,
            max_speed,
            filament_diameter,
            max_volumetric_speed,
            toolhead: Toolhead {
                tip_radius: nozzle / 2.0,
                flare: NOZZLE_FLARE,
                nozzle_height: m.number(M, "nozzle_height")?,
                head_radius: m.number(M, "extruder_clearance_max_radius")?,
                gantry_height: m.number(M, "extruder_clearance_height_to_rod")?,
                margin: HEAD_MARGIN,
            },
        },
        scope,
        filament_kind: m.text(Filament, "filament_type")?,
        filament_code: m.text(Filament, "filament_id").unwrap_or_default(),
        filament_density: m.number(Filament, "filament_density")?,
        printer_model: m.text(M, "model_id").unwrap_or_default(),
        nozzle_diameter: nozzle,
        bed_type,
    })
}
