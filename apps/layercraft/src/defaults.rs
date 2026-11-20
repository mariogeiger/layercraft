//! Every default layercraft prints with, written here and nowhere else: the printer's
//! measurements, limits and own G-code, each quality's and each material's settings, and
//! layercraft's own choices. The X2D's started as its maker's values for the 0.4 mm nozzle (read
//! from Bambu Studio 2.8's profiles as data, never its code), then changed where the owner's
//! prints asked; nothing is read from a slicer any more. The command line picks the quality, the
//! materials and the plate (`commands`) and a tuning file overrides motion by place (`--tune`);
//! `settings` turns these values into each stage's parameters. Lengths are in mm, speeds in
//! mm/s, accelerations in mm/s², temperatures in °C, angles in degrees, shares as fractions.

use layercraft_motion::RoleMotion;
use layercraft_path::{Face, SupportLines};
use layercraft_region::SupportParams;

/// What the command line prints with when it names nothing else.
pub const QUALITY: &str = "0.20";
pub const MATERIAL: &str = "pla";
/// A filament's colour when neither the 3MF nor the command line gives one.
pub const COLOUR: &str = "#808080";

/// A printer: what it is, where it reaches, how fast it may move, and its own G-code.
pub struct Printer {
    /// The name the printer's files give it, and its model code in the plate summary.
    pub name: &'static str,
    pub model_id: &'static str,
    /// The slicer version whose file conventions the printer's firmware checks in the files it
    /// prints.
    pub slicer_version: &'static str,
    pub nozzle_diameter: f64,
    pub nozzle: Nozzle,
    /// The room kept under the head's underside, for beads that curl up and a plate not quite
    /// flat.
    pub head_margin: f64,
    /// The printable area's lowest and highest corners, and the highest a part may reach.
    pub area: [[f64; 2]; 2],
    pub max_z: f64,
    pub firmware: Firmware,
    /// The plate printed on when the command line names none.
    pub plate: Plate,
    pub retract_speed: f64,
    pub deretract_speed: f64,
    /// The shortest travel that retracts.
    pub retract_min_travel: f64,
    /// The share of a retraction done before the wipe, the rest during it.
    pub retract_before_wipe: f64,
    /// The hop over a travel, turned as a helix this wide: the smallest that breaks the string.
    pub z_hop: f64,
    pub spiral_lift_radius: f64,
    /// The filament each change flushes: none. The printer pulls the old filament back out of
    /// the nozzle before it cuts it (each material's `withdraw_at_cut`), and what little wets the
    /// nozzle mixes into the new pass's first beads, the inner walls: the owner's choice
    /// (2026-11-18, 2026-11-20), no purge thrown away.
    pub flush: f64,
    /// Pulled back as a filament change hands the head to the new filament, one per extruder.
    pub toolchange_retraction: [f64; 2],
    pub gcode: Sequences,
}

/// The nozzle: a flat face whose flank rises to a column, which reaches `protrusion` below the
/// underside of the rest of the head. Measured on the printer (`printers/bambu-lab-x2d/`).
pub struct Nozzle {
    pub face_diameter: f64,
    /// The flank's angle above the plate.
    pub flank_degrees: f64,
    pub column_diameter: f64,
    pub protrusion: f64,
}

/// The motion limits given the firmware at the start: each axis's acceleration and speed (X, Y,
/// Z, E), the accelerations of prints, retractions and travels, and each axis's jerk.
pub struct Firmware {
    pub max_accel: [f64; 4],
    pub max_speed: [f64; 4],
    pub print_accel: f64,
    pub retract_accel: f64,
    pub travel_accel: f64,
    pub jerk: [f64; 4],
}

/// The printer's own G-code templates, each expanded with the settings and the program
/// (`layercraft_template`): its start, what follows it before the first layer, its layer change,
/// its filament change and its end.
pub struct Sequences {
    pub start: &'static str,
    pub prologue: &'static str,
    pub layer_change: &'static str,
    pub filament_change: &'static str,
    pub end: &'static str,
}

/// A build plate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Plate {
    TexturedPei,
    Cool,
    Engineering,
    HighTemp,
}

impl Plate {
    pub const ALL: [Plate; 4] = [Plate::TexturedPei, Plate::Cool, Plate::Engineering, Plate::HighTemp];

    /// The plate as the printer's files and the command line name it.
    pub fn name(self) -> &'static str {
        match self {
            Plate::TexturedPei => "Textured PEI Plate",
            Plate::Cool => "Cool Plate",
            Plate::Engineering => "Engineering Plate",
            Plate::HighTemp => "High Temp Plate",
        }
    }

    /// Its short name in the plate summary.
    pub fn code(self) -> &'static str {
        match self {
            Plate::TexturedPei => "textured_plate",
            Plate::Cool => "cool_plate",
            Plate::Engineering => "eng_plate",
            Plate::HighTemp => "hot_plate",
        }
    }
}

/// A print quality: its layers, its beads' widths, its shells, its supports and its motion.
pub struct Quality {
    pub layer_height: f64,
    pub first_layer_height: f64,
    pub walls: usize,
    pub top_layers: usize,
    pub bottom_layers: usize,
    pub widths: Widths,
    /// How far the first layer's outline is drawn in, against the squashed bead's foot.
    pub first_layer_shrink: f64,
    /// A top surface takes one wall, its solid the rest.
    pub one_wall_on_top: bool,
    /// The sparse infill's share of its area, the smallest area it fills, how far it overlaps
    /// the walls (a share of its bead), and its lines' turn from each solid's own axis.
    pub sparse_density: f64,
    pub min_sparse_area: f64,
    pub infill_wall_overlap: f64,
    pub infill_angle: f64,
    /// The faces laid in one direction, side by side.
    pub monotone: &'static [Face],
    /// The finest detail kept, and arcs written as arcs.
    pub resolution: f64,
    pub arc_fitting: bool,
    /// The gap a loop leaves at its seam.
    pub seam_gap: f64,
    pub supports: SupportParams,
    pub support_lines: SupportLines,
    pub motion: Motion,
}

/// The beads' widths, by role.
pub struct Widths {
    pub outer_wall: f64,
    pub inner_wall: f64,
    pub top_surface: f64,
    pub solid: f64,
    pub sparse: f64,
    pub first_layer: f64,
}

/// Each role's speed, acceleration and share of the flow, and the slower ones a wall takes as
/// its bead hangs: a quarter, a half, three quarters, wholly over air, and floating alone.
pub struct Motion {
    pub outer_wall: RoleMotion,
    pub inner_wall: RoleMotion,
    pub top_surface: RoleMotion,
    pub solid: RoleMotion,
    pub sparse: RoleMotion,
    pub bridge: RoleMotion,
    pub support: RoleMotion,
    pub support_interface: RoleMotion,
    pub travel: RoleMotion,
    pub overhang_speeds: [f64; 5],
    pub first_layer_wall_speed: f64,
    pub first_layer_infill_speed: f64,
    pub first_layer_accel: f64,
}

/// A filament: what it is, how it melts and flows, how hot it prints, how it is cooled, and how
/// a filament change handles it.
pub struct Material {
    /// What it is, as the printer's files and its AMS name it.
    pub kind: &'static str,
    pub code: &'static str,
    /// In g/cm³.
    pub density: f64,
    pub diameter: f64,
    pub flow_ratio: f64,
    /// The most the hot end melts, in mm³/s.
    pub max_volumetric_speed: f64,
    /// The nozzle on the first layer and on the others, and the hottest it takes, which a
    /// change flushes at.
    pub nozzle: [f64; 2],
    pub flush_temperature: f64,
    /// How much cooler than its own temperature it waits after a change before it prints.
    pub cooling_after_change: f64,
    /// The bed on each plate it holds to, on every layer.
    pub beds: &'static [(Plate, f64)],
    /// Where it softens: the chamber is cooled to stay under it.
    pub softening: f64,
    pub retract_length: f64,
    pub wipe_distance: f64,
    /// How far a change pulls it back when it cuts it, and as it leaves the extruder; `None`,
    /// not at all.
    pub withdraw_at_cut: Option<f64>,
    pub withdraw_at_exit: Option<f64>,
    pub cooling: Cooling,
}

/// How a filament is cooled.
pub struct Cooling {
    /// The part fan, a share of its full speed, off for the first layers.
    pub fan: f64,
    pub fan_off_layers: usize,
    /// The part fan over bridges, strands on air and walls hanging past `overhang_share` of
    /// their bead, when faster than `fan`: PETG, cooled gently, sets its hanging beads fast.
    pub overhang_fan: f64,
    pub overhang_share: f64,
    /// A layer takes at least this long to cool, its beads slowed to no less than `min_speed`.
    pub layer_time: f64,
    pub min_speed: f64,
    /// The auxiliary fans, in percent as the printer's G-code takes them: at `first` for the
    /// first `first_layers` layers, climbing to `full` by layer `full_layer` (0, at once), then
    /// handed to the chamber's cooling.
    pub aux_first_layers: f64,
    pub aux_first: f64,
    pub aux_full: f64,
    pub aux_full_layer: f64,
}

/// The Bambu Lab X2D with its 0.4 mm nozzle, printing from its first extruder.
pub static X2D: Printer = Printer {
    name: "Bambu Lab X2D",
    model_id: "N6",
    slicer_version: "02.08.02.61",
    nozzle_diameter: 0.4,
    nozzle: Nozzle { face_diameter: 1.0, flank_degrees: 50.0, column_diameter: 3.11, protrusion: 2.94 },
    head_margin: 0.2,
    area: [[0.0, 0.0], [256.0, 256.0]],
    max_z: 261.0,
    firmware: Firmware {
        max_accel: [20000.0, 20000.0, 500.0, 30000.0],
        max_speed: [1000.0, 1000.0, 20.0, 30.0],
        print_accel: 20000.0,
        retract_accel: 30000.0,
        travel_accel: 9000.0,
        jerk: [9.0, 9.0, 3.0, 2.5],
    },
    plate: Plate::TexturedPei,
    retract_speed: 30.0,
    deretract_speed: 30.0,
    retract_min_travel: 1.0,
    retract_before_wipe: 0.0,
    z_hop: 0.4,
    spiral_lift_radius: 1.2,
    flush: 0.0,
    toolchange_retraction: [2.0, 2.0],
    gcode: Sequences {
        start: include_str!("../../../printers/bambu-lab-x2d/start.gcode"),
        prologue: ";VT[initial_filament_id] H[initial_no_support_hotend]\nM981 S1 P20000 ;open spaghetti detector\n",
        layer_change: include_str!("../../../printers/bambu-lab-x2d/layer-change.gcode"),
        filament_change: include_str!("../../../printers/bambu-lab-x2d/filament-change.gcode"),
        end: include_str!("../../../printers/bambu-lab-x2d/end.gcode"),
    },
};

/// A role's motion at the full flow.
const fn role(speed: f64, accel: f64) -> RoleMotion {
    RoleMotion { speed, accel, flow: 1.0 }
}

/// The X2D's 0.20 mm quality: the everyday one.
const STANDARD: Quality = Quality {
    layer_height: 0.2,
    first_layer_height: 0.2,
    walls: 2,
    top_layers: 5,
    bottom_layers: 3,
    widths: Widths {
        outer_wall: 0.42,
        inner_wall: 0.45,
        top_surface: 0.42,
        solid: 0.42,
        sparse: 0.45,
        first_layer: 0.5,
    },
    first_layer_shrink: 0.15,
    one_wall_on_top: true,
    sparse_density: 0.15,
    min_sparse_area: 15.0,
    infill_wall_overlap: 0.15,
    infill_angle: 45.0,
    monotone: &[Face::Top, Face::Bottom],
    resolution: 0.012,
    arc_fitting: true,
    seam_gap: 0.06,
    // A strand held at both ends bridges any length, slowly (`motion.bridge`).
    supports: SupportParams {
        max_bridge: None,
        top_gap: 0.2,
        bottom_gap: 0.2,
        xy_gap: 0.35,
        interface_layers: 2,
        on_plate_only: false,
        column_material: None,
        interface_material: None,
    },
    support_lines: SupportLines { width: 0.42, interface_gap: 0.5 },
    motion: Motion {
        outer_wall: role(200.0, 5000.0),
        inner_wall: role(300.0, 10000.0),
        top_surface: role(200.0, 2000.0),
        solid: role(250.0, 10000.0),
        sparse: role(270.0, 10000.0),
        bridge: role(50.0, 10000.0),
        support: role(150.0, 10000.0),
        support_interface: role(80.0, 10000.0),
        travel: role(1000.0, 10000.0),
        overhang_speeds: [0.0, 50.0, 30.0, 10.0, 10.0],
        first_layer_wall_speed: 50.0,
        first_layer_infill_speed: 105.0,
        first_layer_accel: 500.0,
    },
};

/// The X2D's 0.12 mm quality: finer layers, slower and gentler, bridges thicker.
const FINE: Quality = Quality {
    layer_height: 0.12,
    bottom_layers: 5,
    supports: SupportParams { top_gap: 0.12, bottom_gap: 0.12, ..STANDARD.supports },
    motion: Motion {
        outer_wall: role(60.0, 2000.0),
        inner_wall: role(150.0, 4000.0),
        top_surface: role(150.0, 2000.0),
        solid: role(180.0, 4000.0),
        sparse: role(150.0, 4000.0),
        bridge: RoleMotion { flow: 1.5, ..role(25.0, 4000.0) },
        support: role(150.0, 4000.0),
        support_interface: role(80.0, 4000.0),
        overhang_speeds: [0.0, 40.0, 30.0, 20.0, 10.0],
        ..STANDARD.motion
    },
    ..STANDARD
};

/// The qualities, by the name the command line gives them.
pub static QUALITIES: [(&str, Quality); 2] = [("0.20", STANDARD), ("0.12", FINE)];

/// Bambu PLA Basic.
const PLA: Material = Material {
    kind: "PLA",
    code: "GFA00",
    density: 1.26,
    diameter: 1.75,
    flow_ratio: 0.98,
    max_volumetric_speed: 21.0,
    nozzle: [220.0, 220.0],
    flush_temperature: 240.0,
    cooling_after_change: 10.0,
    beds: &[
        (Plate::TexturedPei, 55.0),
        (Plate::Cool, 35.0),
        (Plate::Engineering, 55.0),
        (Plate::HighTemp, 55.0),
    ],
    softening: 45.0,
    retract_length: 0.4,
    wipe_distance: 1.0,
    withdraw_at_cut: Some(10.0),
    withdraw_at_exit: Some(3.0),
    cooling: Cooling {
        fan: 1.0,
        fan_off_layers: 1,
        overhang_fan: 1.0,
        overhang_share: 0.5,
        layer_time: 4.0,
        min_speed: 20.0,
        aux_first_layers: 1.0,
        aux_first: 40.0,
        aux_full: 70.0,
        aux_full_layer: 0.0,
    },
};

/// Bambu PETG Translucent: hotter, slower to melt, cooled gently and slowly.
const PETG_TRANSLUCENT: Material = Material {
    kind: "PETG",
    code: "GFG01",
    density: 1.25,
    flow_ratio: 0.95,
    max_volumetric_speed: 6.0,
    nozzle: [250.0, 245.0],
    flush_temperature: 270.0,
    beds: &[(Plate::TexturedPei, 70.0), (Plate::Engineering, 70.0), (Plate::HighTemp, 70.0)],
    softening: 70.0,
    cooling: Cooling {
        fan: 0.4,
        fan_off_layers: 3,
        overhang_fan: 0.9,
        overhang_share: 0.1,
        layer_time: 8.0,
        min_speed: 10.0,
        aux_first_layers: 3.0,
        aux_full: 10.0,
        ..PLA.cooling
    },
    ..PLA
};

/// The materials, by the name the command line gives them.
pub static MATERIALS: [(&str, Material); 2] = [("pla", PLA), ("petg-tr", PETG_TRANSLUCENT)];

/// How layers bend to follow a part's top surfaces (`layercraft_section::Foliation`),
/// layercraft's own choices, the same on every printer.
pub struct Bending {
    /// The steepest top surface the layers bend to follow: across one triangle a layer may lean
    /// the secant of half its largest angle more, but over any length rises no faster than
    /// along their edges, about 20°, far under the X2D nozzle's flank of 50°.
    pub max_slope: f64,
    /// How deep the nozzle's face may iron the beads of the layer it prints: on a bent layer it
    /// passes over those it laid just before, up the slope. The layers lean no more than keeps
    /// it so, about 0.77 mm times their slope with the X2D's face and beads: here 21.4°.
    pub ironing: f64,
    /// How much thinner and thicker than the layer height a bent slab may be, as a share: a
    /// half keeps the bead between half and three quarters of the nozzle's width high.
    pub slab_spread: f64,
    /// How wide the triangles layers bend on are, about, in nozzle diameters: fine enough that
    /// they cost the nozzle's face little of its reach, coarse enough that a bead crosses few of
    /// their edges. A layer meets a top surface's flat faces exactly, whatever their size.
    pub cell: f64,
    /// The layers that keep a followed surface's shape, its own included.
    pub parallel_layers: usize,
}

pub const BENDING: Bending =
    Bending { max_slope: 20.0, ironing: 0.3, slab_spread: 0.5, cell: 0.625, parallel_layers: 2 };

/// Convection rolls fill sparse areas: 30° off square to the walls, which braces them against
/// each other and against shear alike, the other way every five layers.
pub const ROLLS_TILT: f64 = 30.0;
pub const ROLLS_PLY_LAYERS: f64 = 5.0;

#[cfg(test)]
mod tests;
