//! Stage 7: the `.gcode.3mf` a Bambu Lab printer prints from its storage.
//!
//! The package is a 3MF archive whose plate carries the G-code, its MD5 (upper-case hex), the
//! plate's slicing summary (`slice_info.config`: time, weight, the filament and its slot, the
//! extruders and nozzle), the filament order, the settings used and thumbnails for the printer's
//! screen. The X2D's firmware refuses a plate whose summary lacks the extruder and nozzle
//! description or names a client version it does not know (error 0500-4046), so the summary
//! states both as the slicer whose conventions the file follows writes them. Nothing in the
//! package starts a print.

use md5::{Digest, Md5};
use std::io::{Seek, Write};

/// The one filament of the plate.
#[derive(Debug, Clone, PartialEq)]
pub struct Filament {
    /// Such as `PLA`.
    pub kind: String,
    /// `#RRGGBB`.
    pub colour: String,
    /// The maker's filament code (`tray_info_idx`), such as `GFA00` for Bambu PLA Basic.
    pub code: String,
    pub used_m: f64,
    pub used_g: f64,
}

/// An RGBA image, row by row from the top.
#[derive(Debug, Clone, PartialEq)]
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

/// Everything the package holds.
#[derive(Debug, Clone, PartialEq)]
pub struct Plate {
    pub gcode: String,
    /// The printer's model code (`N6` for the X2D).
    pub printer_model: String,
    pub nozzle_diameter: f64,
    pub filament: Filament,
    pub seconds: f64,
    /// Such as `textured_plate`.
    pub bed_type: String,
    /// The settings used, as a JSON object.
    pub settings_json: String,
    pub thumbnail: Image,
    pub small_thumbnail: Image,
    /// The program that wrote the file, such as `layercraft 0.1.3`.
    pub generator: String,
    /// The version of the slicer whose file conventions the plate follows, such as
    /// `02.08.02.61`; the firmware checks it.
    pub client_version: String,
    /// The part's name, as the printer lists it.
    pub object_name: String,
    pub layers: usize,
    pub layer_height: f64,
    pub first_layer_seconds: f64,
    /// Where the plastic lies on the plate, `[min, max]` in mm.
    pub bounds: [[f64; 2]; 2],
}

/// Why the package could not be written.
#[derive(Debug)]
pub struct PackageError(pub String);

impl std::fmt::Display for PackageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "cannot write the package: {}", self.0)
    }
}

impl std::error::Error for PackageError {}

fn fail(e: impl std::fmt::Display) -> PackageError {
    PackageError(e.to_string())
}

/// Encodes an image as PNG.
pub fn png(image: &Image) -> Result<Vec<u8>, PackageError> {
    let expected = image.width as usize * image.height as usize * 4;
    if image.rgba.len() != expected {
        return Err(PackageError(format!("an image of {} bytes for {expected}", image.rgba.len())));
    }
    let mut out = Vec::new();
    let mut encoder = png::Encoder::new(&mut out, image.width, image.height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(fail)?;
    writer.write_image_data(&image.rgba).map_err(fail)?;
    writer.finish().map_err(fail)?;
    Ok(out)
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

fn slice_info(plate: &Plate) -> String {
    let f = &plate.filament;
    let d = plate.nozzle_diameter;
    format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <header>
    <header_item key="X-BBL-Client-Type" value="slicer"/>
    <header_item key="X-BBL-Client-Version" value="{client}"/>
  </header>
  <plate>
    <metadata key="index" value="1"/>
    <metadata key="extruder_type" value="0 1"/>
    <metadata key="nozzle_volume_type" value="0 0"/>
    <metadata key="printer_model_id" value="{model}"/>
    <metadata key="nozzle_diameters" value="{d},{d}"/>
    <metadata key="timelapse_type" value="0"/>
    <metadata key="prediction" value="{seconds}"/>
    <metadata key="weight" value="{weight:.2}"/>
    <metadata key="pause_count" value="0"/>
    <metadata key="first_layer_time" value="{first:.6}"/>
    <metadata key="outside" value="false"/>
    <metadata key="support_used" value="false"/>
    <metadata key="label_object_enabled" value="false"/>
    <metadata key="support_material_on_wipe_tower" value="false"/>
    <metadata key="enable_filament_dynamic_map" value="false"/>
    <metadata key="has_filament_switcher" value="false"/>
    <metadata key="filament_maps" value="1"/>
    <metadata key="limit_filament_maps" value="0"/>
    <object identify_id="1" name="{object}" skipped="false" />
    <filament id="1" tray_info_idx="{code}" type="{kind}" color="{colour}" used_m="{used_m:.2}" used_g="{used_g:.2}" group_id="0" nozzle_diameter="{d:.2}" volume_type="Standard" used_for_object="true" used_for_support="false" total_load_time="0.00" total_unload_time="0.00"/>
    <nozzle id="0" extruder_id="1" nozzle_diameter="{d}" volume_type="Standard"/>
    <layer_filament_lists>
      <layer_filament_list filament_list="0" layer_ranges="0 {last_layer}" />
    </layer_filament_lists>
  </plate>
</config>
"#,
        client = escape(&plate.client_version),
        first = plate.first_layer_seconds,
        object = escape(&plate.object_name),
        last_layer = plate.layers.saturating_sub(1),
        model = escape(&plate.printer_model),
        seconds = plate.seconds.round() as u64,
        weight = f.used_g,
        code = escape(&f.code),
        kind = escape(&f.kind),
        colour = escape(&f.colour),
        used_m = f.used_m,
        used_g = f.used_g,
    )
}

/// The plate's extent and first layer, as the printer's screen shows them.
fn plate_json(plate: &Plate) -> String {
    let [lo, hi] = plate.bounds;
    let bbox = [lo[0], lo[1], hi[0], hi[1]];
    let area = (hi[0] - lo[0]).max(0.0) * (hi[1] - lo[1]).max(0.0);
    let json = |s: &str| s.replace(['\\', '"'], "");
    format!(
        r#"{{"bbox_all":{bbox:?},"bbox_objects":[{{"area":{area},"bbox":{bbox:?},"id":1,"layer_height":{},"name":"{}"}}],"bed_type":"{}","filament_colors":["{}"],"filament_ids":[0],"first_extruder":1,"first_layer_time":{},"is_seq_print":false,"nozzle_diameter":{},"version":2}}"#,
        plate.layer_height,
        json(&plate.object_name),
        json(&plate.bed_type),
        json(&plate.filament.colour),
        plate.first_layer_seconds,
        plate.nozzle_diameter
    )
}

/// Writes the package.
pub fn write_package<W: Write + Seek>(plate: &Plate, writer: W) -> Result<(), PackageError> {
    let digest = Md5::digest(plate.gcode.as_bytes());
    let md5: String = digest.iter().map(|b| format!("{b:02X}")).collect();
    let thumbnail = png(&plate.thumbnail)?;
    let small = png(&plate.small_thumbnail)?;
    let files: Vec<(&str, Vec<u8>)> = vec![
        ("[Content_Types].xml", br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
 <Default Extension="png" ContentType="image/png"/>
 <Default Extension="gcode" ContentType="text/x.gcode"/>
</Types>"#.to_vec()),
        ("_rels/.rels", br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
 <Relationship Target="/Metadata/plate_1.png" Id="rel-2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail"/>
 <Relationship Target="/Metadata/plate_1.png" Id="rel-4" Type="http://schemas.bambulab.com/package/2021/cover-thumbnail-middle"/>
 <Relationship Target="/Metadata/plate_1_small.png" Id="rel-5" Type="http://schemas.bambulab.com/package/2021/cover-thumbnail-small"/>
</Relationships>"#.to_vec()),
        ("3D/3dmodel.model", format!(r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <metadata name="Application">{}</metadata>
 <resources>
 </resources>
 <build/>
</model>"#, escape(&plate.generator)).into_bytes()),
        ("Metadata/plate_1.gcode", plate.gcode.as_bytes().to_vec()),
        ("Metadata/plate_1.gcode.md5", md5.into_bytes()),
        ("Metadata/model_settings.config", br#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <plate>
    <metadata key="plater_id" value="1"/>
    <metadata key="plater_name" value=""/>
    <metadata key="locked" value="false"/>
    <metadata key="filament_map_mode" value="Manual"/>
    <metadata key="filament_maps" value="1"/>
    <metadata key="filament_volume_maps" value="0"/>
    <metadata key="gcode_file" value="Metadata/plate_1.gcode"/>
    <metadata key="thumbnail_file" value="Metadata/plate_1.png"/>
    <metadata key="thumbnail_no_light_file" value="Metadata/plate_no_light_1.png"/>
    <metadata key="top_file" value="Metadata/top_1.png"/>
    <metadata key="pattern_bbox_file" value="Metadata/plate_1.json"/>
  </plate>
</config>"#.to_vec()),
        ("Metadata/_rels/model_settings.config.rels", br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/Metadata/plate_1.gcode" Id="rel-1" Type="http://schemas.bambulab.com/package/2021/gcode"/>
</Relationships>"#.to_vec()),
        ("Metadata/slice_info.config", slice_info(plate).into_bytes()),
        ("Metadata/plate_1.json", plate_json(plate).into_bytes()),
        ("Metadata/filament_sequence.json", br#"{"plate_1":{"nozzle_sequence":[0],"optimal_assignment":[0],"sequence":[1]}}"#.to_vec()),
        ("Metadata/project_settings.config", plate.settings_json.as_bytes().to_vec()),
        ("Metadata/plate_1.png", thumbnail.clone()),
        ("Metadata/plate_no_light_1.png", thumbnail.clone()),
        ("Metadata/top_1.png", thumbnail),
        ("Metadata/plate_1_small.png", small),
    ];
    let mut zip = zip::ZipWriter::new(writer);
    let options = zip::write::SimpleFileOptions::default();
    for (name, bytes) in files {
        zip.start_file(name, options).map_err(fail)?;
        zip.write_all(&bytes).map_err(fail)?;
    }
    zip.finish().map_err(fail)?;
    Ok(())
}
