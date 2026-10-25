//! The package holds what the X2D's firmware checks before it prints a plate.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use layercraft_package::{Filament, Image, Plate, write_package};
use md5::Digest;
use std::io::{Cursor, Read};

fn plate() -> Plate {
    let image = |n: u32| Image { width: n, height: n, rgba: vec![0; (n * n * 4) as usize] };
    Plate {
        gcode: "; HEADER_BLOCK_START\nG1 X1\n".into(),
        printer_model: "N6".into(),
        nozzle_diameter: 0.4,
        filament: Filament {
            kind: "PLA".into(),
            colour: "#0A2989".into(),
            code: "GFA00".into(),
            used_m: 0.33,
            used_g: 0.99,
        },
        seconds: 118.0,
        bed_type: "textured_plate".into(),
        settings_json: "{}".into(),
        thumbnail: image(4),
        small_thumbnail: image(2),
        generator: "layercraft test".into(),
        client_version: "02.08.02.61".into(),
        object_name: "carre-test".into(),
        layers: 15,
        layer_height: 0.2,
        first_layer_seconds: 12.5,
        bounds: [[118.0, 118.0], [138.0, 138.0]],
    }
}

fn entry(zip: &mut zip::ZipArchive<Cursor<Vec<u8>>>, name: &str) -> String {
    let mut text = String::new();
    zip.by_name(name).unwrap().read_to_string(&mut text).unwrap();
    text
}

#[test]
fn the_plate_summary_describes_extruders_nozzle_and_client() {
    let mut bytes = Cursor::new(Vec::new());
    write_package(&plate(), &mut bytes).unwrap();
    let mut zip = zip::ZipArchive::new(Cursor::new(bytes.into_inner())).unwrap();
    let info = entry(&mut zip, "Metadata/slice_info.config");
    for needed in [
        r#"key="X-BBL-Client-Version" value="02.08.02.61""#,
        r#"key="extruder_type" value="0 1""#,
        r#"key="nozzle_volume_type" value="0 0""#,
        r#"key="limit_filament_maps" value="0""#,
        r#"<nozzle id="0" extruder_id="1" nozzle_diameter="0.4" volume_type="Standard"/>"#,
        r#"layer_ranges="0 14""#,
        r#"name="carre-test""#,
    ] {
        assert!(info.contains(needed), "slice_info lacks {needed}:\n{info}");
    }
    let gcode = entry(&mut zip, "Metadata/plate_1.gcode");
    let md5 = entry(&mut zip, "Metadata/plate_1.gcode.md5");
    let expected: String = md5::Md5::digest(gcode.as_bytes()).iter().map(|b| format!("{b:02X}")).collect();
    assert_eq!(md5, expected);
    assert!(entry(&mut zip, "Metadata/filament_sequence.json").contains("plate_1"));
    assert!(entry(&mut zip, "Metadata/plate_1.json").contains(r#""bbox_all":[118.0, 118.0, 138.0, 138.0]"#));
}
