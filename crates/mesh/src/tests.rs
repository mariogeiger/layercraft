//! 3MF round trips and hostile packages.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::solids::{cuboid, prism, regular_polygon};
use super::*;
use std::io::{Cursor, Write};

fn package(model: &str) -> Vec<u8> {
    let mut zip = zip::ZipWriter::new(Cursor::new(Vec::new()));
    let o = zip::write::SimpleFileOptions::default();
    zip.start_file("_rels/.rels", o).unwrap();
    zip.write_all(br#"<Relationships><Relationship Target="/3D/m.model" Id="r" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#).unwrap();
    zip.start_file("3D/m.model", o).unwrap();
    zip.write_all(model.as_bytes()).unwrap();
    zip.finish().unwrap().into_inner()
}

#[test]
fn analytic_volumes() {
    assert!((cuboid("c", [0.0; 3], [2.0, 3.0, 4.0]).volume() - 24.0).abs() < 1e-12);
    let hexagon = regular_polygon([5.0, 5.0], 2.0, 6);
    let area = 3.0 * 3f64.sqrt() / 2.0 * 4.0;
    assert!((prism("h", &hexagon, 1.0, 3.0).volume() - 2.0 * area).abs() < 1e-9);
}

#[test]
fn round_trip_keeps_geometry() {
    let mesh = Mesh { bodies: vec![cuboid("a <&> b", [0.0; 3], [1.0, 2.0, 3.0])] };
    let mut bytes = Cursor::new(Vec::new());
    write_3mf(&mesh, &mut bytes).unwrap();
    let back = read_3mf(Cursor::new(bytes.into_inner())).unwrap();
    assert_eq!(back, mesh);
}

#[test]
fn units_and_transforms_and_components() {
    let model = r#"<model unit="centimeter"><resources>
      <object id="1"><mesh><vertices><vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/></vertices>
      <triangles><triangle v1="0" v2="1" v3="2"/></triangles></mesh></object>
      <object id="2"><components><component objectid="1" transform="1 0 0 0 1 0 0 0 1 5 0 0"/></components></object>
      </resources><build><item objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 2"/></build></model>"#;
    let mesh = read_3mf(Cursor::new(package(model))).unwrap();
    assert_eq!(mesh.bodies[0].vertices, vec![[50.0, 0.0, 20.0], [60.0, 0.0, 20.0], [50.0, 10.0, 20.0]]);
}

#[test]
fn hostile_packages_are_refused() {
    let bad_index = r#"<model><resources><object id="1"><mesh><vertices><vertex x="0" y="0" z="0"/></vertices>
      <triangles><triangle v1="0" v2="1" v3="4000000000"/></triangles></mesh></object></resources><build><item objectid="1"/></build></model>"#;
    assert!(read_3mf(Cursor::new(package(bad_index))).is_err());
    let nan = r#"<model><resources><object id="1"><mesh><vertices><vertex x="NaN" y="0" z="0"/></vertices></mesh></object></resources></model>"#;
    assert!(read_3mf(Cursor::new(package(nan))).is_err());
    let cycle = r#"<model><resources><object id="1"><components><component objectid="1"/></components></object></resources><build><item objectid="1"/></build></model>"#;
    assert!(read_3mf(Cursor::new(package(cycle))).is_err());
    assert!(read_3mf(Cursor::new(b"not a zip".to_vec())).is_err());
}
