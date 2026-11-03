//! Compiled end to end with the imported X2D description, which is local: the owner's pottery
//! stamp, also local and private, and a T whose bar stands on supports. Each test skips when what
//! it reads is missing.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::path::PathBuf;
use std::process::Command;

fn home() -> Option<PathBuf> {
    std::env::var_os("HOME").map(PathBuf::from)
}

#[test]
fn the_stamp_compiles_and_passes_verify() {
    let Some(home) = home() else { return };
    let stamp = home.join("Desktop/tampon/tampon.3mf");
    let machine = home.join(".config/layercraft/machines/x2d.json");
    if !stamp.exists() || !machine.exists() {
        eprintln!("skipped: no stamp or no imported X2D");
        return;
    }
    let out = std::env::temp_dir().join(format!("layercraft-dogfood-{}.gcode.3mf", std::process::id()));
    let status = Command::new(env!("CARGO_BIN_EXE_layercraft"))
        .args(["compile", stamp.to_str().unwrap(), "--out", out.to_str().unwrap()])
        .output()
        .unwrap();
    let _ = std::fs::remove_file(&out);
    assert!(status.status.success(), "{}", String::from_utf8_lossy(&status.stderr));
    let report = String::from_utf8_lossy(&status.stdout);
    assert!(report.contains("133 layers"), "{report}");
}

#[test]
fn a_cantilever_compiles_onto_supports_and_passes_verify() {
    use layercraft_mesh::{Mesh, solids::cuboid};
    let Some(home) = home() else { return };
    if !home.join(".config/layercraft/machines/x2d.json").exists() {
        eprintln!("skipped: no imported X2D");
        return;
    }
    // A T in the middle of the plate: a 4 mm stem 10 mm high under a 20 mm bar, 8 mm out on
    // either side, flat underneath.
    let mesh = Mesh {
        bodies: vec![
            cuboid("stem", [126.0, 126.0, 0.0], [130.0, 130.0, 10.0]),
            cuboid("bar", [118.0, 126.0, 10.0], [138.0, 130.0, 12.0]),
        ],
        ..Mesh::default()
    };
    let dir = std::env::temp_dir().join(format!("layercraft-supports-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (input, paths, out) = (dir.join("tee.json"), dir.join("paths.json"), dir.join("tee.gcode.3mf"));
    std::fs::write(&input, serde_json::to_string(&mesh).unwrap()).unwrap();
    let from = format!("mesh={}", input.display());
    let emit = format!("paths={}", paths.display());
    let status = Command::new(env!("CARGO_BIN_EXE_layercraft"))
        .args([
            "compile",
            "--from",
            &from,
            "--emit",
            &emit,
            "--supports",
            "on",
            "--out",
            out.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    let laid = std::fs::read_to_string(&paths).unwrap_or_default();
    let _ = std::fs::remove_dir_all(&dir);
    assert!(status.status.success(), "{}", String::from_utf8_lossy(&status.stderr));
    let paths: layercraft_path::Paths = serde_json::from_str(&laid).unwrap();
    // Under the bar (z 10), whose first strands hang 0.4 mm from z 10.2 down to 9.8: the
    // columns' body, then their interface up to 9.6, then the 0.2 mm of air.
    let kinds = |top: f64| -> Vec<layercraft_path::Role> {
        let layer = paths.layers.iter().find(|l| (l.top - top).abs() < 1e-6).unwrap();
        layer.paths.iter().map(|p| p.role).filter(|r| r.kind == layercraft_path::Kind::Support).collect()
    };
    assert!(kinds(5.0).iter().all(|&r| r == layercraft_path::Role::SUPPORT) && !kinds(5.0).is_empty());
    assert!(
        kinds(9.6).iter().all(|&r| r == layercraft_path::Role::SUPPORT_INTERFACE) && !kinds(9.6).is_empty()
    );
    assert!(kinds(9.8).is_empty() && kinds(10.0).is_empty());
}
