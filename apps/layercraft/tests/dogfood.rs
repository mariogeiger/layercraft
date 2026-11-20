//! Compiled end to end with the X2D's own settings: the owner's pottery stamp, local and private
//! (skipped when absent), a T whose bar stands on supports, a block whose sparse infill is rolls
//! unless lines are asked for, and two blocks in two colours whose changes flush nothing.
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
    if !stamp.exists() {
        eprintln!("skipped: no stamp");
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
        .args(["compile", "--from", &from, "--emit", &emit, "--out", out.to_str().unwrap()])
        .output()
        .unwrap();
    let laid = std::fs::read_to_string(&paths).unwrap_or_default();
    let _ = std::fs::remove_dir_all(&dir);
    assert!(status.status.success(), "{}", String::from_utf8_lossy(&status.stderr));
    let paths: layercraft_path::Paths = serde_json::from_str(&laid).unwrap();
    // Under the bar, whose first layer lies flat from z 10: the columns' loops, then their
    // interface up to 9.8, its first layer over air, then the 0.2 mm of air.
    use layercraft_path::Role;
    let kinds = |top: f64| -> Vec<Role> {
        let layer = paths.passes.iter().find(|l| (l.top - top).abs() < 1e-6).unwrap();
        layer.paths.iter().map(|p| p.role).filter(|r| r.kind == layercraft_path::Kind::Support).collect()
    };
    let resting = Role::SUPPORT_INTERFACE_RESTING;
    for (top, interface) in [(5.0, None), (9.6, Some(Role::SUPPORT_INTERFACE)), (9.8, Some(resting))] {
        let roles = kinds(top);
        assert!(roles.contains(&Role::SUPPORT), "z {top}: no loop");
        assert!(roles.iter().all(|&r| r == Role::SUPPORT || Some(r) == interface), "z {top}: {roles:?}");
        assert_eq!(interface.is_some(), roles.iter().any(|&r| r != Role::SUPPORT), "z {top}");
    }
    assert!(kinds(10.0).is_empty());
}

#[test]
fn sparse_infill_is_rolls_unless_lines_are_asked_for() {
    use layercraft_mesh::{Mesh, solids::cuboid};
    // A block 12 mm square and 4 mm high in the middle of the plate: sparse between its skins.
    let mesh =
        Mesh { bodies: vec![cuboid("block", [122.0, 122.0, 0.0], [134.0, 134.0, 4.0])], ..Mesh::default() };
    let dir = std::env::temp_dir().join(format!("layercraft-infill-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (input, paths, out) = (dir.join("block.json"), dir.join("paths.json"), dir.join("block.gcode.3mf"));
    std::fs::write(&input, serde_json::to_string(&mesh).unwrap()).unwrap();
    let (from, emit) = (format!("mesh={}", input.display()), format!("paths={}", paths.display()));
    let laid = |infill: &[&str]| -> String {
        let status = Command::new(env!("CARGO_BIN_EXE_layercraft"))
            .args(["compile", "--from", &from, "--emit", &emit, "--out", out.to_str().unwrap()])
            .args(infill)
            .output()
            .unwrap();
        assert!(status.status.success(), "{}", String::from_utf8_lossy(&status.stderr));
        std::fs::read_to_string(&paths).unwrap()
    };
    let (default, rolls, lines) = (laid(&[]), laid(&["--infill", "rolls"]), laid(&["--infill", "lines"]));
    let _ = std::fs::remove_dir_all(&dir);
    assert!(default == rolls, "the default infill is not rolls");
    assert!(default != lines, "rolls and lines lay the same paths");
}

#[test]
fn a_colour_change_pulls_the_old_filament_back_and_flushes_nothing() {
    use layercraft_mesh::{Material, Mesh, solids::cuboid};
    // A dark block and a light one side by side, a change on every layer: the most a flush
    // that follows the colours would ask for.
    let colour = |c: &str| Material { colour: Some(c.into()) };
    let mut light = cuboid("light", [132.0, 124.0, 0.0], [140.0, 132.0, 1.0]);
    light.material = 1;
    let mesh = Mesh {
        bodies: vec![cuboid("dark", [116.0, 124.0, 0.0], [124.0, 132.0, 1.0]), light],
        materials: vec![colour("#000000"), colour("#FFFFFF")],
    };
    let dir = std::env::temp_dir().join(format!("layercraft-flush-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (input, gcode, out) = (dir.join("two.json"), dir.join("two.gcode"), dir.join("two.gcode.3mf"));
    std::fs::write(&input, serde_json::to_string(&mesh).unwrap()).unwrap();
    let status = Command::new(env!("CARGO_BIN_EXE_layercraft"))
        .args(["compile", "--from", &format!("mesh={}", input.display())])
        .args(["--gcode", gcode.to_str().unwrap(), "--out", out.to_str().unwrap()])
        .output()
        .unwrap();
    let text = std::fs::read_to_string(&gcode).unwrap_or_default();
    let _ = std::fs::remove_dir_all(&dir);
    assert!(status.status.success(), "{}", String::from_utf8_lossy(&status.stderr));
    // The old filament pulled back out of the nozzle before it is cut, by the filament's own
    // 10 mm, and nothing flushed: what wets the nozzle goes into the new colour's first beads.
    let flushes: Vec<&str> =
        text.lines().filter(|l| l.starts_with("M620.10 A1") && l.contains(" L")).collect();
    assert!(flushes.len() > 1);
    assert!(flushes.iter().all(|l| l.contains(" L0 ")), "{flushes:?}");
    assert!(text.lines().filter(|l| l.starts_with("SYNC T")).all(|l| l.starts_with("SYNC T0 ")));
    let cuts: Vec<&str> =
        text.lines().filter(|l| l.starts_with("M620.11 P") && l.contains("old filament")).collect();
    assert!(
        !cuts.is_empty() && cuts.iter().all(|l| l.starts_with("M620.11 P1 ") && l.contains(" E-10 ")),
        "{cuts:?}"
    );
}
