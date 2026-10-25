//! The owner's pottery stamp, compiled end to end with the imported X2D description. Both are
//! local and private: the test skips when either is missing.
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
