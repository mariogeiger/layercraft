//! The toolpaths as a SolidCraft script, for SolidCraft to draw where no window shows a PLY (a
//! headless snapshot): one sketch per layer and role, in the layer's plane, each bead's centre
//! line a sketch line, closed where the bead loops.

use crate::role_name;
use layercraft_path::{Paths, Role};
use serde_json::{Value, json};

/// A SolidCraft script drawing every bead's centre line, layers `from..to`.
pub fn solidcraft_script(paths: &Paths, layers: std::ops::Range<usize>) -> Value {
    let mut commands = Vec::new();
    for layer in paths.layers.iter().filter(|l| layers.contains(&l.index)) {
        for role in Role::NAMED {
            let beads: Vec<_> =
                layer.paths.iter().filter(|p| p.role == role && p.points.len() >= 2).collect();
            if beads.is_empty() {
                continue;
            }
            let name = format!("L{:03} {}", layer.index, role_name(role));
            commands.push(json!({"command": "sketch.create", "params": {
                "plane": {"origin": [0.0, 0.0, layer.top], "x_dir": [1.0, 0.0, 0.0], "y_dir": [0.0, 1.0, 0.0]},
                "name": name,
            }}));
            for bead in beads {
                commands.push(json!({"command": "sketch.line", "params": {
                    "points": bead.points, "closed": bead.closed,
                }}));
            }
            commands.push(json!({"command": "sketch.finish", "params": {}}));
        }
    }
    json!({ "commands": commands })
}
