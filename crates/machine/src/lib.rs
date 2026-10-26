//! The machine description: a printer, a print process and a filament, as flat settings, and
//! the macro language of the printer's own G-code templates.
//!
//! Settings come from the printer maker's profiles, imported once from an installed slicer
//! (`import`) and kept outside the repository. A setting may hold one value per extruder
//! variant (extruder × nozzle flow) or per extruder; `Machine` reads the one for the variant
//! it prints with.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod expr;
pub mod import;
mod template;
mod value;

pub use template::{Scope, TemplateError, expand};
pub use value::{Value, render_number};

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Which of the three profiles a setting comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Machine,
    Process,
    Filament,
}

/// A printer, process and filament, flattened.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Machine {
    /// Where the settings came from (slicer version and profile names).
    pub source: String,
    /// The version of the slicer whose profiles and file conventions these are, such as
    /// `02.08.02.61`; the printer's firmware checks it in the files it prints.
    #[serde(default)]
    pub slicer_version: String,
    /// The extruder variant printed with, such as `Direct Drive Standard`.
    pub variant: String,
    pub machine: BTreeMap<String, Value>,
    pub process: BTreeMap<String, Value>,
    pub filament: BTreeMap<String, Value>,
}

/// Why a machine description cannot be used.
#[derive(Debug, Clone, PartialEq)]
pub enum MachineError {
    Missing(String),
    NotANumber(String, String),
    Import(String),
    Unsupported(String),
}

impl std::fmt::Display for MachineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MachineError::Missing(k) => write!(f, "the machine description has no `{k}`"),
            MachineError::NotANumber(k, v) => write!(f, "setting `{k}` is not a number: {v}"),
            MachineError::Import(e) => write!(f, "cannot import the machine: {e}"),
            MachineError::Unsupported(e) => write!(f, "not supported yet: {e}"),
        }
    }
}

impl std::error::Error for MachineError {}

impl Machine {
    fn table(&self, kind: Kind) -> &BTreeMap<String, Value> {
        match kind {
            Kind::Machine => &self.machine,
            Kind::Process => &self.process,
            Kind::Filament => &self.filament,
        }
    }

    /// The position of the variant printed with in a profile's list of variants.
    fn variant_index(&self, kind: Kind) -> Option<usize> {
        let key = match kind {
            Kind::Machine => "printer_extruder_variant",
            Kind::Process => "print_extruder_variant",
            Kind::Filament => "filament_extruder_variant",
        };
        match self.table(kind).get(key) {
            Some(Value::List(list)) => list.iter().position(|v| v.render() == self.variant),
            _ => None,
        }
    }

    /// The extruder (0-based) the variant belongs to.
    fn extruder_index(&self) -> usize {
        let ids = self.machine.get("printer_extruder_id");
        match (ids, self.variant_index(Kind::Machine)) {
            (Some(Value::List(ids)), Some(i)) => {
                ids.get(i).and_then(Value::number).map(|n| (n as usize).saturating_sub(1)).unwrap_or(0)
            }
            _ => 0,
        }
    }

    /// A setting's value for the variant printed with.
    pub fn get(&self, kind: Kind, key: &str) -> Result<Value, MachineError> {
        let value = self.table(kind).get(key).ok_or_else(|| MachineError::Missing(key.to_string()))?;
        let Value::List(list) = value else { return Ok(value.clone()) };
        let variants = match self.table(kind).get(match kind {
            Kind::Machine => "printer_extruder_variant",
            Kind::Process => "print_extruder_variant",
            Kind::Filament => "filament_extruder_variant",
        }) {
            Some(Value::List(v)) => v.len(),
            _ => 0,
        };
        let extruders = match self.machine.get("extruder_type") {
            Some(Value::List(v)) => v.len(),
            _ => 1,
        };
        let index = if list.len() == variants && variants > 1 {
            self.variant_index(kind).unwrap_or(0)
        } else if list.len() == extruders && extruders > 1 {
            self.extruder_index()
        } else {
            0
        };
        list.get(index).or(list.first()).cloned().ok_or_else(|| MachineError::Missing(key.to_string()))
    }

    /// A setting's whole value, lists included, such as the corners of the printable area.
    pub fn get_all(&self, kind: Kind, key: &str) -> Result<Value, MachineError> {
        self.table(kind).get(key).cloned().ok_or_else(|| MachineError::Missing(key.to_string()))
    }

    /// A numeric setting; `15%` reads as 15.
    pub fn number(&self, kind: Kind, key: &str) -> Result<f64, MachineError> {
        let value = self.get(kind, key)?;
        value
            .number()
            .filter(|v| v.is_finite())
            .ok_or_else(|| MachineError::NotANumber(key.to_string(), value.render()))
    }

    /// A text setting, such as a G-code template.
    pub fn text(&self, kind: Kind, key: &str) -> Result<String, MachineError> {
        Ok(self.get(kind, key)?.render())
    }

    /// The variant of each extruder: its type with its default nozzle flow, such as
    /// `Direct Drive Standard` for the first extruder.
    fn extruder_variants(&self) -> Vec<String> {
        let list = |key: &str| match self.machine.get(key) {
            Some(Value::List(v)) => v.iter().map(Value::render).collect(),
            Some(v) => vec![v.render()],
            None => Vec::new(),
        };
        let flows = list("default_nozzle_volume_type");
        list("extruder_type")
            .into_iter()
            .enumerate()
            .map(|(i, t)| format!("{t} {}", flows.get(i).map(String::as_str).unwrap_or("Standard")))
            .collect()
    }

    /// Every setting as a template variable, the way the slicer's own configuration holds it:
    /// a per-variant list becomes one value per extruder (machine and process) or per filament
    /// (filament, here one). Filament settings override process ones, which override machine ones.
    pub fn scope(&self) -> Scope {
        let per_extruder = self.extruder_variants();
        let mut scope = Scope::new();
        for kind in [Kind::Machine, Kind::Process, Kind::Filament] {
            let table = self.table(kind);
            let variants: Vec<String> = match table.get(match kind {
                Kind::Machine => "printer_extruder_variant",
                Kind::Process => "print_extruder_variant",
                Kind::Filament => "filament_extruder_variant",
            }) {
                Some(Value::List(v)) => v.iter().map(Value::render).collect(),
                _ => Vec::new(),
            };
            let pick = |list: &[Value], name: &str| {
                variants.iter().position(|v| v == name).and_then(|i| list.get(i)).cloned()
            };
            for (key, value) in table {
                let value = match value {
                    Value::List(list) if variants.len() > 1 && list.len() == variants.len() => match kind {
                        Kind::Filament => Value::List(pick(list, &self.variant).into_iter().collect()),
                        _ => Value::List(per_extruder.iter().filter_map(|name| pick(list, name)).collect()),
                    },
                    other => other.clone(),
                };
                scope.insert(key.clone(), value);
            }
        }
        scope
    }
}

#[cfg(test)]
mod tests;
