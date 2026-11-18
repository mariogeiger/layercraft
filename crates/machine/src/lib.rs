//! The machine description: a printer, a print process and a filament, as flat settings, and
//! the macro language of the printer's own G-code templates.
//!
//! The printer maker's profiles, read from an installed slicer at each compile (`import`), bring
//! the printer's G-code templates and the rest of what its files declare; layercraft's own
//! settings, kept in the repository, replace theirs (`set`). A setting may hold one value per
//! extruder variant (extruder × nozzle flow) or per extruder; `Machine` reads and sets the one
//! for the variant it prints with.

mod expr;
pub mod import;
mod template;
mod value;

pub use template::{Scope, TemplateError, expand};
pub use value::{Value, render_number};

use std::collections::BTreeMap;

/// Which of the three profiles a setting comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Machine,
    Process,
    Filament,
}

/// A printer, process and filament, flattened.
#[derive(Debug, Clone, PartialEq)]
pub struct Machine {
    /// The version of the slicer whose profiles and file conventions these are, such as
    /// `02.08.02.61`; the printer's firmware checks it in the files it prints.
    pub slicer_version: String,
    /// The extruder variant printed with, such as `Direct Drive Standard`.
    pub variant: String,
    pub machine: BTreeMap<String, Value>,
    pub process: BTreeMap<String, Value>,
    pub filament: BTreeMap<String, Value>,
}

/// Why a machine description cannot be used.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MachineError {
    #[error("the machine description has no `{0}`")]
    Missing(String),
    #[error("setting `{0}` is not a number: {1}")]
    NotANumber(String, String),
    #[error("setting `{0}` is out of range: {1}")]
    OutOfRange(String, String),
    #[error("cannot import the machine: {0}")]
    Import(String),
    #[error("not supported yet: {0}")]
    Unsupported(String),
}

impl Machine {
    fn table(&self, kind: Kind) -> &BTreeMap<String, Value> {
        match kind {
            Kind::Machine => &self.machine,
            Kind::Process => &self.process,
            Kind::Filament => &self.filament,
        }
    }

    /// The extruder variants a profile lists, which its per-variant settings follow.
    fn variants(&self, kind: Kind) -> Vec<String> {
        let key = match kind {
            Kind::Machine => "printer_extruder_variant",
            Kind::Process => "print_extruder_variant",
            Kind::Filament => "filament_extruder_variant",
        };
        match self.table(kind).get(key) {
            Some(Value::List(list)) => list.iter().map(Value::render).collect(),
            _ => Vec::new(),
        }
    }

    /// The position of the variant printed with in a profile's list of variants.
    fn variant_index(&self, kind: Kind) -> Option<usize> {
        self.variants(kind).iter().position(|v| *v == self.variant)
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

    /// The entry of a setting's list `len` long that belongs to the variant printed with: one
    /// per variant, one per extruder, or else the first.
    fn entry(&self, kind: Kind, len: usize) -> usize {
        let variants = self.variants(kind).len();
        let extruders = match self.machine.get("extruder_type") {
            Some(Value::List(v)) => v.len(),
            _ => 1,
        };
        if len == variants && variants > 1 {
            self.variant_index(kind).unwrap_or(0)
        } else if len == extruders && extruders > 1 {
            self.extruder_index()
        } else {
            0
        }
    }

    /// A setting's value for the variant printed with.
    pub fn get(&self, kind: Kind, key: &str) -> Result<Value, MachineError> {
        let value = self.table(kind).get(key).ok_or_else(|| MachineError::Missing(key.to_string()))?;
        let Value::List(list) = value else { return Ok(value.clone()) };
        let index = self.entry(kind, list.len());
        list.get(index).or(list.first()).cloned().ok_or_else(|| MachineError::Missing(key.to_string()))
    }

    /// Gives a setting `value` for the variant printed with: in a list of one entry per variant
    /// or per extruder, the entry `get` reads, the others kept; a list given, or a setting
    /// without a list, whole.
    pub fn set(&mut self, kind: Kind, key: &str, value: Value) {
        let entry = match (self.table(kind).get(key), &value) {
            (Some(Value::List(list)), single) if !matches!(single, Value::List(_)) => {
                Some(self.entry(kind, list.len()))
            }
            _ => None,
        };
        let table = match kind {
            Kind::Machine => &mut self.machine,
            Kind::Process => &mut self.process,
            Kind::Filament => &mut self.filament,
        };
        let slot = match (entry, table.get_mut(key)) {
            (Some(i), Some(Value::List(list))) => list.get_mut(i),
            _ => None,
        };
        match slot {
            Some(slot) => *slot = value,
            None => {
                table.insert(key.to_string(), value);
            }
        }
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
    /// The keys a kind of setting holds.
    pub fn keys(&self, kind: Kind) -> Vec<String> {
        self.table(kind).keys().cloned().collect()
    }

    pub fn scope(&self) -> Scope {
        let per_extruder = self.extruder_variants();
        let mut scope = Scope::new();
        for kind in [Kind::Machine, Kind::Process, Kind::Filament] {
            let table = self.table(kind);
            let variants = self.variants(kind);
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
