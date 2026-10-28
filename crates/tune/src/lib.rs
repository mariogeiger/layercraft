//! Parameters that differ by place.
//!
//! A tuning is a list of overrides; each holds a box of the build volume and part of the
//! settings, as the settings' own JSON. An item of a stage (a shape of a layer, an island, a
//! bead) takes its stage's base parameters with every override whose box holds the item's
//! centre merged in, in order (JSON merge patch, RFC 7386). One mechanism varies any parameter
//! of any stage by height, by object or by any box, and an empty tuning is the base everywhere.
//! A box is half-open on each axis it bounds, `[from, to)`, so adjacent boxes share no point.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// The most overrides a tuning holds.
pub const MAX_OVERRIDES: usize = 10_000;

/// A box of the build volume, in mm; an axis left out is unbounded.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Within {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub y: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub z: Option<[f64; 2]>,
}

impl Within {
    /// Whether the box holds `p`.
    pub fn holds(&self, p: [f64; 3]) -> bool {
        [self.x, self.y, self.z].iter().zip(p).all(|(range, v)| range.is_none_or(|[a, b]| a <= v && v < b))
    }
}

/// Part of the settings, applied inside a box.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Override {
    #[serde(default)]
    pub within: Within,
    /// The settings changed, keyed by stage, such as `{"motion": {"lift_radius": 0.6}}`.
    pub set: Value,
}

/// The overrides, in the order they apply.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Tuning {
    pub overrides: Vec<Override>,
}

/// Why parameters could not be resolved.
#[derive(Debug, Clone, PartialEq)]
pub struct TuneError(pub String);

impl std::fmt::Display for TuneError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "tuning: {}", self.0)
    }
}

impl std::error::Error for TuneError {}

impl Tuning {
    /// Checks that every override sets only stages named in `stages`.
    pub fn check_stages(&self, stages: &[&str]) -> Result<(), TuneError> {
        for (i, o) in self.overrides.iter().enumerate() {
            if let Some(stage) =
                o.set.as_object().into_iter().flat_map(|m| m.keys()).find(|k| !stages.contains(&k.as_str()))
            {
                return Err(TuneError(format!("override {i}: `{stage}` is not one of {stages:?}")));
            }
        }
        Ok(())
    }

    /// Checks the tuning read from a file: a bounded number of overrides, finite boxes.
    pub fn check(&self) -> Result<(), TuneError> {
        if self.overrides.len() > MAX_OVERRIDES {
            return Err(TuneError(format!("more than {MAX_OVERRIDES} overrides")));
        }
        for (i, o) in self.overrides.iter().enumerate() {
            let w = &o.within;
            if ![w.x, w.y, w.z].iter().flatten().flatten().all(|v| v.is_finite()) {
                return Err(TuneError(format!("override {i}: a box bound is not finite")));
            }
            if !o.set.is_object() {
                return Err(TuneError(format!("override {i}: `set` is not an object of stages")));
            }
        }
        Ok(())
    }
}

/// Merges `patch` into `target` (RFC 7386): objects key by key, `null` removes, anything else
/// replaces.
pub fn merge_patch(target: &mut Value, patch: &Value) {
    let Value::Object(changes) = patch else {
        *target = patch.clone();
        return;
    };
    if !target.is_object() {
        *target = Value::Object(Default::default());
    }
    if let Value::Object(fields) = target {
        for (key, change) in changes {
            if change.is_null() {
                fields.remove(key);
            } else {
                merge_patch(fields.entry(key.clone()).or_insert(Value::Null), change);
            }
        }
    }
}

/// The first key of `patch` that `base` lacks, as a dotted path: a misspelt setting.
fn unknown_key(base: &Value, patch: &Value, at: &str) -> Option<String> {
    let (Value::Object(fields), Value::Object(changes)) = (base, patch) else { return None };
    changes.iter().find_map(|(key, change)| {
        let here = if at.is_empty() { key.clone() } else { format!("{at}.{key}") };
        match fields.get(key) {
            None => Some(here),
            Some(field) => unknown_key(field, change, &here),
        }
    })
}

/// One stage's parameters over the build volume.
pub struct Field<P> {
    base: P,
    json: Value,
    patches: Vec<(Within, Value)>,
    /// The parameters for each set of overrides that has held an item so far.
    resolved: RefCell<HashMap<Vec<usize>, Rc<P>>>,
}

impl<P: Clone + Serialize + DeserializeOwned> Field<P> {
    /// The same parameters everywhere.
    pub fn uniform(base: P) -> Field<P> {
        Field { json: Value::Null, base, patches: Vec::new(), resolved: RefCell::default() }
    }

    /// `base` with the overrides of `tuning` that set the stage named `stage`.
    pub fn new(base: P, tuning: &Tuning, stage: &str) -> Result<Field<P>, TuneError> {
        tuning.check()?;
        let patches: Vec<(Within, Value)> = tuning
            .overrides
            .iter()
            .filter_map(|o| Some((o.within.clone(), o.set.get(stage)?.clone())))
            .collect();
        let json = serde_json::to_value(&base).map_err(|e| TuneError(e.to_string()))?;
        if let Some(key) = patches.iter().find_map(|(_, patch)| unknown_key(&json, patch, "")) {
            return Err(TuneError(format!("`{stage}` has no setting `{key}`")));
        }
        Ok(Field { base, json, patches, resolved: RefCell::default() })
    }

    /// The base parameters, which hold where no override does.
    pub fn base(&self) -> &P {
        &self.base
    }

    /// The parameters at `p`. Each new combination of overrides is merged once and passed to
    /// `check`, the stage's own validation.
    pub fn at<E: From<TuneError>>(
        &self,
        p: [f64; 3],
        check: impl Fn(&P) -> Result<(), E>,
    ) -> Result<Rc<P>, E> {
        let key: Vec<usize> =
            self.patches.iter().enumerate().filter(|(_, (w, _))| w.holds(p)).map(|(i, _)| i).collect();
        if let Some(found) = self.resolved.borrow().get(&key) {
            return Ok(found.clone());
        }
        let params = if key.is_empty() {
            self.base.clone()
        } else {
            let mut json = self.json.clone();
            for patch in key.iter().filter_map(|&i| self.patches.get(i)).map(|(_, v)| v) {
                merge_patch(&mut json, patch);
            }
            serde_json::from_value(json).map_err(|e| TuneError(format!("overrides {key:?}: {e}")))?
        };
        check(&params)?;
        let params = Rc::new(params);
        self.resolved.borrow_mut().insert(key, params.clone());
        Ok(params)
    }
}

/// The centre of the box around `points`, at height `z`; the origin when there are none.
pub fn centre<'a>(points: impl IntoIterator<Item = &'a [f64; 2]>, z: f64) -> [f64; 3] {
    let (lo, hi) = points.into_iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| {
        ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])])
    });
    if lo[0] > hi[0] { [0.0, 0.0, z] } else { [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, z] }
}

#[cfg(test)]
mod tests;
