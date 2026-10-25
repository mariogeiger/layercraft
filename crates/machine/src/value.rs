//! Values of settings and template variables: numbers, strings, booleans and lists.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Value {
    Bool(bool),
    Num(f64),
    Str(String),
    List(Vec<Value>),
}

impl Value {
    /// A setting as the printer's profiles write it: a number when it reads as one.
    pub fn from_setting(text: &str) -> Value {
        match text.trim().parse::<f64>() {
            Ok(v) if v.is_finite() => Value::Num(v),
            _ => Value::Str(text.to_string()),
        }
    }

    /// A setting from a profile's JSON: a string, or a list of strings.
    pub fn from_json(json: &serde_json::Value) -> Value {
        match json {
            serde_json::Value::String(s) => Value::from_setting(s),
            serde_json::Value::Array(items) => Value::List(items.iter().map(Value::from_json).collect()),
            serde_json::Value::Bool(b) => Value::Bool(*b),
            serde_json::Value::Number(n) => n.as_f64().map(Value::Num).unwrap_or(Value::Num(0.0)),
            other => Value::Str(other.to_string()),
        }
    }

    pub fn truthy(&self) -> bool {
        match self {
            Value::Bool(b) => *b,
            Value::Num(v) => *v != 0.0,
            Value::Str(s) => !s.is_empty() && s != "0" && s != "false",
            Value::List(l) => !l.is_empty(),
        }
    }

    pub fn number(&self) -> Option<f64> {
        match self {
            Value::Num(v) => Some(*v),
            Value::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
            Value::Str(s) => s.trim().trim_end_matches('%').parse().ok(),
            Value::List(_) => None,
        }
    }

    /// The value as printed into G-code: integers without a decimal point, other numbers
    /// with at most six decimals.
    pub fn render(&self) -> String {
        match self {
            Value::Num(v) => render_number(*v),
            Value::Bool(b) => if *b { "1" } else { "0" }.to_string(),
            Value::Str(s) => s.clone(),
            Value::List(l) => l.iter().map(Value::render).collect::<Vec<_>>().join(","),
        }
    }
}

pub fn render_number(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        let text = format!("{v:.6}");
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}
