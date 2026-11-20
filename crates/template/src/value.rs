//! Values of template variables: numbers, strings, booleans and lists.

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
    /// with six significant digits, as C's `%g` prints them.
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
        return format!("{}", v as i64);
    }
    let digits_before = if v == 0.0 || !v.is_finite() { 1 } else { v.abs().log10().floor() as i64 + 1 };
    let decimals = (6 - digits_before).clamp(0, 12) as usize;
    let text = format!("{v:.decimals$}");
    let text = if text.contains('.') { text.trim_end_matches('0').trim_end_matches('.') } else { &text };
    if text == "-0" { "0".to_string() } else { text.to_string() }
}
