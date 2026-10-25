//! The template language and the reading of per-variant settings.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;

fn scope(pairs: &[(&str, Value)]) -> Scope {
    pairs.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
}

fn list(items: &[f64]) -> Value {
    Value::List(items.iter().map(|v| Value::Num(*v)).collect())
}

#[test]
fn variables_expressions_and_branches() {
    let s = scope(&[
        ("t", list(&[220.0, 230.0])),
        ("i", Value::Num(1.0)),
        ("kind", Value::Str("PLA".into())),
        ("z", Value::Num(1.25)),
    ]);
    assert_eq!(expand("M104 S[i] T{t[i] - 10}\n", &s).unwrap(), "M104 S1 T220\n");
    assert_eq!(expand("G1 Z{z + 0.4} F{max(1, 2, 3)*100}", &s).unwrap(), "G1 Z1.65 F300");
    let branchy = "{if kind == \"PETG\"}a{elsif (kind == \"PLA\") && t[0] >= 220}b{else}c{endif}";
    assert_eq!(expand(branchy, &s).unwrap(), "b");
    assert_eq!(expand("{if 1 > 2}{if missing}x{endif}{else}y{endif}", &s).unwrap(), "y");
    assert_eq!(expand("{i == 1 ? \"one\" : missing}", &s).unwrap(), "one");
    assert_eq!(
        expand("G29.1 Z{-0.003} ; keep a[0] and [1 2]", &s).unwrap(),
        "G29.1 Z-0.003 ; keep a[0] and [1 2]"
    );
}

#[test]
fn precedence_is_arithmetic() {
    let s = Scope::new();
    assert_eq!(expand("{1 + 2 * 3 - 4 / 2}", &s).unwrap(), "5");
    assert_eq!(expand("{(1 + 2) * 3 % 4}", &s).unwrap(), "1");
    assert_eq!(expand("{!(1 < 2) || 2 <= 2 && 3 != 4}", &s).unwrap(), "1");
}

#[test]
fn missing_or_broken_input_is_an_error() {
    let s = Scope::new();
    assert!(expand("M104 S{nozzle}", &s).unwrap_err().0.contains("nozzle"));
    assert!(expand("[nozzle]", &s).is_err());
    assert!(expand("{if 1}no end", &s).is_err());
    assert!(expand("{endif}", &s).is_err());
    assert!(expand("{1 / 0}", &s).is_err());
    assert!(expand(&format!("{{{}}}", "(".repeat(500)), &s).is_err());
    assert!(expand(&format!("{{{}1}}", "!".repeat(5000)), &s).is_err());
}

#[test]
fn settings_follow_the_variant() {
    let machine = Machine {
        source: String::new(),
        variant: "Bowden Standard".into(),
        machine: [
            (
                "printer_extruder_variant".to_string(),
                Value::List(vec![
                    Value::Str("Direct Drive Standard".into()),
                    Value::Str("Bowden Standard".into()),
                ]),
            ),
            ("printer_extruder_id".to_string(), list(&[1.0, 2.0])),
            (
                "extruder_type".to_string(),
                Value::List(vec![Value::Str("DD".into()), Value::Str("B".into()), Value::Str("C".into())]),
            ),
            ("retraction_length".to_string(), list(&[0.8, 2.0])),
            ("nozzle_diameter".to_string(), list(&[0.4, 0.6, 0.8])),
        ]
        .into_iter()
        .collect(),
        process: BTreeMap::new(),
        filament: [("density".to_string(), Value::Str("15%".into()))].into_iter().collect(),
    };
    assert_eq!(machine.number(Kind::Machine, "retraction_length").unwrap(), 2.0);
    assert_eq!(machine.number(Kind::Machine, "nozzle_diameter").unwrap(), 0.6);
    assert_eq!(machine.number(Kind::Filament, "density").unwrap(), 15.0);
    assert!(machine.number(Kind::Process, "absent").is_err());
}
