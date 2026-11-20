//! The template language: variables, expressions, branches and comments.
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
    // A list placed bare is the item of the extruder in use: the first, unless named.
    assert_eq!(expand("M204 S[t]", &s).unwrap(), "M204 S220");
    let second = scope(&[("t", list(&[220.0, 230.0])), ("current_extruder", Value::Num(1.0))]);
    assert_eq!(expand("M204 S[t]", &second).unwrap(), "M204 S230");
    let past = scope(&[("t", list(&[220.0])), ("current_extruder", Value::Num(3.0))]);
    assert!(expand("M204 S[t]", &past).is_err());
}

#[test]
fn comments_and_lines_of_branches_print_nothing() {
    let s = scope(&[("i", Value::Num(1.0))]);
    let template = "G1 X1\n{# a note, {braces} and all,\n   over two lines #}\n  {if i == 1}\nM1\n\t{else}\nM2\n{endif}\nM3 {# kept: M3 is on its line #}\n\n";
    assert_eq!(expand(template, &s).unwrap(), "G1 X1\nM1\nM3 \n\n");
    assert_eq!(expand("{if i}a{endif}\n{#x#}", &s).unwrap(), "a\n");
    assert!(expand("{# never closed }", &s).is_err());
}

#[test]
fn precedence_is_arithmetic() {
    let s = Scope::new();
    assert_eq!(expand("{1 + 2 * 3 - 4 / 2}", &s).unwrap(), "5");
    assert_eq!(expand("{(1 + 2) * 3 % 4}", &s).unwrap(), "1");
    assert_eq!(expand("G29.1 Z{+0.0} Z{-(-2)}", &s).unwrap(), "G29.1 Z0 Z2");
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
