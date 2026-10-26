//! Boxes, merge patches and resolution at a place.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use serde_json::json;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Params {
    speed: f64,
    inner: Inner,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Inner {
    radius: f64,
    on: bool,
}

fn base() -> Params {
    Params { speed: 100.0, inner: Inner { radius: 1.2, on: true } }
}

fn tuning(text: &str) -> Tuning {
    serde_json::from_str(text).unwrap()
}

fn ok(_: &Params) -> Result<(), TuneError> {
    Ok(())
}

#[test]
fn boxes_are_half_open_and_unbounded_where_unsaid() {
    let w = Within { z: Some([0.0, 5.0]), ..Within::default() };
    assert!(w.holds([1e9, -1e9, 0.0]));
    assert!(w.holds([0.0, 0.0, 4.999]));
    assert!(!w.holds([0.0, 0.0, 5.0]));
    assert!(!w.holds([0.0, 0.0, f64::NAN]));
    assert!(Within::default().holds([1.0, 2.0, 3.0]));
}

#[test]
fn merge_patch_follows_rfc_7386() {
    let mut target = json!({"a": 1, "b": {"c": 2, "d": 3}, "e": [1, 2]});
    merge_patch(&mut target, &json!({"a": null, "b": {"c": 5}, "e": [3], "f": {"g": 1}}));
    assert_eq!(target, json!({"b": {"c": 5, "d": 3}, "e": [3], "f": {"g": 1}}));
}

#[test]
fn an_item_takes_every_override_that_holds_it_in_order() {
    let t = tuning(
        r#"[{"within": {"z": [0, 5]}, "set": {"motion": {"inner": {"radius": 0.6}}}},
            {"within": {"x": [10, 20]}, "set": {"motion": {"speed": 50, "inner": {"radius": 2.4}}}},
            {"set": {"path": {"other": 1}}}]"#,
    );
    let field = Field::new(base(), &t, "motion").unwrap();
    assert_eq!(*field.at([0.0, 0.0, 10.0], ok).unwrap(), base());
    let low = field.at([0.0, 0.0, 1.0], ok).unwrap();
    assert_eq!((low.speed, low.inner.radius, low.inner.on), (100.0, 0.6, true));
    // Both hold: the later one wins where they overlap.
    let both = field.at([15.0, 0.0, 1.0], ok).unwrap();
    assert_eq!((both.speed, both.inner.radius), (50.0, 2.4));
    // Resolved once per combination of overrides.
    assert!(Rc::ptr_eq(&low, &field.at([3.0, 3.0, 2.0], ok).unwrap()));
}

#[test]
fn a_bad_override_is_an_error_not_a_crash() {
    let wrong_type = tuning(r#"[{"set": {"motion": {"speed": "fast"}}}]"#);
    let field = Field::new(base(), &wrong_type, "motion").unwrap();
    assert!(field.at([0.0; 3], ok).is_err());
    let field = Field::new(base(), &tuning(r#"[{"set": {"motion": {"speed": -1}}}]"#), "motion").unwrap();
    let positive = |p: &Params| if p.speed > 0.0 { Ok(()) } else { Err(TuneError("speed".into())) };
    assert!(field.at([0.0; 3], positive).is_err());
    assert!(serde_json::from_str::<Tuning>(r#"[{"within": {"w": [0, 1]}, "set": {}}]"#).is_err());
    assert!(tuning(r#"[{"set": 3}]"#).check().is_err());
    // A misspelt setting or stage is an error, never silently nothing.
    let typo = tuning(r#"[{"set": {"motion": {"inner": {"raduis": 1}}}}]"#);
    let err = Field::new(base(), &typo, "motion").err().unwrap().to_string();
    assert!(err.contains("inner.raduis"), "{err}");
    assert!(tuning(r#"[{"set": {"moton": {}}}]"#).check_stages(&["motion", "path"]).is_err());
    assert!(tuning(r#"[{"set": {"motion": {}}}]"#).check_stages(&["motion", "path"]).is_ok());
}

#[test]
fn the_centre_is_of_the_bounding_box() {
    assert_eq!(centre(&[[0.0, 0.0], [10.0, 2.0], [4.0, 8.0]], 0.2), [5.0, 4.0, 0.2]);
    assert_eq!(centre(&[], 1.0), [0.0, 0.0, 1.0]);
}
