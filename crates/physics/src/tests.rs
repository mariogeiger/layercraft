//! The viscosity law through its anchors, and a column cooling as heat flows say it must.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use column::{Column, Layer};

#[test]
fn the_viscosity_runs_through_its_glass_and_its_melt() {
    let p = Physics::default();
    let law = viscosity::Viscosity::of(&p);
    assert!((law.at(p.glass_temperature) / 1e12 - 1.0).abs() < 1e-9);
    assert!((law.at(p.melt_temperature) / p.melt_viscosity - 1.0).abs() < 1e-9);
    // Stiffer as it cools, glass below its divergence.
    assert!(law.at(150.0) > law.at(200.0));
    assert_eq!(law.at(p.glass_temperature - 60.0), f64::INFINITY);
}

#[test]
fn a_layer_left_long_cools_to_the_air_and_the_bed() {
    // A 20 mm square on the bed: left a long time, it settles between the bed and the air.
    let p = Physics::default();
    let mut c = Column::new(p.clone());
    let layer = Layer { volume: 400.0 * 0.2, height: 0.2, perimeter: 80.0, fan: 1.0 };
    let soon = c.clone().lay(&layer, 0.5);
    let late = c.lay(&layer, 600.0);
    assert!(soon > late, "{soon} then {late}");
    assert!(late > p.air_temperature && late < p.bed_temperature + 1.0, "{late}");
}

#[test]
fn a_small_layer_laid_fast_on_a_tall_column_stays_soft() {
    // A 3 mm square pillar: at 1 s a layer, the top keeps hotter than at 10 s, and squeezes more.
    let p = Physics::default();
    let pillar = Layer { volume: 9.0 * 0.2, height: 0.2, perimeter: 12.0, fan: 1.0 };
    let top = |seconds: f64| {
        let mut c = Column::new(p.clone());
        (0..50).map(|_| c.lay(&pillar, seconds)).last().unwrap()
    };
    let (fast, slow) = (top(1.0), top(10.0));
    assert!(fast > slow, "{fast} vs {slow}");
    assert!(p.squeeze(0.2, fast) > p.squeeze(0.2, slow));
}
