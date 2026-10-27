//! Choosing a layer's beads by the objective (`layercraft_cost`), with an e-graph.
//!
//! Every way to lay a layer is a term: a piece of an area filled one way, two parts laid each
//! their best way. Rewrite rules (`fill.egg`) say which terms are equal, that is, lay the same
//! area: an e-graph holds all of them at once, sharing what they share, and equality saturation
//! (egglog) grows it until no rule adds anything. Extraction then takes, from each class of
//! equal terms, the one of least cost, the costs of its parts combined as the objective says:
//! the worst surface, the summed rest. Each alternative is costed only when extraction reaches
//! it, by the caller, who knows its geometry.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod quality;

use egglog::extract::TreeCostModel;
use egglog::{ArcSort, EGraph, Function, Value};
use layercraft_cost::Cost;
use std::cell::RefCell;
use std::collections::HashMap;

/// Why a choice failed: the e-graph refused its program, or found no way to lay a piece.
#[derive(Debug)]
pub struct ChooseError(pub String);

impl std::fmt::Display for ChooseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "choosing beads: {}", self.0)
    }
}

impl std::error::Error for ChooseError {}

/// Costs `Hatch p a` by the caller's `cost(p, a)`, once each; a piece unfilled cannot print.
struct Costs<'a, F: Fn(usize, usize) -> Cost> {
    cost: F,
    known: RefCell<HashMap<(i64, i64), Cost>>,
    egraph: &'a EGraph,
}

impl<F: Fn(usize, usize) -> Cost> TreeCostModel<Cost> for Costs<'_, F> {
    type EnodeCost = Cost;
    type ContainerCost = Cost;

    fn base_value_cost(&self, _: &EGraph, _: &ArcSort, _: Value) -> Cost {
        Cost::default()
    }

    fn enode_cost(&self, _: &EGraph, func: &Function, enode: &egglog::Enode<'_>) -> Cost {
        match func.name() {
            "Hatch" => {
                let arg = |i: usize| enode.children.get(i).map(|&v| self.egraph.value_to_base::<i64>(v));
                let (Some(p), Some(a)) = (arg(0), arg(1)) else { return Cost::INFINITE };
                *self.known.borrow_mut().entry((p, a)).or_insert_with(|| {
                    match (usize::try_from(p), usize::try_from(a)) {
                        (Ok(p), Ok(a)) => (self.cost)(p, a),
                        _ => Cost::INFINITE,
                    }
                })
            }
            "Both" => Cost::default(),
            _ => Cost::INFINITE,
        }
    }

    fn container_cost(&self, _: &EGraph, _: &ArcSort, _: Value) -> Cost {
        Cost::default()
    }

    fn fold_enode_cost(&self, own: Cost, children: &[Cost]) -> Cost {
        children.iter().fold(own, |c, d| c.and(*d))
    }

    fn fold_container_cost(&self, own: Cost, elements: &[Cost]) -> Cost {
        elements.iter().fold(own, |c, d| c.and(*d))
    }
}

/// The best way to fill each of `candidates.len()` pieces, piece `p` having `candidates[p]`
/// ways, costed by `cost(p, a)`: the chosen way of each piece, and the cost of them all.
pub fn choose_fills(
    candidates: &[usize],
    cost: impl Fn(usize, usize) -> Cost,
) -> Result<(Vec<usize>, Cost), ChooseError> {
    if candidates.is_empty() {
        return Ok((Vec::new(), Cost::default()));
    }
    let mut program = String::from(include_str!("fill.egg"));
    // The layer: its pieces side by side, a balanced tree so no term grows deep.
    fn tree(lo: usize, hi: usize) -> String {
        if hi - lo == 1 {
            format!("(Piece {lo})")
        } else {
            let mid = (lo + hi) / 2;
            format!("(Both {} {})", tree(lo, mid), tree(mid, hi))
        }
    }
    program.push_str(&format!("(let root {})\n", tree(0, candidates.len())));
    for (p, &n) in candidates.iter().enumerate() {
        for a in 0..n {
            program.push_str(&format!("(candidate {p} {a})\n"));
        }
    }
    program.push_str("(run-schedule (saturate (run)))\n");
    let mut egraph = EGraph::default();
    egraph.parse_and_run_program(None, &program).map_err(|e| ChooseError(e.to_string()))?;
    let root = egraph.parser.get_expr_from_string(None, "root").map_err(|e| ChooseError(e.to_string()))?;
    let (sort, value) = egraph.eval_expr(&root).map_err(|e| ChooseError(e.to_string()))?;
    let model = Costs { cost, known: RefCell::default(), egraph: &egraph };
    let extracted = egraph
        .extract_best_with_cost_model(vec![(sort, value)], model)
        .map_err(|e| ChooseError(e.to_string()))?;
    let Some(Some(best)) = extracted.terms.first() else {
        return Err(ChooseError("no way to fill every piece".into()));
    };
    let mut chosen = vec![usize::MAX; candidates.len()];
    let mut stack = vec![best.term];
    while let Some(id) = stack.pop() {
        if let egglog::Term::App(name, children) = extracted.termdag.get(id) {
            match name.as_str() {
                "Hatch" => {
                    let int = |i: usize| match children.get(i).map(|&c| extracted.termdag.get(c)) {
                        Some(egglog::Term::Lit(egglog::ast::Literal::Int(v))) => usize::try_from(*v).ok(),
                        _ => None,
                    };
                    if let (Some(p), Some(a)) = (int(0), int(1))
                        && let Some(slot) = chosen.get_mut(p)
                    {
                        *slot = a;
                    }
                }
                _ => stack.extend(children.iter().copied()),
            }
        }
    }
    if chosen.contains(&usize::MAX) {
        return Err(ChooseError("a piece left unfilled".into()));
    }
    Ok((chosen, best.cost))
}

#[cfg(test)]
mod tests;
