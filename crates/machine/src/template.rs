//! The macro language of the printer's G-code templates.
//!
//! Text is copied as is, except `[name]`, replaced by a variable's value, and `{…}` blocks: an
//! expression, printed, or `{if c}`, `{elsif c}`, `{else}`, `{endif}`. Expressions have
//! numbers, strings, variables, indexing `v[i]`, `ceil`/`floor`/`min`/`max`/`abs`/`int`, the
//! arithmetic, comparison and logical operators, and `c ? a : b`. Evaluation is lazy and
//! strict: a branch not taken is never evaluated, and a variable that is needed but not given
//! is an error naming it, never a silent empty value.

use crate::value::Value;
use std::collections::BTreeMap;

/// The variables a template can read.
pub type Scope = BTreeMap<String, Value>;

/// Why a template could not be expanded.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error("template: {0}")]
pub struct TemplateError(pub String);

fn err<T>(msg: impl Into<String>) -> Result<T, TemplateError> {
    Err(TemplateError(msg.into()))
}

#[derive(Debug)]
enum Piece {
    Text(String),
    Var(String),
    Expr(String),
    If(String),
    Elsif(String),
    Else,
    Endif,
}

fn is_ident(s: &str) -> bool {
    let mut chars = s.chars();
    chars.next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn split_text(text: &str, out: &mut Vec<Piece>) {
    let mut rest = text;
    while let Some(open) = rest.find('[') {
        let after = rest.get(open + 1..).unwrap_or("");
        match after.find(']') {
            Some(close) if is_ident(after.get(..close).unwrap_or("")) => {
                out.push(Piece::Text(rest.get(..open).unwrap_or("").to_string()));
                out.push(Piece::Var(after.get(..close).unwrap_or("").to_string()));
                rest = after.get(close + 1..).unwrap_or("");
            }
            _ => {
                out.push(Piece::Text(rest.get(..=open).unwrap_or("").to_string()));
                rest = after;
            }
        }
    }
    out.push(Piece::Text(rest.to_string()));
}

fn pieces(template: &str) -> Result<Vec<Piece>, TemplateError> {
    let mut out = Vec::new();
    let mut rest = template;
    while let Some(open) = rest.find('{') {
        split_text(rest.get(..open).unwrap_or(""), &mut out);
        let after = rest.get(open + 1..).unwrap_or("");
        let Some(close) = after.find('}') else { return err("a `{` is never closed") };
        let inner = after.get(..close).unwrap_or("").trim();
        let word = |w: &str| inner.strip_prefix(w).filter(|r| r.is_empty() || r.starts_with([' ', '(']));
        out.push(if let Some(c) = word("if") {
            Piece::If(c.to_string())
        } else if let Some(c) = word("elsif") {
            Piece::Elsif(c.to_string())
        } else if inner == "else" {
            Piece::Else
        } else if inner == "endif" {
            Piece::Endif
        } else {
            Piece::Expr(inner.to_string())
        });
        rest = after.get(close + 1..).unwrap_or("");
    }
    split_text(rest, &mut out);
    Ok(out)
}

/// Expands a template with the given variables.
pub fn expand(template: &str, scope: &Scope) -> Result<String, TemplateError> {
    let pieces = pieces(template)?;
    let mut out = String::new();
    // One frame per open `if`: (this branch is printing, a branch was already taken, parent prints).
    let mut frames: Vec<(bool, bool, bool)> = Vec::new();
    let printing = |frames: &[(bool, bool, bool)]| frames.last().is_none_or(|f| f.0);
    for piece in pieces {
        match piece {
            Piece::Text(t) if printing(&frames) => out.push_str(&t),
            Piece::Var(v) if printing(&frames) => match scope.get(&v) {
                Some(value) => out.push_str(&value.render()),
                None => return err(format!("variable `{v}` is not provided")),
            },
            Piece::Expr(e) if printing(&frames) => out.push_str(&crate::expr::eval(&e, scope)?.render()),
            Piece::If(c) => {
                let parent = printing(&frames);
                let take = parent && crate::expr::eval(&c, scope)?.truthy();
                frames.push((take, take, parent));
            }
            Piece::Elsif(c) => {
                let Some(frame) = frames.last_mut() else { return err("`elsif` without `if`") };
                let take = frame.2 && !frame.1 && crate::expr::eval(&c, scope)?.truthy();
                *frame = (take, frame.1 || take, frame.2);
            }
            Piece::Else => {
                let Some(frame) = frames.last_mut() else { return err("`else` without `if`") };
                *frame = (frame.2 && !frame.1, true, frame.2);
            }
            Piece::Endif if frames.pop().is_none() => return err("`endif` without `if`"),
            _ => {}
        }
    }
    if !frames.is_empty() {
        return err("an `if` is never closed");
    }
    Ok(out)
}
