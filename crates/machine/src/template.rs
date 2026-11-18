//! The macro language of the printer's G-code templates.
//!
//! Text is copied as is, except `[name]`, replaced by a variable's value — of a list, the item of
//! the extruder in use, `current_extruder` (the first when not given) — and `{…}` blocks: an
//! expression, printed, or `{if c}`, `{elsif c}`, `{else}`, `{endif}`, and `{# … #}`, a comment
//! for the template's reader that prints nothing. A line holding nothing but such branches and
//! comments, and blanks, leaves nothing, not even its line break, so a template's structure and
//! notes stay out of the G-code. Expressions have numbers, strings, variables, indexing `v[i]`,
//! `ceil`/`floor`/`min`/`max`/`abs`/`int`, the arithmetic, comparison and logical operators, and
//! `c ? a : b`. Evaluation is lazy and strict: a branch not taken is never evaluated, and a
//! variable that is needed but not given is an error naming it, never a silent empty value.

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
    Comment,
}

impl Piece {
    /// Whether the piece prints nothing whatever the variables: a branch's frame or a comment.
    fn silent(&self) -> bool {
        matches!(self, Piece::If(_) | Piece::Elsif(_) | Piece::Else | Piece::Endif | Piece::Comment)
    }
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
        if let Some(note) = after.strip_prefix('#') {
            let Some(close) = note.find("#}") else { return err("a `{#` is never closed") };
            out.push(Piece::Comment);
            rest = note.get(close + 2..).unwrap_or("");
            continue;
        }
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
    Ok(silent_lines_dropped(out))
}

/// The pieces without the lines that hold silent pieces and nothing else but blanks, each
/// dropped with its line break; the silent pieces stay, to frame the branches.
fn silent_lines_dropped(pieces: Vec<Piece>) -> Vec<Piece> {
    // Text cut after each line break, so each line ends with its own piece.
    let mut cut = Vec::with_capacity(pieces.len());
    for piece in pieces {
        match piece {
            Piece::Text(t) => cut.extend(t.split_inclusive('\n').map(|l| Piece::Text(l.to_string()))),
            other => cut.push(other),
        }
    }
    fn flush(line: &mut Vec<Piece>, out: &mut Vec<Piece>) {
        let blank = |p: &Piece| match p {
            Piece::Text(t) => t.chars().all(|c| matches!(c, ' ' | '\t' | '\r' | '\n')),
            other => other.silent(),
        };
        if line.iter().any(Piece::silent) && line.iter().all(blank) {
            out.extend(line.drain(..).filter(Piece::silent));
        } else {
            out.append(line);
        }
    }
    let mut out = Vec::with_capacity(cut.len());
    let mut line: Vec<Piece> = Vec::new();
    for piece in cut {
        let ends = matches!(&piece, Piece::Text(t) if t.ends_with('\n'));
        line.push(piece);
        if ends {
            flush(&mut line, &mut out);
        }
    }
    flush(&mut line, &mut out);
    out
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
                Some(Value::List(items)) => {
                    let extruder = scope.get("current_extruder").and_then(Value::number).unwrap_or(0.0);
                    match items.get(extruder.max(0.0) as usize) {
                        Some(item) => out.push_str(&item.render()),
                        None => return err(format!("`{v}` has no item for extruder {extruder}")),
                    }
                }
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
