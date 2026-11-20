//! The macro language of a printer's own G-code templates: variables, expressions and branches
//! (`template`, `expr`) over values of settings and of the program (`value`), expanded by
//! `expand` with a `Scope`, the variables a template reads.

mod expr;
mod template;
mod value;

pub use template::{Scope, TemplateError, expand};
pub use value::{Value, render_number};

#[cfg(test)]
mod tests;
