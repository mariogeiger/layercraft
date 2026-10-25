//! Expressions of the template language: tokens, a precedence-climbing parser, lazy evaluation.

use crate::template::{Scope, TemplateError};
use crate::value::Value;

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Num(f64),
    Str(String),
    Ident(String),
    Op(&'static str),
}

/// The deepest nesting of parentheses, brackets and conditionals.
const MAX_DEPTH: usize = 64;
/// The longest expression, which bounds the depth of evaluation.
const MAX_TOKENS: usize = 2_000;

const OPS: [&str; 20] = [
    "==", "!=", "<=", ">=", "&&", "||", "<", ">", "+", "-", "*", "/", "%", "!", "?", ":", "(", ")", "[", "]",
];

fn fail<T>(msg: String) -> Result<T, TemplateError> {
    Err(TemplateError(msg))
}

fn tokens(text: &str) -> Result<Vec<Token>, TemplateError> {
    let mut out = Vec::new();
    let chars: Vec<char> = text.chars().collect();
    let mut i = 0;
    let at = |i: usize| chars.get(i).copied().unwrap_or('\0');
    while i < chars.len() {
        let c = at(i);
        if c.is_whitespace() {
            i += 1;
        } else if c.is_ascii_digit() || (c == '.' && at(i + 1).is_ascii_digit()) {
            let start = i;
            while at(i).is_ascii_digit() || at(i) == '.' {
                i += 1;
            }
            let s: String = chars.get(start..i).unwrap_or(&[]).iter().collect();
            out.push(Token::Num(s.parse().map_err(|_| TemplateError(format!("bad number {s}")))?));
        } else if c.is_ascii_alphabetic() || c == '_' {
            let start = i;
            while at(i).is_ascii_alphanumeric() || at(i) == '_' {
                i += 1;
            }
            out.push(Token::Ident(chars.get(start..i).unwrap_or(&[]).iter().collect()));
        } else if c == '"' {
            i += 1;
            let mut s = String::new();
            while i < chars.len() && at(i) != '"' {
                if at(i) == '\\' {
                    i += 1;
                }
                s.push(at(i));
                i += 1;
            }
            if at(i) != '"' {
                return fail(format!("unclosed string in `{text}`"));
            }
            i += 1;
            out.push(Token::Str(s));
        } else if c == ',' {
            out.push(Token::Op(","));
            i += 1;
        } else {
            let two: String = [c, at(i + 1)].iter().collect();
            let op = OPS
                .iter()
                .find(|op| op.len() == 2 && **op == two)
                .or_else(|| OPS.iter().find(|op| op.len() == 1 && op.starts_with(c)));
            match op {
                Some(op) => {
                    out.push(Token::Op(op));
                    i += op.len();
                }
                None => return fail(format!("unexpected `{c}` in `{text}`")),
            }
        }
    }
    Ok(out)
}

#[derive(Debug)]
enum Node {
    Lit(Value),
    Var(String),
    Index(Box<Node>, Box<Node>),
    Call(String, Vec<Node>),
    Unary(&'static str, Box<Node>),
    Binary(&'static str, Box<Node>, Box<Node>),
    Cond(Box<Node>, Box<Node>, Box<Node>),
}

struct Parser {
    tokens: Vec<Token>,
    pos: usize,
    depth: usize,
}

fn precedence(op: &str) -> Option<u8> {
    Some(match op {
        "||" => 1,
        "&&" => 2,
        "==" | "!=" => 3,
        "<" | ">" | "<=" | ">=" => 4,
        "+" | "-" => 5,
        "*" | "/" | "%" => 6,
        _ => return None,
    })
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.pos)
    }

    fn eat(&mut self, op: &str) -> bool {
        if self.peek()
            == Some(&Token::Op(match op {
                "," => ",",
                ")" => ")",
                "]" => "]",
                ":" => ":",
                "(" => "(",
                _ => return false,
            }))
        {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn expect(&mut self, op: &str) -> Result<(), TemplateError> {
        if self.eat(op) { Ok(()) } else { fail(format!("expected `{op}`")) }
    }

    fn expression(&mut self) -> Result<Node, TemplateError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return fail("expression nests too deep".into());
        }
        let cond = self.binary(0)?;
        let node = if self.peek() == Some(&Token::Op("?")) {
            self.pos += 1;
            let yes = self.expression()?;
            self.expect(":")?;
            let no = self.expression()?;
            Node::Cond(Box::new(cond), Box::new(yes), Box::new(no))
        } else {
            cond
        };
        self.depth -= 1;
        Ok(node)
    }

    fn binary(&mut self, min: u8) -> Result<Node, TemplateError> {
        let mut left = self.unary()?;
        while let Some(Token::Op(op)) = self.peek().cloned() {
            let Some(p) = precedence(op) else { break };
            if p < min {
                break;
            }
            self.pos += 1;
            let right = self.binary(p + 1)?;
            left = Node::Binary(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn unary(&mut self) -> Result<Node, TemplateError> {
        match self.peek().cloned() {
            Some(Token::Op(op @ ("!" | "-" | "+"))) => {
                self.pos += 1;
                Ok(Node::Unary(op, Box::new(self.unary()?)))
            }
            _ => self.postfix(),
        }
    }

    fn postfix(&mut self) -> Result<Node, TemplateError> {
        let mut node = self.primary()?;
        while self.peek() == Some(&Token::Op("[")) {
            self.pos += 1;
            let index = self.expression()?;
            self.expect("]")?;
            node = Node::Index(Box::new(node), Box::new(index));
        }
        Ok(node)
    }

    fn primary(&mut self) -> Result<Node, TemplateError> {
        let token = self.peek().cloned();
        self.pos += 1;
        match token {
            Some(Token::Num(v)) => Ok(Node::Lit(Value::Num(v))),
            Some(Token::Str(s)) => Ok(Node::Lit(Value::Str(s))),
            Some(Token::Ident(name)) if name == "true" || name == "false" => {
                Ok(Node::Lit(Value::Bool(name == "true")))
            }
            Some(Token::Ident(name)) if self.eat("(") => {
                let mut args = Vec::new();
                if !self.eat(")") {
                    loop {
                        args.push(self.expression()?);
                        if self.eat(")") {
                            break;
                        }
                        self.expect(",")?;
                    }
                }
                Ok(Node::Call(name, args))
            }
            Some(Token::Ident(name)) => Ok(Node::Var(name)),
            Some(Token::Op("(")) => {
                let inner = self.expression()?;
                self.expect(")")?;
                Ok(inner)
            }
            other => fail(format!("unexpected {other:?}")),
        }
    }
}

fn number(v: &Value, what: &str) -> Result<f64, TemplateError> {
    v.number().ok_or_else(|| TemplateError(format!("{what} is not a number: {v:?}")))
}

fn evaluate(node: &Node, scope: &Scope) -> Result<Value, TemplateError> {
    Ok(match node {
        Node::Lit(v) => v.clone(),
        Node::Var(name) => match scope.get(name) {
            Some(v) => v.clone(),
            None => return fail(format!("variable `{name}` is not provided")),
        },
        Node::Index(list, index) => {
            let list = evaluate(list, scope)?;
            let i = number(&evaluate(index, scope)?, "an index")?;
            match list {
                Value::List(items) if i >= 0.0 => match items.get(i as usize) {
                    Some(v) => v.clone(),
                    None => return fail(format!("index {i} past the end of a list of {}", items.len())),
                },
                // A scalar setting stands for every index, as for single-extruder values.
                Value::List(_) => return fail(format!("negative index {i}")),
                scalar => scalar,
            }
        }
        Node::Call(name, args) => {
            let values = args
                .iter()
                .map(|a| evaluate(a, scope).and_then(|v| number(&v, name)))
                .collect::<Result<Vec<f64>, _>>()?;
            let one =
                || values.first().copied().ok_or_else(|| TemplateError(format!("{name} needs an argument")));
            Value::Num(match name.as_str() {
                "ceil" => one()?.ceil(),
                "floor" => one()?.floor(),
                "int" => one()?.trunc(),
                "abs" => one()?.abs(),
                "min" => values.iter().copied().fold(f64::INFINITY, f64::min),
                "max" => values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
                _ => return fail(format!("unknown function `{name}`")),
            })
        }
        Node::Unary("!", inner) => Value::Bool(!evaluate(inner, scope)?.truthy()),
        Node::Unary("+", inner) => Value::Num(number(&evaluate(inner, scope)?, "an operand")?),
        Node::Unary(_, inner) => Value::Num(-number(&evaluate(inner, scope)?, "an operand")?),
        Node::Cond(c, yes, no) => {
            if evaluate(c, scope)?.truthy() {
                evaluate(yes, scope)?
            } else {
                evaluate(no, scope)?
            }
        }
        Node::Binary("&&", a, b) => Value::Bool(evaluate(a, scope)?.truthy() && evaluate(b, scope)?.truthy()),
        Node::Binary("||", a, b) => Value::Bool(evaluate(a, scope)?.truthy() || evaluate(b, scope)?.truthy()),
        Node::Binary(op, a, b) => {
            let (a, b) = (evaluate(a, scope)?, evaluate(b, scope)?);
            match (*op, a.number(), b.number()) {
                ("==", Some(x), Some(y)) => Value::Bool(x == y),
                ("!=", Some(x), Some(y)) => Value::Bool(x != y),
                ("==", _, _) => Value::Bool(a.render() == b.render()),
                ("!=", _, _) => Value::Bool(a.render() != b.render()),
                (_, Some(x), Some(y)) => match *op {
                    "<" => Value::Bool(x < y),
                    ">" => Value::Bool(x > y),
                    "<=" => Value::Bool(x <= y),
                    ">=" => Value::Bool(x >= y),
                    "+" => Value::Num(x + y),
                    "-" => Value::Num(x - y),
                    "*" => Value::Num(x * y),
                    "/" if y != 0.0 => Value::Num(x / y),
                    "%" if y != 0.0 => Value::Num(x % y),
                    _ => return fail(format!("cannot compute {x} {op} {y}")),
                },
                _ => return fail(format!("`{op}` needs numbers, got {a:?} and {b:?}")),
            }
        }
    })
}

/// Parses and evaluates one expression.
pub fn eval(text: &str, scope: &Scope) -> Result<Value, TemplateError> {
    let tokens = tokens(text)?;
    if tokens.len() > MAX_TOKENS {
        return fail(format!("an expression of {} tokens is longer than {MAX_TOKENS}", tokens.len()));
    }
    let mut parser = Parser { tokens, pos: 0, depth: 0 };
    let node = parser.expression()?;
    if parser.pos != parser.tokens.len() {
        return fail(format!("unexpected text after the expression in `{text}`"));
    }
    evaluate(&node, scope)
}
