//! A small JSON reader and writer (RFC 8259), enough for SigMF metadata and strict about it.
//!
//! Objects keep their members in file order (a metadata file read and written back keeps its
//! layout), and numbers keep their text (a 64-bit sample index or a frequency written as an
//! integer is not pushed through a double).

/// A JSON value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// `null`.
    Null,
    /// `true` or `false`.
    Bool(bool),
    /// A number, as its text in the grammar of RFC 8259 §6.
    Number(String),
    /// A string, unescaped.
    String(String),
    /// An array.
    Array(Vec<Value>),
    /// An object: members in order; duplicate names are kept as read.
    Object(Vec<(String, Value)>),
}

/// What is wrong with a JSON text, and at which byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JsonError {
    /// Byte offset of the problem in the text.
    pub offset: usize,
    /// What was expected or found there.
    pub what: &'static str,
}

impl std::fmt::Display for JsonError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "JSON: {} at byte {}", self.what, self.offset)
    }
}

impl std::error::Error for JsonError {}

/// Arrays and objects nested deeper than this are refused (a hostile file must not exhaust the
/// stack).
pub const MAX_DEPTH: usize = 64;

impl Value {
    /// A number from an unsigned integer.
    pub fn from_u64(v: u64) -> Value {
        Value::Number(v.to_string())
    }

    /// A number from a double; `None` for NaN and the infinities, which JSON cannot write.
    pub fn from_f64(v: f64) -> Option<Value> {
        // Rust prints the shortest decimal that reads back as the same double, without an
        // exponent: always inside the JSON grammar.
        v.is_finite().then(|| Value::Number(format!("{v}")))
    }

    /// The value as a double, if it is a number.
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Number(t) => t.parse().ok(),
            _ => None,
        }
    }

    /// The value as an unsigned integer, if it is a number with an exact integer value in range
    /// (`20000000`, `2e7` and `20000000.0` all are).
    pub fn as_u64(&self) -> Option<u64> {
        let Value::Number(t) = self else {
            return None;
        };
        if let Ok(v) = t.parse::<u64>() {
            return Some(v);
        }
        let v: f64 = t.parse().ok()?;
        // Doubles hold every integer up to 2^53 exactly; beyond that only the plain integer
        // form above is trusted.
        ((0.0..=9_007_199_254_740_992.0).contains(&v) && v.fract() == 0.0).then_some(v as u64)
    }

    /// The value as a string slice, if it is a string.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::String(s) => Some(s),
            _ => None,
        }
    }

    /// The first member named `name`, if the value is an object that has one.
    pub fn get(&self, name: &str) -> Option<&Value> {
        match self {
            Value::Object(members) => members.iter().find(|(k, _)| k == name).map(|(_, v)| v),
            _ => None,
        }
    }

    /// The text of the value on one line, without spaces.
    pub fn to_compact(&self) -> String {
        let mut out = String::new();
        write_value(self, None, 0, &mut out);
        out
    }

    /// The text of the value indented by four spaces per level, with a final newline.
    pub fn to_pretty(&self) -> String {
        let mut out = String::new();
        write_value(self, Some(4), 0, &mut out);
        out.push('\n');
        out
    }
}

fn write_string(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            // RFC 8259 §7: the control characters must be escaped; everything else may be raw.
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

fn write_value(v: &Value, indent: Option<usize>, level: usize, out: &mut String) {
    let newline = |out: &mut String, level: usize| {
        if let Some(n) = indent {
            out.push('\n');
            out.push_str(&" ".repeat(n * level));
        }
    };
    match v {
        Value::Null => out.push_str("null"),
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Number(t) => out.push_str(t),
        Value::String(s) => write_string(s, out),
        Value::Array(items) => {
            out.push('[');
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                newline(out, level + 1);
                write_value(item, indent, level + 1, out);
            }
            if !items.is_empty() {
                newline(out, level);
            }
            out.push(']');
        }
        Value::Object(members) => {
            out.push('{');
            for (i, (name, item)) in members.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                newline(out, level + 1);
                write_string(name, out);
                out.push(':');
                if indent.is_some() {
                    out.push(' ');
                }
                write_value(item, indent, level + 1, out);
            }
            if !members.is_empty() {
                newline(out, level);
            }
            out.push('}');
        }
    }
}

struct Parser<'a> {
    text: &'a [u8],
    at: usize,
}

impl Parser<'_> {
    fn error<T>(&self, what: &'static str) -> Result<T, JsonError> {
        Err(JsonError {
            offset: self.at,
            what,
        })
    }

    fn peek(&self) -> Option<u8> {
        self.text.get(self.at).copied()
    }

    fn skip_space(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\n' | b'\r')) {
            self.at += 1;
        }
    }

    fn literal(&mut self, word: &'static [u8], value: Value) -> Result<Value, JsonError> {
        if self.text[self.at..].starts_with(word) {
            self.at += word.len();
            Ok(value)
        } else {
            self.error("unknown literal")
        }
    }

    fn digits(&mut self) -> usize {
        let start = self.at;
        while matches!(self.peek(), Some(b'0'..=b'9')) {
            self.at += 1;
        }
        self.at - start
    }

    fn number(&mut self) -> Result<Value, JsonError> {
        // RFC 8259 §6: [ minus ] int [ frac ] [ exp ]; no leading zeros, no bare point.
        let start = self.at;
        if self.peek() == Some(b'-') {
            self.at += 1;
        }
        match self.peek() {
            Some(b'0') => self.at += 1,
            Some(b'1'..=b'9') => {
                self.digits();
            }
            _ => return self.error("digit expected"),
        }
        if self.peek() == Some(b'.') {
            self.at += 1;
            if self.digits() == 0 {
                return self.error("digit expected after the point");
            }
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.at += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.at += 1;
            }
            if self.digits() == 0 {
                return self.error("digit expected in the exponent");
            }
        }
        match std::str::from_utf8(&self.text[start..self.at]) {
            Ok(t) => Ok(Value::Number(t.to_string())),
            Err(_) => self.error("number is not text"),
        }
    }

    fn hex4(&mut self) -> Result<u32, JsonError> {
        let mut v = 0u32;
        for _ in 0..4 {
            let d = match self.peek() {
                Some(c @ b'0'..=b'9') => u32::from(c - b'0'),
                Some(c @ b'a'..=b'f') => u32::from(c - b'a') + 10,
                Some(c @ b'A'..=b'F') => u32::from(c - b'A') + 10,
                _ => return self.error("four hexadecimal digits expected"),
            };
            v = v * 16 + d;
            self.at += 1;
        }
        Ok(v)
    }

    fn string(&mut self) -> Result<String, JsonError> {
        // The opening quote is at `self.at`.
        self.at += 1;
        let mut out: Vec<u8> = Vec::new();
        loop {
            let Some(c) = self.peek() else {
                return self.error("unterminated string");
            };
            self.at += 1;
            match c {
                b'"' => break,
                b'\\' => {
                    let Some(e) = self.peek() else {
                        return self.error("unterminated escape");
                    };
                    self.at += 1;
                    let ch = match e {
                        b'"' => '"',
                        b'\\' => '\\',
                        b'/' => '/',
                        b'b' => '\u{8}',
                        b'f' => '\u{c}',
                        b'n' => '\n',
                        b'r' => '\r',
                        b't' => '\t',
                        b'u' => {
                            let first = self.hex4()?;
                            let code = if (0xd800..0xdc00).contains(&first) {
                                // A high surrogate must be followed by an escaped low one.
                                if self.peek() != Some(b'\\') {
                                    return self.error("low surrogate expected");
                                }
                                self.at += 1;
                                if self.peek() != Some(b'u') {
                                    return self.error("low surrogate expected");
                                }
                                self.at += 1;
                                let second = self.hex4()?;
                                if !(0xdc00..0xe000).contains(&second) {
                                    return self.error("low surrogate expected");
                                }
                                0x10000 + ((first - 0xd800) << 10) + (second - 0xdc00)
                            } else {
                                first
                            };
                            match char::from_u32(code) {
                                Some(ch) => ch,
                                None => return self.error("lone surrogate"),
                            }
                        }
                        _ => {
                            self.at -= 1;
                            return self.error("unknown escape");
                        }
                    };
                    let mut buf = [0u8; 4];
                    out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                }
                0..=0x1f => {
                    self.at -= 1;
                    return self.error("control character in a string");
                }
                c => out.push(c),
            }
        }
        // Raw bytes were copied from a `&str`, escapes were encoded as UTF-8: the result is
        // valid unless an escape split a multi-byte character, which the check catches.
        match String::from_utf8(out) {
            Ok(s) => Ok(s),
            Err(_) => self.error("string is not UTF-8"),
        }
    }

    fn value(&mut self, depth: usize) -> Result<Value, JsonError> {
        if depth > MAX_DEPTH {
            return self.error("nested too deeply");
        }
        self.skip_space();
        match self.peek() {
            None => self.error("value expected"),
            Some(b'n') => self.literal(b"null", Value::Null),
            Some(b't') => self.literal(b"true", Value::Bool(true)),
            Some(b'f') => self.literal(b"false", Value::Bool(false)),
            Some(b'"') => self.string().map(Value::String),
            Some(b'-' | b'0'..=b'9') => self.number(),
            Some(b'[') => {
                self.at += 1;
                let mut items = Vec::new();
                self.skip_space();
                if self.peek() == Some(b']') {
                    self.at += 1;
                    return Ok(Value::Array(items));
                }
                loop {
                    items.push(self.value(depth + 1)?);
                    self.skip_space();
                    match self.peek() {
                        Some(b',') => self.at += 1,
                        Some(b']') => {
                            self.at += 1;
                            return Ok(Value::Array(items));
                        }
                        _ => return self.error("',' or ']' expected"),
                    }
                }
            }
            Some(b'{') => {
                self.at += 1;
                let mut members = Vec::new();
                self.skip_space();
                if self.peek() == Some(b'}') {
                    self.at += 1;
                    return Ok(Value::Object(members));
                }
                loop {
                    self.skip_space();
                    if self.peek() != Some(b'"') {
                        return self.error("member name expected");
                    }
                    let name = self.string()?;
                    self.skip_space();
                    if self.peek() != Some(b':') {
                        return self.error("':' expected");
                    }
                    self.at += 1;
                    members.push((name, self.value(depth + 1)?));
                    self.skip_space();
                    match self.peek() {
                        Some(b',') => self.at += 1,
                        Some(b'}') => {
                            self.at += 1;
                            return Ok(Value::Object(members));
                        }
                        _ => return self.error("',' or '}' expected"),
                    }
                }
            }
            Some(_) => self.error("value expected"),
        }
    }
}

/// Parses a JSON text: one value, with nothing but white space around it.
pub fn parse(text: &str) -> Result<Value, JsonError> {
    let mut p = Parser {
        text: text.as_bytes(),
        at: 0,
    };
    let v = p.value(0)?;
    p.skip_space();
    if p.at != p.text.len() {
        return p.error("text after the value");
    }
    Ok(v)
}
