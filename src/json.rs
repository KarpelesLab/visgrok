//! A small JSON reader and writer (enough for visgrok's sidecar files and
//! the web UI protocol; no dependencies).

/// A parsed JSON value.
#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    /// `null`.
    Null,
    /// `true` / `false`.
    Bool(bool),
    /// A number.
    Num(f64),
    /// A string.
    Str(String),
    /// An array.
    Arr(Vec<Json>),
    /// An object, in source order.
    Obj(Vec<(String, Json)>),
}

impl Json {
    /// Parses a complete JSON document.
    pub fn parse(s: &str) -> Option<Json> {
        let mut p = Parser { b: s.as_bytes(), i: 0 };
        let v = p.value()?;
        p.ws();
        (p.i == p.b.len()).then_some(v)
    }

    /// Member `k` of an object.
    pub fn get(&self, k: &str) -> Option<&Json> {
        match self {
            Json::Obj(o) => o.iter().find(|(n, _)| n == k).map(|(_, v)| v),
            _ => None,
        }
    }

    /// The string, if this is one.
    pub fn str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    /// The number, if this is one.
    pub fn num(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }

    /// The boolean, if this is one.
    pub fn bool(&self) -> Option<bool> {
        match self {
            Json::Bool(b) => Some(*b),
            _ => None,
        }
    }
}

struct Parser<'a> {
    b: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn ws(&mut self) {
        while self.i < self.b.len() && self.b[self.i].is_ascii_whitespace() {
            self.i += 1;
        }
    }

    fn eat(&mut self, c: u8) -> Option<()> {
        self.ws();
        (self.b.get(self.i) == Some(&c)).then(|| self.i += 1)
    }

    fn value(&mut self) -> Option<Json> {
        self.ws();
        match *self.b.get(self.i)? {
            b'{' => {
                self.i += 1;
                let mut o = Vec::new();
                if self.eat(b'}').is_some() {
                    return Some(Json::Obj(o));
                }
                loop {
                    self.ws();
                    let Json::Str(k) = self.string()? else { return None };
                    self.eat(b':')?;
                    o.push((k, self.value()?));
                    if self.eat(b',').is_none() {
                        self.eat(b'}')?;
                        return Some(Json::Obj(o));
                    }
                }
            }
            b'[' => {
                self.i += 1;
                let mut a = Vec::new();
                if self.eat(b']').is_some() {
                    return Some(Json::Arr(a));
                }
                loop {
                    a.push(self.value()?);
                    if self.eat(b',').is_none() {
                        self.eat(b']')?;
                        return Some(Json::Arr(a));
                    }
                }
            }
            b'"' => self.string(),
            b't' if self.b[self.i..].starts_with(b"true") => {
                self.i += 4;
                Some(Json::Bool(true))
            }
            b'f' if self.b[self.i..].starts_with(b"false") => {
                self.i += 5;
                Some(Json::Bool(false))
            }
            b'n' if self.b[self.i..].starts_with(b"null") => {
                self.i += 4;
                Some(Json::Null)
            }
            _ => {
                let s = self.i;
                while self.i < self.b.len() && matches!(self.b[self.i], b'0'..=b'9' | b'-' | b'+' | b'.' | b'e' | b'E') {
                    self.i += 1;
                }
                std::str::from_utf8(&self.b[s..self.i]).ok()?.parse().ok().map(Json::Num)
            }
        }
    }

    fn string(&mut self) -> Option<Json> {
        if self.b.get(self.i) != Some(&b'"') {
            return None;
        }
        self.i += 1;
        let mut out = String::new();
        loop {
            let c = *self.b.get(self.i)?;
            self.i += 1;
            match c {
                b'"' => return Some(Json::Str(out)),
                b'\\' => {
                    let e = *self.b.get(self.i)?;
                    self.i += 1;
                    match e {
                        b'n' => out.push('\n'),
                        b't' => out.push('\t'),
                        b'r' => out.push('\r'),
                        b'b' => out.push('\u{8}'),
                        b'f' => out.push('\u{c}'),
                        b'u' => {
                            let h = std::str::from_utf8(self.b.get(self.i..self.i + 4)?).ok()?;
                            self.i += 4;
                            out.push(char::from_u32(u32::from_str_radix(h, 16).ok()?).unwrap_or('\u{fffd}'));
                        }
                        c => out.push(c as char),
                    }
                }
                _ => {
                    // Copy a run of plain UTF-8 bytes.
                    let s = self.i - 1;
                    while self.i < self.b.len() && self.b[self.i] != b'"' && self.b[self.i] != b'\\' {
                        self.i += 1;
                    }
                    out.push_str(std::str::from_utf8(&self.b[s..self.i]).ok()?);
                }
            }
        }
    }
}

/// JSON string literal.
pub fn jstr(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

/// Builds a JSON object, member by member.
#[derive(Default)]
pub struct Obj(Vec<String>);

impl Obj {
    #[allow(missing_docs)]
    pub fn new() -> Obj {
        Obj(Vec::new())
    }
    #[allow(missing_docs)]
    pub fn str(&mut self, k: &str, v: &str) {
        self.0.push(format!("{}:{}", jstr(k), jstr(v)));
    }
    #[allow(missing_docs)]
    pub fn num(&mut self, k: &str, v: f64) {
        self.0
            .push(format!("{}:{}", jstr(k), if v.is_finite() { v.to_string() } else { "null".into() }));
    }
    #[allow(missing_docs)]
    pub fn bool(&mut self, k: &str, v: bool) {
        self.0.push(format!("{}:{v}", jstr(k)));
    }
    #[allow(missing_docs)]
    pub fn raw(&mut self, k: &str, v: &str) {
        self.0.push(format!("{}:{v}", jstr(k)));
    }
    #[allow(missing_docs)]
    pub fn finish(self) -> String {
        format!("{{{}}}", self.0.join(","))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let j = Json::parse(r#"{"cmd":"view","start":12,"x":[1,"a\"b",true,null],"u":"été ok"}"#).unwrap();
        assert_eq!(j.get("cmd").and_then(Json::str), Some("view"));
        assert_eq!(j.get("start").and_then(Json::num), Some(12.0));
        assert_eq!(j.get("u").and_then(Json::str), Some("été ok"));
        assert_eq!(jstr("a\"b\n"), r#""a\"b\n""#);
    }
}
