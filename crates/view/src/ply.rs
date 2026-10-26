//! The attributed complex a stage is drawn as, written as binary PLY.
//!
//! A complex is points in space and the segments and triangles between them, each element
//! carrying named numbers. A point's values vary along the segments and across the triangles
//! that meet there (a time); a segment's or a triangle's are constant on it (a layer, a role,
//! a width). PLY says exactly that: `vertex`, `edge` and `face` elements, each with its own
//! properties. Positions and measures are written as `double`, labels and counts as `int`,
//! little-endian. A segment with `width` and `height` stands for a bead of that cross-section
//! centred on it; without them, for a line.

use std::io::Write;

/// How a property is written: a label or a count, or a measure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Int,
    Float,
}

/// A named number every element of one kind carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Property {
    pub name: &'static str,
    pub kind: Kind,
}

/// A label or a count.
pub const fn int(name: &'static str) -> Property {
    Property { name, kind: Kind::Int }
}

/// A measure.
pub const fn float(name: &'static str) -> Property {
    Property { name, kind: Kind::Float }
}

/// Elements of one kind, `C` saying where each is, with `A` values each.
#[derive(Debug, Clone, PartialEq)]
pub struct Table<C, const A: usize> {
    pub properties: [Property; A],
    pub rows: Vec<(C, [f64; A])>,
}

/// Points with `V` values each, segments with `E`, triangles with `F`.
#[derive(Debug, Clone, PartialEq)]
pub struct Complex<const V: usize, const E: usize, const F: usize> {
    /// What the labels mean, one line each.
    pub comments: Vec<String>,
    pub points: Table<[f64; 3], V>,
    pub segments: Table<[u32; 2], E>,
    pub triangles: Table<[u32; 3], F>,
}

/// The most points a complex holds: PLY's `int` indices reach them all.
pub const MAX_POINTS: usize = i32::MAX as usize;

/// Why a complex could not be built.
#[derive(Debug)]
pub enum ViewError {
    /// More than `MAX_POINTS` points.
    TooLarge,
    /// The stage refers to something it does not hold.
    Invalid(String),
    Geometry(layercraft_geom::GeomError),
    Io(std::io::Error),
}

impl std::fmt::Display for ViewError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ViewError::TooLarge => write!(f, "the view holds more than {MAX_POINTS} points"),
            ViewError::Invalid(e) => write!(f, "invalid stage: {e}"),
            ViewError::Geometry(e) => write!(f, "view geometry: {e}"),
            ViewError::Io(e) => write!(f, "cannot write the view: {e}"),
        }
    }
}

impl std::error::Error for ViewError {}

impl From<std::io::Error> for ViewError {
    fn from(e: std::io::Error) -> Self {
        ViewError::Io(e)
    }
}

impl<const V: usize, const E: usize, const F: usize> Complex<V, E, F> {
    pub fn new(points: [Property; V], segments: [Property; E], triangles: [Property; F]) -> Self {
        Complex {
            comments: Vec::new(),
            points: Table { properties: points, rows: Vec::new() },
            segments: Table { properties: segments, rows: Vec::new() },
            triangles: Table { properties: triangles, rows: Vec::new() },
        }
    }

    /// Adds a point and returns its index.
    pub fn point(&mut self, at: [f64; 3], values: [f64; V]) -> Result<u32, ViewError> {
        let index = self.points.rows.len();
        if index >= MAX_POINTS {
            return Err(ViewError::TooLarge);
        }
        self.points.rows.push((at, values));
        Ok(index as u32)
    }

    pub fn segment(&mut self, ends: [u32; 2], values: [f64; E]) {
        self.segments.rows.push((ends, values));
    }

    pub fn triangle(&mut self, corners: [u32; 3], values: [f64; F]) {
        self.triangles.rows.push((corners, values));
    }

    /// The complex as a binary little-endian PLY file.
    pub fn write(&self, mut w: impl Write) -> Result<(), ViewError> {
        let mut header = String::from("ply\nformat binary_little_endian 1.0\n");
        for comment in &self.comments {
            // A line break would end the comment and start a header line of its own.
            header += &format!("comment {}\n", comment.replace(['\n', '\r'], " "));
        }
        header += &format!("element vertex {}\n", self.points.rows.len());
        header += "property double x\nproperty double y\nproperty double z\n";
        header += &properties(&self.points.properties);
        header += &format!("element edge {}\n", self.segments.rows.len());
        header += "property int vertex1\nproperty int vertex2\n";
        header += &properties(&self.segments.properties);
        header += &format!("element face {}\n", self.triangles.rows.len());
        header += "property list uchar int vertex_indices\n";
        header += &properties(&self.triangles.properties);
        header += "end_header\n";
        w.write_all(header.as_bytes())?;
        for (at, values) in &self.points.rows {
            for c in at {
                w.write_all(&c.to_le_bytes())?;
            }
            write_values(&mut w, &self.points.properties, values)?;
        }
        for (ends, values) in &self.segments.rows {
            for i in ends {
                w.write_all(&(*i as i32).to_le_bytes())?;
            }
            write_values(&mut w, &self.segments.properties, values)?;
        }
        for (corners, values) in &self.triangles.rows {
            w.write_all(&[3u8])?;
            for i in corners {
                w.write_all(&(*i as i32).to_le_bytes())?;
            }
            write_values(&mut w, &self.triangles.properties, values)?;
        }
        Ok(w.flush()?)
    }
}

fn properties(list: &[Property]) -> String {
    list.iter()
        .map(|p| match p.kind {
            Kind::Int => format!("property int {}\n", p.name),
            Kind::Float => format!("property double {}\n", p.name),
        })
        .collect()
}

fn write_values(w: &mut impl Write, properties: &[Property], values: &[f64]) -> std::io::Result<()> {
    for (p, v) in properties.iter().zip(values) {
        match p.kind {
            // Labels and counts are whole numbers well inside an `int`; `as` saturates otherwise.
            Kind::Int => w.write_all(&(*v as i32).to_le_bytes())?,
            Kind::Float => w.write_all(&v.to_le_bytes())?,
        }
    }
    Ok(())
}
