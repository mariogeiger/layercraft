//! 3MF core specification: the root model, its objects (meshes or components) and build items.
//!
//! The root model is found through `_rels/.rels`; components may point into other model parts
//! of the package (production extension `p:path`). Transforms are 3×4 affine matrices in the
//! specification's row order `m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`.

use crate::{Body, Mesh, MeshError, Vertex};
use quick_xml::events::{BytesStart, Event};
use std::collections::HashMap;
use std::io::{Read, Seek, Write};

/// The largest model part read, in bytes.
const MAX_PART_BYTES: u64 = 1 << 30;
/// The largest number of vertices or triangles in one package.
const MAX_ELEMENTS: usize = 50_000_000;
/// The deepest component nesting followed.
const MAX_DEPTH: usize = 16;
const MODEL_REL: &str = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";

type Transform = [f64; 12];
const IDENTITY: Transform = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

#[derive(Default)]
struct Object {
    name: String,
    vertices: Vec<Vertex>,
    triangles: Vec<[u32; 3]>,
    components: Vec<(Option<String>, String, Transform)>,
}

#[derive(Default)]
struct Part {
    scale: f64,
    objects: HashMap<String, Object>,
    build: Vec<(String, Transform)>,
}

fn invalid(msg: impl Into<String>) -> MeshError {
    MeshError::Invalid(msg.into())
}

fn read_part<R: Read + Seek>(zip: &mut zip::ZipArchive<R>, path: &str) -> Result<String, MeshError> {
    let name = path.trim_start_matches('/');
    let file = zip.by_name(name).map_err(|e| MeshError::Zip(format!("{name}: {e}")))?;
    let mut text = String::new();
    file.take(MAX_PART_BYTES).read_to_string(&mut text)?;
    Ok(text)
}

fn root_model_path<R: Read + Seek>(zip: &mut zip::ZipArchive<R>) -> Result<String, MeshError> {
    let rels = read_part(zip, "_rels/.rels")?;
    let mut reader = quick_xml::Reader::from_str(&rels);
    loop {
        match reader.read_event().map_err(|e| MeshError::Xml(e.to_string()))? {
            Event::Empty(e) | Event::Start(e) if e.local_name().as_ref() == "Relationship" => {
                if attr(&e, "Type")?.as_deref() == Some(MODEL_REL) {
                    return attr(&e, "Target")?.ok_or_else(|| invalid("relationship without target"));
                }
            }
            Event::Eof => return Ok("3D/3dmodel.model".to_string()),
            _ => {}
        }
    }
}

fn attr(e: &BytesStart, key: &str) -> Result<Option<String>, MeshError> {
    for a in e.attributes() {
        let a = a.map_err(|e| MeshError::Xml(e.to_string()))?;
        if AsRef::<str>::as_ref(&a.key.local_name()) == key {
            let v = a
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .map_err(|e| MeshError::Xml(e.to_string()))?;
            return Ok(Some(v.into_owned()));
        }
    }
    Ok(None)
}

fn number(e: &BytesStart, key: &str) -> Result<f64, MeshError> {
    let text = attr(e, key)?.ok_or_else(|| invalid(format!("missing {key}")))?;
    let v: f64 = text.trim().parse().map_err(|_| invalid(format!("not a number: {text}")))?;
    if v.is_finite() { Ok(v) } else { Err(invalid(format!("not finite: {text}"))) }
}

fn index(e: &BytesStart, key: &str) -> Result<u32, MeshError> {
    let text = attr(e, key)?.ok_or_else(|| invalid("triangle without its three indices"))?;
    text.trim().parse().map_err(|_| invalid(format!("not an index: {text}")))
}

fn transform(e: &BytesStart) -> Result<Transform, MeshError> {
    let Some(text) = attr(e, "transform")? else { return Ok(IDENTITY) };
    let values: Vec<f64> = text.split_whitespace().filter_map(|s| s.parse().ok()).collect();
    match <[f64; 12]>::try_from(values) {
        Ok(m) if m.iter().all(|v| v.is_finite()) => Ok(m),
        _ => Err(invalid(format!("transform needs 12 finite numbers: {text}"))),
    }
}

fn unit_scale(unit: &str) -> Result<f64, MeshError> {
    Ok(match unit {
        "micron" => 0.001,
        "millimeter" => 1.0,
        "centimeter" => 10.0,
        "inch" => 25.4,
        "foot" => 304.8,
        "meter" => 1000.0,
        other => return Err(invalid(format!("unknown unit {other}"))),
    })
}

fn parse_part(text: &str, count: &mut usize) -> Result<Part, MeshError> {
    let mut part = Part { scale: 1.0, ..Part::default() };
    let mut reader = quick_xml::Reader::from_str(text);
    let mut current: Option<(String, Object)> = None;
    loop {
        let event = reader.read_event().map_err(|e| MeshError::Xml(e.to_string()))?;
        let (e, closes) = match &event {
            Event::Start(e) => (e, false),
            Event::Empty(e) => (e, true),
            Event::End(e) if e.local_name().as_ref() == "object" => {
                if let Some((id, object)) = current.take() {
                    part.objects.insert(id, object);
                }
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        match AsRef::<str>::as_ref(&e.local_name()) {
            "model" => part.scale = unit_scale(attr(e, "unit")?.as_deref().unwrap_or("millimeter"))?,
            "object" => {
                let id = attr(e, "id")?.ok_or_else(|| invalid("object without id"))?;
                let object = Object { name: attr(e, "name")?.unwrap_or_default(), ..Object::default() };
                if closes {
                    part.objects.insert(id, object);
                } else {
                    current = Some((id, object));
                }
            }
            tag @ ("vertex" | "triangle" | "component") => {
                let (_, object) = current.as_mut().ok_or_else(|| invalid("mesh data outside an object"))?;
                *count += 1;
                if *count > MAX_ELEMENTS {
                    return Err(invalid(format!("more than {MAX_ELEMENTS} elements")));
                }
                match tag {
                    "vertex" => object.vertices.push([number(e, "x")?, number(e, "y")?, number(e, "z")?]),
                    "triangle" => object.triangles.push([index(e, "v1")?, index(e, "v2")?, index(e, "v3")?]),
                    _ => {
                        let id = attr(e, "objectid")?.ok_or_else(|| invalid("component without objectid"))?;
                        object.components.push((attr(e, "path")?, id, transform(e)?));
                    }
                }
            }
            "item" => {
                let id = attr(e, "objectid")?.ok_or_else(|| invalid("build item without objectid"))?;
                part.build.push((id, transform(e)?));
            }
            _ => {}
        }
    }
    Ok(part)
}

fn compose(outer: &Transform, inner: &Transform) -> Transform {
    // Row vectors: p' = p · inner · outer.
    let m = |t: &Transform, r: usize, c: usize| t.get(r * 3 + c).copied().unwrap_or(0.0);
    let mut out = [0.0; 12];
    for r in 0..4 {
        for c in 0..3 {
            let mut v = if r == 3 { m(outer, 3, c) } else { 0.0 };
            for k in 0..3 {
                v += m(inner, r, k) * m(outer, k, c);
            }
            if let Some(slot) = out.get_mut(r * 3 + c) {
                *slot = v;
            }
        }
    }
    out
}

fn apply(t: &Transform, v: &Vertex, scale: f64) -> Vertex {
    let m = |i: usize| t.get(i).copied().unwrap_or(0.0);
    let [x, y, z] = *v;
    [
        (x * m(0) + y * m(3) + z * m(6) + m(9)) * scale,
        (x * m(1) + y * m(4) + z * m(7) + m(10)) * scale,
        (x * m(2) + y * m(5) + z * m(8) + m(11)) * scale,
    ]
}

struct Package<R: Read + Seek> {
    zip: zip::ZipArchive<R>,
    parts: HashMap<String, Part>,
    count: usize,
}

impl<R: Read + Seek> Package<R> {
    fn part(&mut self, path: &str) -> Result<&Part, MeshError> {
        if !self.parts.contains_key(path) {
            let text = read_part(&mut self.zip, path)?;
            let part = parse_part(&text, &mut self.count)?;
            self.parts.insert(path.to_string(), part);
        }
        self.parts.get(path).ok_or_else(|| invalid(format!("missing part {path}")))
    }

    fn collect(
        &mut self,
        path: &str,
        id: &str,
        t: Transform,
        depth: usize,
        into: &mut Body,
    ) -> Result<(), MeshError> {
        if depth > MAX_DEPTH {
            return Err(invalid("components nest too deep (a cycle?)"));
        }
        let part = self.part(path)?;
        let scale = part.scale;
        let object = part.objects.get(id).ok_or_else(|| invalid(format!("no object {id} in {path}")))?;
        if into.name.is_empty() {
            into.name = object.name.clone();
        }
        let base = into.vertices.len();
        let n = object.vertices.len();
        let offset = u32::try_from(base).map_err(|_| invalid("too many vertices"))?;
        for tri in &object.triangles {
            if tri.iter().any(|&i| i as usize >= n) {
                return Err(invalid(format!("object {id} has a triangle index out of range")));
            }
            let shifted = tri.map(|i| i.checked_add(offset));
            match shifted {
                [Some(a), Some(b), Some(c)] => into.triangles.push([a, b, c]),
                _ => return Err(invalid("too many vertices")),
            }
        }
        into.vertices.extend(object.vertices.iter().map(|v| apply(&t, v, scale)));
        let components = object.components.clone();
        for (sub_path, sub_id, sub_t) in components {
            let sub_path = sub_path.unwrap_or_else(|| path.to_string());
            self.collect(&sub_path, &sub_id, compose(&t, &sub_t), depth + 1, into)?;
        }
        Ok(())
    }
}

/// Reads every build item of a 3MF package as one body, in millimetres.
pub fn read_3mf<R: Read + Seek>(reader: R) -> Result<Mesh, MeshError> {
    let mut zip = zip::ZipArchive::new(reader).map_err(|e| MeshError::Zip(e.to_string()))?;
    let root = root_model_path(&mut zip)?;
    let mut package = Package { zip, parts: HashMap::new(), count: 0 };
    let build = package.part(&root)?.build.clone();
    let mut mesh = Mesh::default();
    for (id, t) in build {
        let mut body = Body { name: String::new(), vertices: Vec::new(), triangles: Vec::new() };
        package.collect(&root, &id, t, 0, &mut body)?;
        if body.name.is_empty() {
            body.name = format!("object {id}");
        }
        mesh.bodies.push(body);
    }
    Ok(mesh)
}

/// Writes a mesh as a minimal 3MF package: one object and one build item per body.
pub fn write_3mf<W: Write + Seek>(mesh: &Mesh, writer: W) -> Result<(), MeshError> {
    let zip_err = |e: zip::result::ZipError| MeshError::Zip(e.to_string());
    let mut zip = zip::ZipWriter::new(writer);
    let options = zip::write::SimpleFileOptions::default();
    zip.start_file("[Content_Types].xml", options).map_err(zip_err)?;
    zip.write_all(br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>"#)?;
    zip.start_file("_rels/.rels", options).map_err(zip_err)?;
    write!(
        zip,
        r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="{MODEL_REL}"/></Relationships>"#
    )?;
    zip.start_file("3D/3dmodel.model", options).map_err(zip_err)?;
    let mut xml = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\"><resources>",
    );
    for (i, body) in mesh.bodies.iter().enumerate() {
        let name = quick_xml::escape::escape(body.name.as_str());
        xml.push_str(&format!("<object id=\"{}\" type=\"model\" name=\"{name}\"><mesh><vertices>", i + 1));
        for v in &body.vertices {
            xml.push_str(&format!("<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", v[0], v[1], v[2]));
        }
        xml.push_str("</vertices><triangles>");
        for t in &body.triangles {
            xml.push_str(&format!("<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]));
        }
        xml.push_str("</triangles></mesh></object>");
    }
    xml.push_str("</resources><build>");
    for i in 0..mesh.bodies.len() {
        xml.push_str(&format!("<item objectid=\"{}\"/>", i + 1));
    }
    xml.push_str("</build></model>");
    zip.write_all(xml.as_bytes())?;
    zip.finish().map_err(zip_err)?;
    Ok(())
}
