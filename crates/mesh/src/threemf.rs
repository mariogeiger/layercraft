//! 3MF core specification: the root model, its objects (meshes or components) and build items.
//!
//! The root model is found through `_rels/.rels`; components may point into other model parts
//! of the package (production extension `p:path`). Transforms are 3×4 affine matrices in the
//! specification's row order `m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`. An object's
//! colour is the property its `pid` and `pindex` name: a base material's `displaycolor` or a
//! colour group's `color` (materials extension); a component without one takes its parent's.
//! Each build item becomes one body per colour, and each colour one material.

use crate::{Body, Material, Mesh, MeshError, Vertex};
use quick_xml::events::{BytesStart, Event};
use std::collections::HashMap;
use std::io::{Read, Seek};

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
    /// The property group and index its `pid` and `pindex` name.
    property: Option<(String, usize)>,
}

#[derive(Default)]
struct Part {
    scale: f64,
    objects: HashMap<String, Object>,
    build: Vec<(String, Transform)>,
    /// Each property group's colours, `#RRGGBB`, by group id.
    colours: HashMap<String, Vec<String>>,
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

/// A 3MF colour, `#RRGGBB` or `#RRGGBBAA`, as `#RRGGBB` in capitals; the alpha is dropped.
fn colour(text: &str) -> Result<String, MeshError> {
    let hex = text.trim().strip_prefix('#').unwrap_or_default();
    if !(hex.len() == 6 || hex.len() == 8) || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(invalid(format!("not a colour: {text}")));
    }
    Ok(format!("#{}", hex.get(..6).unwrap_or_default().to_ascii_uppercase()))
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
    // The property group being read: base materials or a colour group.
    let mut group: Option<String> = None;
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
            Event::End(e) if matches!(e.local_name().as_ref(), "basematerials" | "colorgroup") => {
                group = None;
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        match AsRef::<str>::as_ref(&e.local_name()) {
            "model" => part.scale = unit_scale(attr(e, "unit")?.as_deref().unwrap_or("millimeter"))?,
            "basematerials" | "colorgroup" if !closes => {
                group = Some(attr(e, "id")?.ok_or_else(|| invalid("property group without id"))?);
            }
            tag @ ("base" | "color") => {
                let key = if tag == "base" { "displaycolor" } else { "color" };
                let (Some(id), Some(text)) = (&group, attr(e, key)?) else { continue };
                *count += 1;
                if *count > MAX_ELEMENTS {
                    return Err(invalid(format!("more than {MAX_ELEMENTS} elements")));
                }
                part.colours.entry(id.clone()).or_default().push(colour(&text)?);
            }
            "object" => {
                let id = attr(e, "id")?.ok_or_else(|| invalid("object without id"))?;
                let property = match (attr(e, "pid")?, attr(e, "pindex")?) {
                    (Some(pid), pindex) => {
                        let text = pindex.unwrap_or_else(|| "0".into());
                        let index =
                            text.trim().parse().map_err(|_| invalid(format!("not an index: {text}")))?;
                        Some((pid, index))
                    }
                    (None, _) => None,
                };
                let object =
                    Object { name: attr(e, "name")?.unwrap_or_default(), property, ..Object::default() };
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

    /// Adds object `id` of the part at `path`, placed by `t`, and its components to the body of
    /// its colour in `into`; `inherited` is its parent's colour.
    fn collect(
        &mut self,
        path: &str,
        id: &str,
        t: Transform,
        depth: usize,
        inherited: Option<&str>,
        bodies: &mut Vec<(Option<String>, Body)>,
    ) -> Result<(), MeshError> {
        if depth > MAX_DEPTH {
            return Err(invalid("components nest too deep (a cycle?)"));
        }
        let part = self.part(path)?;
        let scale = part.scale;
        let object = part.objects.get(id).ok_or_else(|| invalid(format!("no object {id} in {path}")))?;
        let own = match &object.property {
            Some((pid, index)) => match part.colours.get(pid) {
                Some(colours) => Some(
                    colours
                        .get(*index)
                        .ok_or_else(|| invalid(format!("object {id}: no property {index} in group {pid}")))?
                        .clone(),
                ),
                // A property group that holds no colour (a texture, a composite) leaves the parent's.
                None => None,
            },
            None => None,
        };
        let colour = own.or_else(|| inherited.map(str::to_string));
        let components = object.components.clone();
        if !object.triangles.is_empty() {
            let at = match bodies.iter().position(|(c, _)| *c == colour) {
                Some(at) => at,
                None => {
                    bodies.push((colour.clone(), Body::default()));
                    bodies.len() - 1
                }
            };
            let Some((_, into)) = bodies.get_mut(at) else { return Err(invalid("lost a body")) };
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
                match tri.map(|i| i.checked_add(offset)) {
                    [Some(a), Some(b), Some(c)] => into.triangles.push([a, b, c]),
                    _ => return Err(invalid("too many vertices")),
                }
            }
            into.vertices.extend(object.vertices.iter().map(|v| apply(&t, v, scale)));
        }
        for (sub_path, sub_id, sub_t) in components {
            let sub_path = sub_path.unwrap_or_else(|| path.to_string());
            self.collect(&sub_path, &sub_id, compose(&t, &sub_t), depth + 1, colour.as_deref(), bodies)?;
        }
        Ok(())
    }
}

/// Reads every build item of a 3MF package as one body per colour, in millimetres. With more
/// than one colour, each is a material, in the order the bodies first use them.
pub fn read_3mf<R: Read + Seek>(reader: R) -> Result<Mesh, MeshError> {
    let mut zip = zip::ZipArchive::new(reader).map_err(|e| MeshError::Zip(e.to_string()))?;
    let root = root_model_path(&mut zip)?;
    let mut package = Package { zip, parts: HashMap::new(), count: 0 };
    let build = package.part(&root)?.build.clone();
    let mut mesh = Mesh::default();
    let mut materials: Vec<Material> = Vec::new();
    for (id, t) in build {
        let mut bodies = Vec::new();
        package.collect(&root, &id, t, 0, None, &mut bodies)?;
        for (colour, mut body) in bodies {
            if body.name.is_empty() {
                body.name = format!("object {id}");
            }
            let material = Material { colour };
            body.material = match materials.iter().position(|m| *m == material) {
                Some(k) => k,
                None => {
                    materials.push(material);
                    materials.len() - 1
                }
            };
            mesh.bodies.push(body);
        }
    }
    if materials.len() > 1 {
        mesh.materials = materials;
    } else {
        mesh.bodies.iter_mut().for_each(|b| b.material = 0);
    }
    Ok(mesh)
}
