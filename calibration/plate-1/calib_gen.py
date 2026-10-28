"""Generate the overhang/bridge/pillar calibration plate as a 3MF of named prisms."""
import zipfile, json, sys

W = 0.42          # outer wall line width of the profile
H = 0.2           # layer height
SLOPE_LAYERS = 15 # layers of each ledge's slope (3 mm)

def ear_clip(poly):
    """Triangles of a simple counter-clockwise polygon, by ear clipping."""
    idx = list(range(len(poly)))
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    def inside(p, a, b, c):
        return cross(a, b, p) > 0 and cross(b, c, p) > 0 and cross(c, a, p) > 0
    tris = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i, j, l = idx[k-1], idx[k], idx[(k+1) % len(idx)]
            a, b, c = poly[i], poly[j], poly[l]
            if cross(a, b, c) <= 0: continue
            if any(inside(poly[m], a, b, c) for m in idx if m not in (i, j, l)): continue
            tris.append((i, j, l)); idx.pop(k); break
        else:
            raise ValueError("not simple")
    tris.append(tuple(idx))
    return tris

def prism(profile, x0, y0, depth):
    """A prism: `profile` (x, z), counter-clockwise, extruded along y from y0 by depth."""
    n = len(profile)
    verts = [(x0+x, y0, z) for x, z in profile] + [(x0+x, y0+depth, z) for x, z in profile]
    tris = []
    # Outward normals: the profile is ccw seen from -y, so the y0 cap faces -y.
    for (a, b, c) in ear_clip(profile):
        tris.append((a, b, c))          # cap at y0, facing -y
        tris.append((a+n, c+n, b+n))    # cap at y0+depth, facing +y
    for i in range(n):
        j = (i+1) % n
        tris.append((i, i+n, j+n)); tris.append((i, j+n, j))
    return verts, tris

bodies = []
# A: ledges, overhang share u per layer, fast row (y 0..8) and slow row (y 20..28).
us = [0.25, 0.375, 0.5, 0.625, 0.75, 1.0]
for row, y0 in (("fast", 0.0), ("slow", 20.0)):
    x = 0.0
    for u in us:
        e = SLOPE_LAYERS * u * W
        prof = [(0,0), (6,0), (6,4), (6+e, 4+SLOPE_LAYERS*H), (6+e, 8), (0, 8)]
        bodies.append((f"ledge u{u} {row}", prism(prof, x, y0, 8.0)))
        x += 6 + SLOPE_LAYERS * max(us) * W + 3
# B: bridges of span S between two 4 mm pillars 5 mm high, a 1 mm plate on top, 6 mm wide.
x = 0.0
for s in (5, 10, 20, 30):
    prof = [(0,0), (4,0), (4,5), (4+s,5), (4+s,0), (8+s,0), (8+s,6), (0,6)]
    bodies.append((f"bridge S{s}", prism(prof, x, 40.0, 6.0)))
    x += 8 + s + 4
# C: square pillars, 20 mm tall.
x = 0.0
for a in (3, 5, 8):
    prof = [(0,0), (a,0), (a,20), (0,20)]
    bodies.append((f"pillar {a}", prism(prof, x, 56.0, float(a))))
    x += a + 12

objs, items = [], []
for k, (name, (v, t)) in enumerate(bodies, 1):
    vs = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in v)
    ts = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t)
    objs.append(f'<object id="{k}" type="model" name="{name}"><mesh><vertices>{vs}</vertices><triangles>{ts}</triangles></mesh></object>')
    items.append(f'<item objectid="{k}"/>')
model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
         f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
out = sys.argv[1]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    z.writestr("3D/3dmodel.model", model)
print(len(bodies), "bodies")

def signed_volume(v, t):
    s = 0.0
    for a, b, c in t:
        (x1,y1,z1),(x2,y2,z2),(x3,y3,z3) = v[a], v[b], v[c]
        s += x1*(y2*z3-y3*z2) - x2*(y1*z3-y3*z1) + x3*(y1*z2-y2*z1)
    return s / 6.0
for name, (v, t) in bodies:
    print(name, round(signed_volume(v, t), 3))
