"""Generate the second calibration plate, every quantity measurable with calipers, as a 3MF.

A: blocks 10 x 10 mm at heights 4, 12 and 20 mm and 3 x 3 mm at 20 mm: height against nominal
   tells a z offset (constant) from a z scale (proportional) from a swelling top (small ones).
B: bridges over a floor, spans 20 and 40 mm, 6 mm wide: the gap under the deck at mid-span (inner
   jaws) against 5.00 is the sag; the deck's thickness there (outer jaws) against 1.00.
C: plates 2 mm thick leaning by theta from vertical, 10 mm tall on a 1 mm pad: the thickness across
   their two parallel faces against 2.00 is how far the overhanging face droops; a fast and a slow row.
"""
import zipfile, sys, math

H = 0.2  # layer height
W = 0.42  # outer wall width

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
    for (a, b, c) in ear_clip(profile):
        tris.append((a, b, c)); tris.append((a+n, c+n, b+n))
    for i in range(n):
        j = (i+1) % n
        tris.append((i, i+n, j+n)); tris.append((i, j+n, j))
    return verts, tris

def box(x, y, z, dx, dy, dz):
    return prism([(0, z), (dx, z), (dx, z+dz), (0, z+dz)], x, y, dy)

bodies = []
# A: heights.
x = 0.0
for h in (4, 12, 20):
    bodies.append((f"block 10 h{h}", box(x, 0, 0, 10, 10, h))); x += 16
for h in (20,):
    bodies.append((f"block 3 h{h}", box(x, 3.5, 0, 3, 3, h))); x += 10
# B: bridges, one object of four touching boxes: floor, two posts, deck.
x = 0.0
for s in (20, 40):
    L = s + 8
    parts = [box(0, 0, 0, L, 6, 1), box(0, 0, 1, 4, 6, 5), box(4+s, 0, 1, 4, 6, 5), box(0, 0, 6, L, 6, 1)]
    for k, (v, t) in enumerate(parts):
        bodies.append((f"bridge S{s} part{k}", ([(a+x, b+30, c) for a, b, c in v], t)))
    x += L + 8
# C: leaning plates, fast row at y 50..60, slow row at y 75..85.
for row, y0 in (("fast", 50.0), ("slow", 75.0)):
    x = 0.0
    for theta in (45, 60):
        tn, w = math.tan(math.radians(theta)), 2.0 / math.cos(math.radians(theta))
        reach = w + 10*tn
        # A pad under the plate's whole reach holds its centre of mass over its footing.
        bodies.append((f"plate {theta} {row} pad", box(x, y0, 0, reach + 4, 10, 1)))
        prof = [(0, 1), (w, 1), (w + 10*tn, 11), (10*tn, 11)]
        bodies.append((f"plate {theta} {row}", prism(prof, x + 2, y0, 10.0)))
        x += reach + 4 + 6

objs, items = [], []
for k, (name, (v, t)) in enumerate(bodies, 1):
    vs = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in v)
    ts = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t)
    objs.append(f'<object id="{k}" type="model" name="{name}"><mesh><vertices>{vs}</vertices><triangles>{ts}</triangles></mesh></object>')
    items.append(f'<item objectid="{k}"/>')
model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
         f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
with zipfile.ZipFile(sys.argv[1], "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    z.writestr("3D/3dmodel.model", model)
print(len(bodies), "bodies")
for theta in (45, 60):
    print(f"plate {theta}: overhang share per layer {H*math.tan(math.radians(theta))/W:.2f}")
