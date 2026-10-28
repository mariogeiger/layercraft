"""Generate the third calibration plate, made to decide between the models of plate 2, as a 3MF.

A: two blocks 10 x 10 x 8, the second with its top surface at 0.8 of the flow: if the +0.10 mm of
   plate 2 comes from the last layer, it shrinks on the second; if from the first, it stays.
B: a tower 3 x 3 x 20, alone above 13 mm: its layers come every 1.4 s, against 2.6 s on plate 2,
   so a hot, soft top lifts it higher than plate 2's +0.28 mm.
C: plates leaning at 45 degrees on 1 mm pads, 2 mm thick, 5, 10 and 12 mm tall, one 3 mm thick and
   10 tall, and one at 30 degrees: an elastic ratchet rises as h^3 sin^2/(t^3 cos^3); the 5 mm one,
   barely, gives the top's own excess.
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
# A: blocks.
bodies.append(("block 10 h8", box(0, 0, 0, 10, 10, 8)))
bodies.append(("block 10 h8 top 0.8", box(20, 0, 0, 10, 10, 8)))
# B: tower.
bodies.append(("tower 3 h20", box(40, 3.5, 0, 3, 3, 20)))
# C: leaning plates on pads.
x = 0.0
for theta, t, h in ((45, 2, 5), (45, 2, 10), (45, 2, 12), (45, 3, 10), (30, 2, 10)):
    tn, w = math.tan(math.radians(theta)), t / math.cos(math.radians(theta))
    reach = w + h * tn
    bodies.append((f"plate {theta} t{t} h{h} pad", box(x, 20, 0, reach + 4, 10, 1)))
    prof = [(0, 1), (w, 1), (w + h * tn, 1 + h), (h * tn, 1 + h)]
    bodies.append((f"plate {theta} t{t} h{h}", prism(prof, x + 2, 20, 10.0)))
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
