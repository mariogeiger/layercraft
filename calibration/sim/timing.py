"""When each object of a plate is printed, layer by layer, from its G-code: for every layer the
time the nozzle first and last extrudes inside the object's box, and the part-cooling fan then.
Moves take a trapezoid of speed within the G-code's accelerations; a junction is crossed no faster
than the cosine of its turn times the slower feed, and arcs are taken as their chords."""
import json, math, re, sys, zipfile, io

def gcode_lines(path):
    """The lines of a .gcode, or of the plate in a .gcode.3mf."""
    if path.endswith(".3mf"):
        with zipfile.ZipFile(path) as z:
            return z.read("Metadata/plate_1.gcode").decode(errors="replace").splitlines(True)
    with open(path, errors="replace") as f:
        return f.readlines()

def moves(path):
    """The G-code's straight moves (arcs as their chords): start, end, feed (mm/s), acceleration
    (mm/s2), extruding or not, and the layer each falls in; fan per layer."""
    x = y = z = 0.0; F = 1.0; acc = 5000.0; fan = 0.0; layer = -1
    out = []; fans = {}
    for raw in gcode_lines(path):
        if raw.startswith("; CHANGE_LAYER"):
            layer += 1; continue
        s = raw.split(";")[0].strip()
        if not s:
            continue
        w = s.split()
        if w[0] == "M204":
            for p in w[1:]:
                if p.startswith("S"): acc = float(p[1:])
            continue
        if w[0] == "M106" and not any(p.startswith("P") for p in w[1:]):
            for p in w[1:]:
                if p.startswith("S"): fan = float(p[1:]) / 255.0
            fans[layer] = fan
            continue
        if w[0] not in ("G0", "G1", "G2", "G3"):
            continue
        a = {p[0]: float(p[1:]) for p in w[1:] if len(p) > 1 and p[0] in "XYZEFIJ"}
        if "F" in a:
            F = a["F"]
        nx, ny, nz = a.get("X", x), a.get("Y", y), a.get("Z", z)
        if (nx, ny, nz) != (x, y, z):
            out.append(((x, y, z), (nx, ny, nz), F / 60.0, acc, a.get("E", 0) > 0, layer))
        x, y, z = nx, ny, nz
    return out, fans

def plan(ms):
    """Times of the moves with trapezoidal speed: each junction no faster than cos(turn) of the
    slower feed, and every speed reachable from the last one within the acceleration."""
    n = len(ms)
    L = [math.dist(a, b) for a, b, *_ in ms]
    d = [tuple((b[i] - a[i]) / l for i in range(3)) if l > 0 else (0, 0, 0) for (a, b, *_), l in zip(ms, L)]
    vj = [0.0] * (n + 1)
    for i in range(1, n):
        c = sum(p * q for p, q in zip(d[i - 1], d[i]))
        vj[i] = min(ms[i - 1][2], ms[i][2]) * max(0.0, c)
    for i in range(n):                      # forward: reachable from the previous junction
        vj[i + 1] = min(vj[i + 1], math.sqrt(vj[i] ** 2 + 2 * ms[i][3] * L[i]))
    for i in range(n - 1, -1, -1):          # backward: able to slow to the next one
        vj[i] = min(vj[i], math.sqrt(vj[i + 1] ** 2 + 2 * ms[i][3] * L[i]))
    t = []
    for i in range(n):
        v, a, l, u, w = ms[i][2], ms[i][3], L[i], vj[i], vj[i + 1]
        vp = min(v, math.sqrt((2 * a * l + u * u + w * w) / 2))
        da, dd = (vp * vp - u * u) / (2 * a), (vp * vp - w * w) / (2 * a)
        t.append((vp - u) / a + (vp - w) / a + max(0.0, l - da - dd) / vp if vp > 0 else 0.0)
    return t

def timing(path, boxes, margin=1.5):
    ms, fans = moves(path)
    ts = plan(ms)
    out = {name: [] for name in boxes}
    t = 0.0; cur = {}; layer = None
    last = {"fan": 0.0}
    def close():
        last["fan"] = fans.get(layer, last["fan"])
        for name, (t0, t1, zz) in cur.items():
            out[name].append({"z": zz, "t0": t0, "t1": t1, "fan": last["fan"]})
    for (a, b, v, acc, extruding, lay), dt in zip(ms, ts):
        if lay != layer:
            close(); cur = {}; layer = lay
        t0 = t; t += dt
        if extruding and lay >= 0:
            for name, (x0, y0, x1, y1) in boxes.items():
                if x0 - margin <= b[0] <= x1 + margin and y0 - margin <= b[1] <= y1 + margin:
                    s0, s1, zz = cur.get(name, (t0, t, b[2]))
                    cur[name] = (s0, t, zz)
    close()
    print("total", round(t / 60, 1), "min")
    return out

if __name__ == "__main__":
    # timing.py PLATE_DIR GCODE: the boxes of PLATE_DIR/plate.json, in the model's frame, moved by
    # its bed_offset; writes PLATE_DIR/timing.json.
    plate_dir, gcode = sys.argv[1], sys.argv[2]
    p = json.load(open(f"{plate_dir}/plate.json"))
    ox, oy = p["bed_offset"]
    boxes = {k: (v[0] + ox, v[1] + oy, v[2] + ox, v[3] + oy) for k, v in p["boxes"].items()}
    out = timing(gcode, boxes)
    for k, v in out.items():
        dts = sorted(b["t0"] - a["t0"] for a, b in zip(v, v[1:]))
        print(f"{k:18s} layers {len(v):3d}  interval median {dts[len(dts) // 2] if dts else 0:6.1f} s")
    json.dump(out, open(f"{plate_dir}/timing.json", "w"))
