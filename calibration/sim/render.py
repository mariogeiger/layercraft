"""Videos of flex.py's runs: the part at true scale and its deviation from the design magnified,
coloured by von Mises stress, with the nozzle while it pushes, and the top's excess layer by layer."""
import math, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter
from flex import simulate

F = 2.35  # N, with E = 1 GPa: the 60-degree fast plate's measured excess, 0.64 mm
cases = {
    "tour": dict(title="Tour 3×3 mm, 20 mm", width=3.0, shift=0.0, depth=3.0, layers=100, mag=100),
    "plaque60": dict(title="Plaque penchée 60°, 2 mm, 10 mm", width=2 / math.cos(math.radians(60)),
                     shift=0.2 * math.tan(math.radians(60)), depth=10.0, layers=50, per_shift=4, mag=1),
}
name = sys.argv[1]
compare, ghost = [], []
if name == "plaque60":
    compare = [(10, 0.64, "mesuré 60° rapide"), (10, 0.50, "mesuré 60° lent"), (10, 0.04, "mesuré 45° rapide"), (10, 0.10, "45° lent")]
    g = simulate(F=F, width=2 / math.cos(math.radians(45)), shift=0.2, depth=10.0, layers=50, per_shift=2)
    ghost = [(np.arange(1, 51) * 0.2, g["excess"], "simulé 45°")]
if name == "tour":
    compare = [(20, 0.18, "mesuré 3×3 (moins 0,10)")]
c = dict(cases[name]); title = c.pop("title"); mag = c.pop("mag")
r = simulate(F=F, **c)
dx, n, h = r["dx"], r["n"], 0.2
k = c.get("per_shift", 0) if c["shift"] > 0 else 0
L = c["layers"]

def design(quads_count):
    """The design's quads in birth order: layer by layer, column by column."""
    out = []
    for j in range(1, L + 1):
        o = (j - 1) * k
        for cc in range(o, o + n):
            out.append(np.array([[cc * dx, (j - 1) * h], [(cc + 1) * dx, (j - 1) * h], [(cc + 1) * dx, j * h], [cc * dx, j * h]]))
    return out[:quads_count]

views = [(1, "à l'échelle réelle")] + ([(mag, f"écart au dessin × {mag}")] if mag != 1 else [])
if len(views) == 2:
    fig = plt.figure(figsize=(12, 6.75), dpi=110)
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.2])
    geo = [fig.add_subplot(gs[0]), fig.add_subplot(gs[1])]; a3 = fig.add_subplot(gs[2])
else:
    fig = plt.figure(figsize=(12, 6.75), dpi=110)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.6, 1])
    geo = [fig.add_subplot(gs[0])]; a3 = fig.add_subplot(gs[1])
xmax = (L - 1) * k * dx + n * dx
zmax = L * h
for a, (_, label) in zip(geo, views):
    a.set_xlim(-1, xmax + 2); a.set_ylim(-0.5, zmax + 3); a.set_aspect("equal"); a.axis("off"); a.set_title(label, fontsize=11)
fig.suptitle(f"{title} — la buse pousse avec F = {F} N (E = 1 GPa)", fontsize=13)
a3.set_xlim(0, zmax); a3.set_ylim(0, 0.75)
a3.set_xlabel("hauteur imprimée (mm)"); a3.set_ylabel("sommet au-dessus de la buse, buse partie (mm)")
for zm, v, lab in compare:
    a3.plot([zm], [v], "o", color="#555", ms=6); a3.annotate(lab, (zm, v), xytext=(-8, 6), textcoords="offset points", ha="right", fontsize=8, color="#555")
for zs, vs, lab in ghost:
    a3.plot(zs, vs, color="#aaa", lw=1.5, ls="--"); a3.text(zs[-1], vs[-1] + 0.02, lab, ha="right", fontsize=8, color="#888")
line, = a3.plot([], [], color="#c0392b", lw=2)
a3.grid(alpha=0.3)
cmap = plt.get_cmap("viridis")
smax = float(np.percentile(np.concatenate([f["stress"] for f in r["frames"][-2:]]), 98))
artists = []
status = fig.text(0.02, 0.03, "", fontsize=11)
value = a3.text(0.04 * zmax, 0.70, "", fontsize=13, color="#c0392b")

writer = FFMpegWriter(fps=12, bitrate=2400)
with writer.saving(fig, f"{name}.mp4", dpi=110):
    for f in r["frames"]:
        for a in artists: a.remove()
        artists = []
        quads = [np.array(q) for q in f["quads"]]
        des = design(len(quads))
        col = [cmap(min(v / smax, 1.0)) for v in f["stress"]]
        for a, (m_, _) in zip(geo, views):
            qs = [d + m_ * (q - d) for q, d in zip(quads, des)]
            pc = PolyCollection(qs, facecolors=col, edgecolors="none"); a.add_collection(pc); artists.append(pc)
            artists += a.plot([-1, xmax + 2], [0, 0], color="k", lw=2)
            zn = f["layer"] * h
            artists += a.plot([-1, xmax + 2], [zn, zn], color="#2471a3", ls="--", lw=0.8)
            if f["nozzle"]:
                x0, x1, z = f["nozzle"]; xm = (x0 + x1) / 2
                noz = plt.Polygon([(xm - 1.2, z + 2.4), (xm + 1.2, z + 2.4), (xm + 0.25, z), (xm - 0.25, z)], fc="#999", ec="k")
                a.add_patch(noz); artists.append(noz)
        phase = "la buse pousse, la couche est posée" if f["phase"] == "deposit" else "la buse est partie : la pièce se redresse"
        status.set_text(f"couche {f['layer']} sur {L} : {phase}.   Couleur : contrainte de von Mises.   Pointillé bleu : hauteur de la buse.")
        m = f["layer"]
        line.set_data(np.arange(1, m + 1) * h, r["excess"][:m])
        value.set_text(f"simulé : {r['excess'][m-1]:.3f} mm")
        writer.grab_frame()
        if f["phase"] == "release":
            writer.grab_frame()
    for _ in range(24):
        writer.grab_frame()
print(name, "final excess", round(r["excess"][-1], 4))
