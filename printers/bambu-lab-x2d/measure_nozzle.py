#!/usr/bin/env -S uv run --script
# /// script
# dependencies = ["numpy", "pillow", "scipy", "matplotlib"]
# ///
"""Measure the nozzle's silhouette in `nozzle-photo.png` and check `printer.json`'s nozzle holds it.

The photo is the front of the toolhead against a grey card. Per row, the silhouette spans the
pixels that leave the card's level (interpolated between its two sides) by more than `EDGE`, a
low threshold that takes the blur's outer edge; the scale is the column's measured diameter over
its width in the same pixels; the tip is where the middle of its end comes back to half contrast.
The column's top hides behind the front edge of the head in the photo: its height is the
caliper's. Writes `nozzle-silhouette.csv` (height above the tip, half width, in mm) and
`toolhead.png`, the photo with the silhouette and the model, and the model to scale.
"""
import json
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

EDGE = 15                           # grey levels off the card
CARD = [(82, 98), (235, 255)]       # columns of card on either side of the nozzle
SEARCH = (100, 215)                 # columns the nozzle lies within
COLUMN_ROWS = (97, 115)             # rows of the column, below the head's edge
MIDDLE = (155, 170)                 # columns down the middle of the tip

p = json.load(open('printer.json'))
n, margin = p['nozzle'], p['head_margin']
R, H, FACE = n['column_diameter'] / 2, n['protrusion'], n['face_diameter'] / 2
FLANK = np.radians(n['flank_degrees'])
KNEE = min((R - FACE) * np.tan(FLANK), H)

rgb = np.asarray(Image.open('nozzle-photo.png').convert('RGB'), dtype=float)
lum = gaussian_filter(rgb.mean(2), 0.8)
xs = np.arange(lum.shape[1])
rows = []
for y in range(lum.shape[0]):
    left, right = (lum[y, a:b].mean() for a, b in CARD)
    centre = [np.mean(c) for c in CARD]
    card = left + (right - left) * (xs - centre[0]) / (centre[1] - centre[0])
    off = (np.abs(lum[y] - card) > EDGE) & (xs > SEARCH[0]) & (xs < SEARCH[1])
    if off.any():
        rows.append((y, xs[off].min(), xs[off].max()))
rows = np.array(rows, float)
# The tip: down the middle from its darkest near the silhouette's last row, back to half contrast.
middle = lum[:, MIDDLE[0]:MIDDLE[1]].mean(1)
last = int(rows[:, 0].max())
start = last - 15 + int(np.argmin(middle[last - 15:last + 1]))
half_contrast = (middle[start] + middle[-5:].mean()) / 2
tip = start + int(np.argmax(middle[start:] > half_contrast))
column = (rows[:, 0] >= COLUMN_ROWS[0]) & (rows[:, 0] <= COLUMN_ROWS[1])
px = np.median(rows[column, 2] - rows[column, 1]) / n['column_diameter']
seen = (rows[:, 0] >= COLUMN_ROWS[0]) & (rows[:, 0] < tip)
z = (tip - rows[seen, 0]) / px
half = (rows[seen, 2] - rows[seen, 1]) / 2 / px
model = np.where(z < KNEE, FACE + z / np.tan(FLANK), R)
np.savetxt('nozzle-silhouette.csv', np.stack([z, half], 1), fmt='%.3f', delimiter=',',
           header='height above the tip (mm),half width (mm)', comments='')
print(f'{px:.1f} px/mm; seen {z.max():.2f} mm of {H} up; face about {2 * half[z < 0.1].mean():.2f} mm across')
# The silhouette's own noise, on the column the caliper measured: a pixel.
print(f'the model holds the silhouette within a pixel: {bool(np.all(model >= half - 1 / px))}; least room {np.min(model - half):+.3f} mm')

fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5.6), gridspec_kw=dict(width_ratios=[1.1, 1]))
a.imshow(rgb.astype(np.uint8))
a.plot(rows[seen, 1], rows[seen, 0], '.', color='tab:red', ms=3, label='measured edge')
a.plot(rows[seen, 2], rows[seen, 0], '.', color='tab:red', ms=3)
axis = (rows[column, 1].mean() + rows[column, 2].mean()) / 2
zz = np.linspace(0, H, 200)
rr = np.where(zz < KNEE, FACE + zz / np.tan(FLANK), R)
for sign in (-1, 1):
    a.plot(axis + sign * rr * px, tip - zz * px, '-', color='yellow', lw=1.5)
a.plot([axis - FACE * px, axis + FACE * px], [tip, tip], '-', color='yellow', lw=1.5, label='model')
a.set_xlim(SEARCH[0] - 30, SEARCH[1] + 30); a.set_ylim(tip + 40, COLUMN_ROWS[0] - 40)
a.set_xticks([]); a.set_yticks([]); a.legend(loc='lower left', fontsize=9)
a.set_title(f'The photo: the silhouette measured (1 pixel ≈ {1 / px:.3f} mm)', fontsize=11)
b.fill_between([-5, 5], H - margin, 4.4, facecolor='none', edgecolor='tab:blue', hatch='//', lw=0)
b.plot([-5, 5], [H, H], color='tab:blue', lw=2)
b.plot([-5, 5], [H - margin, H - margin], color='tab:blue', lw=1, ls='--')
b.text(0, 3.75, 'forbidden (head, other nozzle, gantry)', ha='center', color='tab:blue', fontsize=10,
       bbox=dict(facecolor='white', edgecolor='none', pad=1))
b.add_patch(Polygon([[-R, H], [-R, KNEE], [-FACE, 0], [FACE, 0], [R, KNEE], [R, H]], closed=True,
                    facecolor='0.85', edgecolor='k', lw=1.5))
b.plot(np.concatenate([-half[::-1], half]), np.concatenate([z[::-1], z]), '.', color='tab:red', ms=3)
b.fill_between([-5, 5], -0.8, 0, color='0.6')
def dim(x, z0, z1, text, side='right'):
    b.annotate('', xy=(x, z0), xytext=(x, z1), arrowprops=dict(arrowstyle='<->', color='tab:red'))
    b.text(x + (0.1 if side == 'right' else -0.1), (z0 + z1) / 2, text, color='tab:red', fontsize=10,
           ha='left' if side == 'right' else 'right', va='center')
dim(3.6, 0, H, f'{H} mm')
dim(-3.0, 0, KNEE, f'{KNEE:.2f} mm', side='left')
b.text(2.0, H - margin - 0.1, f'margin {margin} mm', color='tab:blue', fontsize=10, va='top')
b.annotate('', xy=(-R, 3.15), xytext=(R, 3.15), arrowprops=dict(arrowstyle='<->', color='tab:red'))
b.text(0, 3.2, f'Ø {2 * R} mm', ha='center', va='bottom', color='tab:red', fontsize=10,
       bbox=dict(facecolor='white', edgecolor='none', pad=0.5))
b.annotate(f'face Ø {2 * FACE} mm', xy=(0.3, 0.02), xytext=(0.9, -0.55), color='tab:red', fontsize=10,
           arrowprops=dict(arrowstyle='->', color='tab:red'))
b.text(-1.25, 0.45, f'flank {n["flank_degrees"]:g}°', color='k', fontsize=10, ha='right')
b.set_aspect('equal'); b.set_xlim(-5, 5); b.set_ylim(-0.8, 4.4); b.set_xlabel('mm'); b.grid(alpha=0.2)
b.set_title('The model, to scale (red: the silhouette measured)', fontsize=11)
fig.tight_layout(); fig.savefig('toolhead.png', dpi=150)
