"""The measured objects as column layouts for printsim, with their G-code timing and measurement."""
import json, math, os

def tower(width, layers, dx=0.2):
    n = max(1, round(width / dx)); return [(0, n)] * layers, dx

def leaning(theta, layers=50, pad_layers=5, k=None, t=2.0):
    shift = 0.2 * math.tan(math.radians(theta)); k = k or (2 if theta > 50 else 1)
    dx = shift / k; n = round(t / math.cos(math.radians(theta)) / dx)
    reach = t / math.cos(math.radians(theta)) + layers * 0.2 * math.tan(math.radians(theta))
    pad = round((reach + 4) / dx); c0 = round(2 / dx)
    return [(0, pad)] * pad_layers + [(c0 + i * k, c0 + i * k + n) for i in range(layers)], dx

# name: (layout, depth, plate timing file, measured top excess over the design, mm)
OBJECTS = {
    "bloc10_h4": (lambda: tower(10, 20), 10, "2", 0.09),
    "bloc10_h12": (lambda: tower(10, 60), 10, "2", 0.10),
    "bloc10_h20": (lambda: tower(10, 100), 10, "2", 0.11),
    "bloc3_h20": (lambda: tower(3, 100), 3, "2", 0.28),
    "pilier3": (lambda: tower(3, 100), 3, "1", 0.27),
    "pilier5": (lambda: tower(5, 100), 5, "1", 0.27),
    "pilier8": (lambda: tower(8, 100), 8, "1", 0.17),
    "plaque45_rapide": (lambda: leaning(45), 10, "2", 0.14),
    "plaque60_rapide": (lambda: leaning(60), 10, "2", 0.74),
    "plaque45_lente": (lambda: leaning(45), 10, "2", 0.20),
    "plaque60_lente": (lambda: leaning(60), 10, "2", 0.60),
    # Plate 3.
    "bloc10_h8": (lambda: tower(10, 40), 10, "3", 0.1),
    "tour3_h20": (lambda: tower(3, 100), 3, "3", 0.21),
    "plaque45_t2_h5": (lambda: leaning(45, layers=25), 10, "3", 0.22),
    "plaque45_t2_h10": (lambda: leaning(45), 10, "3", 0.22),
    "plaque45_t2_h12": (lambda: leaning(45, layers=60), 10, "3", 0.116),
    "plaque45_t3_h10": (lambda: leaning(45, t=3.0), 10, "3", 0.115),
    "plaque30_t2_h10": (lambda: leaning(30, k=1), 10, "3", 0.16),
}

HERE = os.path.dirname(os.path.abspath(__file__))

def schedule(name):
    """When the nozzle is on the object, layer by layer: its plate's timing.json (timing.py)."""
    plate = OBJECTS[name][2]
    with open(os.path.join(HERE, "..", f"plate-{plate}", "timing.json")) as f:
        return [(L["t0"], L["t1"]) for L in json.load(f)[name]]
