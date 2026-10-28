"""A part printed layer by layer under the nozzle's push: plane-stress finite elements with element birth.

Each layer is one row of bilinear quads, h tall, whose columns sit on a fixed grid dx wide, so a
leaning section shifts by a whole number of columns per layer. Printing layer j:
  1. the nozzle pushes the part's top (layer j-1) down with force F, spread over its width;
  2. layer j is born stress-free in that loaded shape: its top at the nozzle's height j*h, its
     bottom on the pushed-down top below (or, where it overhangs, level with it);
  3. the nozzle leaves: the part springs back, carrying layer j up with it.
An element born with node positions X resists with K_e (x - X), K_e computed on X; equilibrium of
all born elements is linear in the positions x. Base nodes are fixed.
"""
import json, math, sys
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

def quad_stiffness(X, E, nu, b):
    """Plane-stress bilinear quad stiffness (8x8) on corner positions X (4x2), thickness b."""
    D = E / (1 - nu * nu) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
    K = np.zeros((8, 8)); g = 1 / math.sqrt(3)
    for xi in (-g, g):
        for eta in (-g, g):
            dN = 0.25 * np.array([[-(1 - eta), (1 - eta), (1 + eta), -(1 + eta)],
                                  [-(1 - xi), -(1 + xi), (1 + xi), (1 - xi)]])
            J = dN @ X; dNx = np.linalg.solve(J, dN)
            B = np.zeros((3, 8))
            B[0, 0::2] = dNx[0]; B[1, 1::2] = dNx[1]; B[2, 0::2] = dNx[1]; B[2, 1::2] = dNx[0]
            K += B.T @ D @ B * np.linalg.det(J) * b
    return K

def quad_strain(X, x):
    """Strain at the centre of a quad born at X and now at x."""
    dN = 0.25 * np.array([[-1, 1, 1, -1], [-1, -1, 1, 1]])
    dNx = np.linalg.solve(dN @ X, dN); u = x - X
    return np.array([dNx[0] @ u[:, 0], dNx[1] @ u[:, 1], dNx[1] @ u[:, 0] + dNx[0] @ u[:, 1]])

def simulate(width, shift, depth, layers, h=0.2, F=1.0, E=1000.0, nu=0.35, per_shift=4, dx_max=0.1):
    """Width and shift per layer of the horizontal section (mm), depth along y (mm); F in N, E in MPa."""
    k = per_shift if shift > 0 else 0
    dx = shift / k if k else dx_max
    n = max(1, round(width / dx))
    ids, pos = {}, []
    def node(c, j, at):
        if (c, j) not in ids:
            ids[(c, j)] = len(pos); pos.append(np.array(at, float))
        return ids[(c, j)]
    for c in range(n + 1):
        node(c, 0, (c * dx, 0.0))
    elems, frames, tops = [], [], []
    def solve(force_on=None):
        N = len(pos); rows, cols, vals = [], [], []; rhs = np.zeros(2 * N)
        for e in elems:
            dof = np.array([[2 * i, 2 * i + 1] for i in e["nodes"]]).ravel()
            Ke = e["K"]
            rows.append(np.repeat(dof, 8)); cols.append(np.tile(dof, 8)); vals.append(Ke.ravel())
            rhs[dof] += Ke @ e["X"].ravel()
        K = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(2 * N, 2 * N))
        if force_on:
            for i, f in force_on:
                rhs[2 * i + 1] -= f
        fixed = np.array([2 * ids[(c, 0)] + d for c in range(n + 1) for d in (0, 1)])
        x = np.array(pos).ravel()
        free = np.setdiff1d(np.arange(2 * N), fixed)
        used = np.unique(np.concatenate(rows))
        free = np.intersect1d(free, used)
        Kff = K[free][:, free]; r = rhs[free] - K[free][:, fixed] @ x[fixed]
        x[free] = spla.spsolve(Kff.tocsc(), r)
        return x.reshape(-1, 2)
    def snapshot(x, phase, j, nozzle):
        stress = []
        D = E / (1 - nu * nu) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
        for e in elems:
            s = D @ quad_strain(e["X"], x[e["nodes"]])
            stress.append(float(math.sqrt(s[0] ** 2 - s[0] * s[1] + s[1] ** 2 + 3 * s[2] ** 2)))
        frames.append({"phase": phase, "layer": j, "nozzle": nozzle,
                       "quads": [x[e["nodes"]].round(5).tolist() for e in elems], "stress": stress})
    x = np.array(pos)
    for j in range(1, layers + 1):
        o_prev, o = (j - 2) * k, (j - 1) * k
        top_prev = [ids[(c, j - 1)] for c in range(max(o_prev, 0), max(o_prev, 0) + n + 1)] if j > 1 else [ids[(c, 0)] for c in range(n + 1)]
        # 1. The nozzle pushes the top down.
        if j > 1:
            force = [(i, F / len(top_prev)) for i in top_prev]
            x = solve(force)
            for i in range(len(pos)):
                pos[i] = x[i]
        loaded = np.array(pos)
        # 2. Layer j is born on the loaded shape, its top at the nozzle's height.
        z_ref = float(np.mean([loaded[i][1] for i in top_prev]))
        for c in range(o, o + n + 1):
            if (c, j - 1) not in ids:   # an overhanging bottom node: level with the top it continues
                node(c, j - 1, (c * dx, loaded[top_prev[-1]][1] if c * dx > loaded[top_prev[-1]][0] else z_ref))
            node(c, j, (c * dx, j * h))
        for c in range(o, o + n):
            nodes = [ids[(c, j - 1)], ids[(c + 1, j - 1)], ids[(c + 1, j)], ids[(c, j)]]
            X = np.array([pos[i] for i in nodes])
            elems.append({"nodes": nodes, "X": X, "K": quad_stiffness(X, E, nu, depth)})
        x = np.array(pos)
        snapshot(x, "deposit", j, [o * dx, (o + n) * dx, j * h])
        # 3. The nozzle leaves: the part springs back.
        x = solve()
        for i in range(len(pos)):
            pos[i] = x[i]
        top = [ids[(c, j)] for c in range(o, o + n + 1)]
        tops.append(float(max(x[i][1] for i in top) - j * h))
        snapshot(x, "release", j, None)
    return {"frames": frames, "excess": tops, "dx": dx, "n": n}

if __name__ == "__main__":
    cases = {
        "tower3": dict(width=3.0, shift=0.0, depth=3.0, layers=100),
        "plate45": dict(width=2 / math.cos(math.radians(45)), shift=0.2 * math.tan(math.radians(45)), depth=10.0, layers=50, per_shift=2),
        "plate60": dict(width=2 / math.cos(math.radians(60)), shift=0.2 * math.tan(math.radians(60)), depth=10.0, layers=50, per_shift=4),
    }
    out = {}
    for name, c in cases.items():
        r = simulate(F=1.0, **c)
        out[name] = r
        print(name, "n", r["n"], "dx", round(r["dx"], 4), "excess at the end per N (E = 1 GPa):", round(r["excess"][-1], 4))
    json.dump(out, open("runs.json", "w"))
