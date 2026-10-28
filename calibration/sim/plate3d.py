"""Compliance of a leaning plate under the nozzle in 3D: a point load moving along the plate's
depth against the same force spread over the depth (what the 2D section assumes).
Trilinear hexahedra, E = 1000 MPa, nu = 0.35, base fixed; the plate 2 mm thick, 10 mm deep."""
import math, sys
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

def hex_k(X, E=1000.0, nu=0.35):
    lam = E * nu / ((1 + nu) * (1 - 2 * nu)); mu = E / (2 * (1 + nu))
    D = np.zeros((6, 6)); D[:3, :3] = lam; D[np.arange(3), np.arange(3)] += 2 * mu; D[np.arange(3, 6), np.arange(3, 6)] = mu
    sgn = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float)
    K = np.zeros((24, 24)); g = 1 / math.sqrt(3)
    for a in (-g, g):
        for b in (-g, g):
            for c in (-g, g):
                p = np.array([a, b, c])
                dN = np.array([sgn[:, i] * np.prod(1 + sgn[:, [j for j in range(3) if j != i]] * p[[j for j in range(3) if j != i]], axis=1) / 8 for i in range(3)])
                J = dN @ X; dNx = np.linalg.solve(J, dN)
                B = np.zeros((6, 24))
                for n in range(8):
                    dx, dy, dz = dNx[:, n]
                    B[0, 3*n] = dx; B[1, 3*n+1] = dy; B[2, 3*n+2] = dz
                    B[3, 3*n] = dy; B[3, 3*n+1] = dx; B[4, 3*n+1] = dz; B[4, 3*n+2] = dy; B[5, 3*n] = dz; B[5, 3*n+2] = dx
                K += B.T @ D @ B * np.linalg.det(J)
    return K

def compliance(theta, H=10.0, t=2.0, depth=10.0, nu_=4, nz=40, ny=20):
    w = t / math.cos(math.radians(theta)); tn = math.tan(math.radians(theta))
    idx = lambda i, k, j: (i * (nz + 1) + k) * (ny + 1) + j
    P = np.zeros(((nu_ + 1) * (nz + 1) * (ny + 1), 3))
    for i in range(nu_ + 1):
        for k in range(nz + 1):
            for j in range(ny + 1):
                z = H * k / nz; P[idx(i, k, j)] = (z * tn + w * i / nu_, depth * j / ny, z)
    rows, cols, vals = [], [], []
    for i in range(nu_):
        for k in range(nz):
            for j in range(ny):
                nodes = [idx(i, k, j), idx(i + 1, k, j), idx(i + 1, k, j + 1), idx(i, k, j + 1),
                         idx(i, k + 1, j), idx(i + 1, k + 1, j), idx(i + 1, k + 1, j + 1), idx(i, k + 1, j + 1)]
                # local order (x, y, z) sign convention: (u, depth, z)
                Ke = hex_k(P[nodes][:, [0, 1, 2]])
                dof = np.array([[3 * n, 3 * n + 1, 3 * n + 2] for n in nodes]).ravel()
                rows.append(np.repeat(dof, 24)); cols.append(np.tile(dof, 24)); vals.append(Ke.ravel())
    N = len(P) * 3
    K = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(N, N))
    fixed = np.array([3 * idx(i, 0, j) + d for i in range(nu_ + 1) for j in range(ny + 1) for d in range(3)])
    free = np.setdiff1d(np.arange(N), fixed)
    solve = spla.factorized(K[free][:, free].tocsc())
    def defl(loads):
        f = np.zeros(N)
        for n, v in loads: f[3 * n + 2] -= v
        u = np.zeros(N); u[free] = solve(f[free]); return u
    top = [(idx(i, nz, j)) for i in range(nu_ + 1) for j in range(ny + 1)]
    u = defl([(n, 1.0 / len(top)) for n in top])
    spread = -np.mean(u[[3 * n + 2 for n in top]])
    # A point load (over the section's width) at each position along the depth: the deflection under it.
    under = []
    for j in range(ny + 1):
        nodes = [idx(i, nz, j) for i in range(nu_ + 1)]
        u = defl([(n, 1.0 / len(nodes)) for n in nodes])
        under.append(-np.mean(u[[3 * n + 2 for n in nodes]]))
    return spread, np.array(under)

for theta in (45, 60):
    s, under = compliance(theta)
    print(f"{theta} deg: spread over the depth {s:.4f} mm/N; point load moving along y: mid {under[len(under)//2]:.4f}, edge {under[0]:.4f}, mean {under.mean():.4f} mm/N -> x{under.mean()/s:.2f}")
