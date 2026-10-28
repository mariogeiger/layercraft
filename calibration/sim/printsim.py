"""A part printed layer by layer, its heat and its stresses, in one x-z section of depth b.

Cells are one layer h tall and dx wide on a fixed column grid; layer j's cells are born at the
time the G-code reaches the object (timing*.json), at the nozzle's temperature, stress-free in
the shape the part has then. Heat: implicit finite volumes, conduction k between cells, the bed
under the first layer, convection h_air on every face open to the air, the section's front and
back included. Stress: plane-stress bilinear quads; each cell is a standard linear solid, a
spring E(T) (glassy E_g below the glass transition T_g, rubbery E_r above, a sigmoid in ln E)
whose reference shape relaxes towards its present one with the time 3·eta(T)/E(T), eta the
Vogel-Fulcher-Tammann viscosity of layercraft-physics. While the nozzle lays layer j it pushes
the top beneath it down with the force F.
"""
import math
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

CONTACT = 1e4  # N/mm: the nozzle as a wall, stiff against the part
PLA = dict(rho=1.24e-3, c=1.8, k=0.13e-3, T_g=60.0, T_melt=220.0, eta_melt=1e3, eta_glass=1e12, wlf_c2=51.6,
           E_g=3500.0, E_r=3.0, sigmoid=3.0, nu=0.35, alpha=0.0)
# Units: mm, s, g, N, MPa (N/mm2), W = J/s; rho in g/mm3, c in J/(g K), k in W/(mm K), h in W/(mm2 K).

def vft(m):
    """eta(T) in Pa s through eta_glass at T_g and eta_melt at T_melt, T0 = T_g - C2."""
    t0 = m["T_g"] - m["wlf_c2"]
    b = (math.log(m["eta_glass"]) - math.log(m["eta_melt"])) / (1 / (m["T_g"] - t0) - 1 / (m["T_melt"] - t0))
    ln_inf = math.log(m["eta_melt"]) - b / (m["T_melt"] - t0)
    def eta(T):
        return np.exp(ln_inf + b / np.maximum(T - t0, 1e-3))
    return eta

def modulus(m):
    def E(T):
        s = 1 / (1 + np.exp(-(m["T_g"] - T) / m["sigmoid"]))
        return np.exp(math.log(m["E_r"]) + (math.log(m["E_g"]) - math.log(m["E_r"])) * s)
    return E

def quad_k0(X, nu, b):
    """Plane-stress bilinear quad stiffness for E = 1 on corners X (4x2), thickness b."""
    D = 1 / (1 - nu * nu) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
    K = np.zeros((8, 8)); g = 1 / math.sqrt(3)
    for xi in (-g, g):
        for eta in (-g, g):
            dN = 0.25 * np.array([[-(1 - eta), (1 - eta), (1 + eta), -(1 + eta)], [-(1 - xi), -(1 + xi), (1 + xi), (1 - xi)]])
            J = dN @ X; dNx = np.linalg.solve(J, dN)
            B = np.zeros((3, 8)); B[0, 0::2] = dNx[0]; B[1, 1::2] = dNx[1]; B[2, 0::2] = dNx[1]; B[2, 1::2] = dNx[0]
            K += B.T @ D @ B * np.linalg.det(J) * b
    return K

class Part:
    def __init__(self, columns, dx, h, b, mat=PLA, T_nozzle=220.0, T_bed=55.0, T_air=35.0, h_air=60e-6, F=2.0,
                 thermal_only=False):
        """columns[j-1] = (first, end) column range of layer j."""
        self.columns, self.dx, self.h, self.b, self.m = columns, dx, h, b, mat
        self.T_nozzle, self.T_bed, self.T_air, self.h_air, self.F = T_nozzle, T_bed, T_air, h_air, F
        self.eta, self.E = vft(mat), modulus(mat)
        self.thermal_only = thermal_only
        self.cell = {}; self.cells = []          # (c, j) -> index; list of (c, j)
        self.T = np.zeros(0); self.T_rest = np.zeros(0)
        self.node = {}; self.pos = []            # (c, j) -> index; positions
        self.k0 = []; self.ref = []; self.enodes = []
        a, e = columns[0]
        for c in range(a, e + 1):
            self._node(c, 0, (c * dx, 0.0))
        self.load = None
        self.x = np.array(self.pos)

    def _node(self, c, j, at):
        if (c, j) not in self.node:
            self.node[(c, j)] = len(self.pos); self.pos.append(np.array(at, float))
        return self.node[(c, j)]

    # Heat.
    def _heat_graph(self):
        """Conductances between cells and to the bed and the air, rebuilt when cells are born."""
        n = len(self.cells); m = self.m
        rows, cols, vals = [], [], []; diag = np.zeros(n); src = np.zeros(n)
        gx = m["k"] * self.h * self.b / self.dx; gz = m["k"] * self.dx * self.b / self.h
        air_side, air_top = self.h_air * self.h * self.b, self.h_air * self.dx * self.b
        air_faces = 2 * self.h_air * self.dx * self.h
        g_bed = m["k"] * self.dx * self.b / (self.h / 2)
        for i, (c, j) in enumerate(self.cells):
            for (nc, nj), g, open_g in (((c + 1, j), gx, air_side), ((c - 1, j), gx, air_side), ((c, j + 1), gz, air_top), ((c, j - 1), gz, air_top)):
                k = self.cell.get((nc, nj))
                if k is not None:
                    rows.append(i); cols.append(k); vals.append(-g); diag[i] += g
                elif nj == 0:
                    diag[i] += g_bed; src[i] += g_bed * self.T_bed
                else:
                    diag[i] += open_g; src[i] += open_g * self.T_air
            diag[i] += air_faces; src[i] += air_faces * self.T_air
        self.G = sp.csr_matrix((vals + list(diag), (rows + list(range(n)), cols + list(range(n)))), shape=(n, n))
        self.src = src
        self.factors = {}

    def _heat_matrix(self, dt):
        key = round(dt, 9)
        if key not in self.factors:
            C = self.m["rho"] * self.m["c"] * self.dx * self.h * self.b
            n = len(self.cells)
            self.factors[key] = spla.factorized((self.G + sp.identity(n) * (C / dt)).tocsc())
        return self.factors[key], self.m["rho"] * self.m["c"] * self.dx * self.h * self.b / dt, self.src

    def set_air(self, T_air, T_bed):
        self.T_air, self.T_bed = T_air, T_bed
        self._heat_graph()

    # Stress.
    def _solve(self, dt):
        if self.thermal_only or not self.k0:
            return
        T = self.T
        E = self.E(T)
        K0 = np.array(self.k0); ref = np.array(self.ref); en = np.array(self.enodes)
        # Heat expands each cell's rest shape about its centre, cold shrinks it: alpha per K.
        if self.m["alpha"]:
            dT = T - self.T_rest
            mid = ref.mean(axis=1, keepdims=True)
            ref = mid + (ref - mid) * (1 + self.m["alpha"] * dT)[:, None, None]
            self.T_rest = T.copy()
        dof = np.stack([2 * en, 2 * en + 1], axis=2).reshape(len(en), 8)
        N = len(self.pos)
        Ke = E[:, None, None] * K0
        rows = np.repeat(dof, 8, axis=1).ravel(); cols = np.tile(dof, (1, 8)).ravel()
        K = sp.csr_matrix((Ke.ravel(), (rows, cols)), shape=(2 * N, 2 * N))
        rhs = np.zeros(2 * N)
        np.add.at(rhs, dof.ravel(), np.einsum("eij,ej->ei", Ke, ref.reshape(len(en), 8)).ravel())
        x = np.array(self.pos).ravel()
        a, e = self.columns[0]
        fixed = np.array([2 * self.node[(c, 0)] + d for c in range(a, e + 1) for d in (0, 1)])
        free = np.setdiff1d(np.unique(dof), fixed)
        Kff = K[free][:, free].tocsc(); base = rhs[free] - K[free][:, fixed] @ x[fixed]
        if not self.load:
            x[free] = spla.spsolve(Kff, base)
        else:
            # The squeeze: through a gap g the melt needs the pressure that pushes the same flow,
            # so each column's share of F grows as (h/g)^3 (Poiseuille). Newton on the gap.
            where = {d: i for i, d in enumerate(free)}
            pairs, zn, A = self.load
            for _ in range(30):
                # Below the gap g_min the nozzle touches the plastic: it is a rigid wall then.
                g_min = 0.5 * self.h
                g = zn - x[2 * np.array([q for _, q in pairs]) + 1]
                ge = np.maximum(g, g_min)
                f = A * (self.h / ge) ** 3 + CONTACT * np.maximum(g_min - g, 0)
                k = np.where(g > g_min, 3 * A * self.h ** 3 / ge ** 4, CONTACT)
                r = base.copy(); kc = np.zeros(len(free))
                for (i, _), fi, ki in zip(pairs, f, k):
                    r[where[2 * i + 1]] -= fi; kc[where[2 * i + 1]] += ki
                Kt = (Kff + sp.diags(kc)).tocsc()
                res = r - Kff @ x[free]
                dxf = spla.spsolve(Kt, res)
                x[free] += dxf
                if np.max(np.abs(dxf)) < 1e-7:
                    break
        self.x = x.reshape(-1, 2)
        self.pos = list(self.x)
        # Each cell's reference relaxes towards its present shape.
        tau = 3 * self.eta(T) / (E * 1e6)
        lam = 1 - np.exp(-dt / np.maximum(tau, 1e-9))
        cur = self.x[en]
        new_ref = ref + lam[:, None, None] * (cur - ref)
        self.ref = list(new_ref)

    def step(self, dt):
        if len(self.cells):
            solve, cdt, src = self._heat_matrix(dt)
            self.T = solve(cdt * self.T + src)
        self._solve(dt)

    def birth(self, j):
        """Lay layer j at the nozzle's height on the part as it is now (pushed down)."""
        a, e = self.columns[j - 1]
        below = [self.node[(c, j - 1)] for c in range(a, e + 1) if (c, j - 1) in self.node]
        zref = float(np.mean([self.pos[i][1] for i in below])) if below else (j - 1) * self.h
        for c in range(a, e + 1):
            if (c, j - 1) not in self.node:
                self._node(c, j - 1, (c * self.dx, zref))
            self._node(c, j, (c * self.dx, j * self.h))
        for c in range(a, e):
            self.cell[(c, j)] = len(self.cells); self.cells.append((c, j))
            nodes = [self.node[(c, j - 1)], self.node[(c + 1, j - 1)], self.node[(c + 1, j)], self.node[(c, j)]]
            X = np.array([self.pos[i] for i in nodes])
            self.enodes.append(nodes); self.ref.append(X.copy()); self.k0.append(quad_k0(X, self.m["nu"], self.b))
        self.T = np.concatenate([self.T, np.full(e - a, self.T_nozzle)])
        self.T_rest = np.concatenate([self.T_rest, np.full(e - a, self.T_nozzle)])
        self._heat_graph()

    def push(self, j, duration):
        """The nozzle laying layer j presses the melt beneath it, which passes the pressure down to
        where the plastic holds its shape for the time it is pressed (3 eta / E > duration): the
        force F, spread over the columns of layer j, acts in each on the top of its highest such cell."""
        a, e = self.columns[j - 1]
        E = self.E(self.T); tau = 3 * self.eta(self.T) / (E * 1e6)
        holds = {cj: tau[i] > duration for cj, i in self.cell.items()}
        pairs = []
        for c in range(a, e):
            for jj in range(j - 1, 0, -1):
                if holds.get((c, jj)):
                    for cc in (c, c + 1):
                        gap = self.node.get((cc, j - 1))
                        if gap is not None:
                            pairs.append((self.node[(cc, jj)], gap))
                    break
        self.load = (pairs, j * self.h, self.F / len(pairs)) if pairs else None

    def top_excess(self, j):
        a, e = self.columns[j - 1]
        return max(self.pos[self.node[(c, j)]][1] for c in range(a, e + 1)) - j * self.h

def run(part, schedule, cooldown=300.0, dt_on=0.02, dt_max=1.0, record=None):
    """schedule: per layer (t0, t1), the nozzle on the object from t0 to t1."""
    t = schedule[0][0]
    excess = []
    for j, (t0, t1) in enumerate(schedule, start=1):
        # Wait until the nozzle comes back, in steps growing from dt_on.
        dt = dt_on
        while t < t0 - 1e-9:
            d = min(dt, t0 - t); part.step(d); t += d; dt = min(dt * 1.5, dt_max)
        part.push(j, max(t1 - t0, dt_on))
        part._solve(1e-6)                 # pushed down before the bead lands
        part.birth(j)
        while t < max(t1, t0 + dt_on) - 1e-9:
            d = min(dt_on, max(t1, t0 + dt_on) - t); part.step(d); t += d
        part.load = None
        part.step(dt_on); t += dt_on
        excess.append(part.top_excess(j))
        if record is not None:
            record(part, j, t)
    part.set_air(25.0, 25.0)
    dt = dt_on
    elapsed = 0.0
    while elapsed < cooldown:
        part.step(dt); elapsed += dt; dt = min(dt * 1.5, 20.0)
    return excess, part.top_excess(len(schedule))
