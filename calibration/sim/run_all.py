"""Simulate every measured object with given parameters, in parallel; print simulated against measured."""
import json, sys, time
from concurrent.futures import ProcessPoolExecutor
from printsim import Part, run, PLA
from objects import OBJECTS, schedule

def one(args):
    name, F, h_air, extra = args
    layout, depth, _, measured = OBJECTS[name]
    cols, dx = layout()
    sched = schedule(name)
    assert len(sched) == len(cols), (name, len(sched), len(cols))
    mat = dict(PLA, **extra.get("mat", {}))
    part = Part(cols, dx, 0.2, depth, mat=mat, F=F, h_air=h_air, **{k: v for k, v in extra.items() if k != "mat"})
    t = time.time()
    ex, final = run(part, sched)
    return name, ex[-1], final, measured, time.time() - t

if __name__ == "__main__":
    F = float(sys.argv[1]); h_air = float(sys.argv[2]) * 1e-6
    extra = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    names = sys.argv[4].split(",") if len(sys.argv) > 4 else list(OBJECTS)
    with ProcessPoolExecutor(12) as ex:
        res = list(ex.map(one, [(n, F, h_air, extra) for n in names]))
    print(f"F {F} N, h_air {h_air*1e6:.0f} W/m2K, {extra}")
    for name, last, final, measured, dt in res:
        print(f"  {name:16s} simulated {final:6.3f} (at release {last:6.3f})  measured {measured if measured is not None else float("nan"):5.2f}   [{dt:4.0f} s]")
    json.dump(res, open(f"all_F{F}_h{h_air*1e6:.0f}.json", "w"))
