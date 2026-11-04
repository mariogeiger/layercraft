# layercraft soul

This file holds the mission of layercraft. It changes more slowly than the
code; technical detail belongs in the code and its file headers.

## Mission

layercraft is a compiler built for AI agents. It takes a solid exported by a
CAD program, SolidCraft first, all the way down to a print on a Bambu Lab X2D.
Think of it as a slicer with the hood off: every stage, from the mesh through
layer sections, regions and toolpaths to the motion the printer runs, writes a
file the agent can read, change and compile on from, and SolidCraft can show
the toolpaths as a design. The compiler is made of small pieces; each does one
thing and does exactly what it says, so the agent can change one and rely on
the rest. Its geometry is exact where exactness gives truth: each layer is the
exact section of the mesh, and every millimetre of filament follows from the
volume it must fill. The nozzle moves freely in the plane, so the plate has no
preferred direction, and neither has the compiler: every algorithm is
equivariant under the plate's isometries, E(2), so a part turned, moved or
mirrored on the plate is printed turned, moved or mirrored, and nothing else
changes. A direction comes from the part's own geometry, its walls, its strips,
its slopes, never from the plate's axes or from a grid laid on them. Only the
machine may break that symmetry, where its hardware does, and its description
says how. Where a symmetric part leaves a choice open, a seam, a start, an
order, the part's own frame makes it. It sends the result to the printer, and
starts the print only when a person explicitly asks it to. It is our own code:
other slicers serve only as references to compare against. It is written in
Rust, which we believe is one of the best languages for agents: its compiler
catches many mistakes before the code ever runs.

- Built for AI agents.
- Solid in, print out.
- Open at every stage.
- Small, independent pieces.
- Each piece does exactly what it says.
- Equivariant: no direction but the part's own.
- Our own code; other slicers as references only.
- Rust.
