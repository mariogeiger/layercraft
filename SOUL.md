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
volume it must fill. It sends the result to the printer, and starts the print
only when a person explicitly asks it to. It is our own code: other slicers
serve only as references to compare against. It is written in Rust, which we
believe is one of the best languages for agents: its compiler catches many 
mistakes before the code ever runs.

- Built for AI agents.
- Solid in, print out.
- Open at every stage.
- Small, independent pieces.
- Each piece does exactly what it says.
- Our own code; other slicers as references only.
- Rust.
