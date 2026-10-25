# layercraft

A compiler that takes a solid all the way down to a print on a Bambu Lab X2D.

Slicers are usually closed boxes: a mesh goes in, G-code comes out, and what
happens in between can only be tuned through settings. layercraft is a slicer
with the hood off, built for AI agents: every stage, from the layer sections to
the motion the printer runs, is a file you can read, change and compile on from.

## Design principles

- **Open at every stage.** `--emit <stage>` writes any stage; `--from <file>` resumes from it.
- **Exact sections, volume-exact extrusion.** A layer is the exact section of the
  mesh, and each millimetre of filament follows from the volume it fills.
- **Never harm the machine.** No program leaves the compiler without passing `verify`,
  and a print starts only when a person asks.
- **Our own code.** MIT OR Apache-2.0, written from public specifications;
  other slicers serve only as references to compare against.

Read the [project soul](SOUL.md) for the mission and [AGENTS.md](AGENTS.md)
for how to work on it.
