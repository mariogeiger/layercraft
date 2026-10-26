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

## Use

```bash
cargo install --path apps/layercraft
layercraft machine import                     # once: the X2D, 0.20 mm Standard, Bambu PLA Basic
layercraft compile part.3mf --out part.gcode.3mf
layercraft compile part.3mf --out part.gcode.3mf --emit paths=paths.json   # write a stage
layercraft compile --from paths=paths.json --out part.gcode.3mf            # resume from it
layercraft compile part.3mf --out part.gcode.3mf --emit program=program.ply # a stage as a picture
layercraft view paths.json --out view.json    # a SolidCraft script: one sketch per layer and role
layercraft compare part.gcode.3mf other.gcode # side by side with another slicer's output
```

`machine import` reads the printer maker's profiles from an installed Bambu Studio, once, into
`~/.config/layercraft/`; nothing from them enters this repository. The stages are `mesh`,
`sections`, `regions`, `paths` and `program`. A stage written to a `.ply` file is a picture of it:
points, segments and triangles with properties (layer, role, width, time…) to colour and filter by.

Read the [project soul](SOUL.md) for the mission and [AGENTS.md](AGENTS.md)
for how to work on it.
