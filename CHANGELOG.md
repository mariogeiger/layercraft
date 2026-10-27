# Changelog

All notable user-visible changes to this project are documented here.

## 0.4.7

### Fixed

- Bridge strands over air run from support to support: of the directions tried, the one whose
  strands leave the fewest ends over air wins, then the one whose longest strand is shortest.
  Before, only the longest strand counted, and a slab 6 mm wide on two posts 20 mm apart was
  laid across its width, each strand hanging from nothing at both ends.

## 0.4.6

### Changed

- A move from bead to bead over what the layer above covers, inside the outer wall and off
  any bridge over air, neither retracts nor lifts: what oozes on the way is buried. Before,
  only the links between neighbouring infill lines stayed down. The Benchy retracts 1,347
  times instead of 3,173 (Bambu Studio: 1,772), its moves taking 33.2 min instead of 37.0.

## 0.4.5

### Added

- Beads are written as the fewest lines and arcs (`G2`/`G3`) within the profile's `resolution`
  (0.012 mm on the X2D) of the beads traced, arcs only with `enable_arc_fitting`. Each move
  keeps its bead's plastic per mm, speed and fan. The Benchy goes from 768,000 moves to
  102,000, about as many as Bambu Studio's.

## 0.4.4

### Changed

- A wall bead hanging over air runs at the speed of the profile's band it falls in, each band
  of a quarter closed above (`overhang_1_4_speed` for up to a quarter of the bead over air, to
  `overhang_4_4_speed`), and `overhang_totally_speed` wholly over air. Interpolated between
  the bands, the Benchy's lower hull ran faster than the profile asks.

### Added

- The overhang fan: with `enable_overhang_bridge_fan`, a bead hanging over air by at least
  `overhang_fan_threshold`, and every bridge, runs the fan at `overhang_fan_speed` at least.

## 0.4.3

### Fixed

- A filament that does not slow down for layer cooling (`slow_down_for_layer_cooling = 0`, as
  Bambu PLA Basic on the X2D) is no longer slowed: layercraft ignored the switch and slowed
  every layer shorter than `slow_down_layer_time`.
- A layer too short to cool is slowed on its hidden beads first, the surface only once they are
  at their floor. Slowed whole, a short layer laid its outer wall slower than its neighbours,
  which shows as a band on the side of the part.

## 0.4.2

### Added

- A wall slows by the share of its bead that hangs over the layer below: each wall segment
  of the paths stage carries it (`overhang`), the regions stage keeps the section below each
  layer (`support`), and motion takes the profile's overhang speeds (`overhang_1_4_speed` to
  `overhang_4_4_speed`, `overhang_totally_speed`), each held at the middle of its band and
  linear between. The Benchy's cabin arch, laid at 200 mm/s, came out ragged; its apex now
  runs at 10 mm/s.

### Fixed

- An inner wall runs only where its bead passes: through a neck or along a cusp narrower than
  the bead, its loop ran in and back over its own plastic (3 % of the Benchy's walls). The band
  it leaves is filled with the inside.
- With one wall on top, an inner wall lying wholly under the layer above stays a loop starting
  at the seam. Clipped open at an arbitrary point, it sent the nozzle across the part to the
  outer wall's seam on every layer, retracting, lifting and priming on the visible surface.

## 0.4.1

### Fixed

- A bridge strand hangs only over air deeper than it reaches: round, a nozzle wide, it hangs
  `nozzle − h` below its layer (0.2 mm for a 0.4 mm nozzle on 0.2 mm layers). Over an engraving
  one layer deep, as the text under the Benchy's hull, it lay on the bed and filled the letters
  with 57 % more plastic than the layer holds; the layer over such a pocket is now laid flat,
  as solid, and the engraving stays open.

## 0.4.0

### Added

- `layercraft merge` prints several programs on one plate, layer by layer under one frame:
  program stages and G-code from any slicer (`.gcode`, `.gcode.3mf`), each already placed. The
  pieces at one height go nearest first, and each finds the filament as it expects it.
  `--emit program=FILE` writes the merged program, as JSON or as a PLY picture.
- G-code is read back into a program (`layercraft_gcode::read`): positions, filament absolute
  or relative, feed, acceleration and fan followed, arcs kept, the machine's own G-code left
  out; an unknown command or feature is an error. A layer starts at its first travel or bead:
  the wipe, pull and lift before it leave the layer below.
- Arcs: a move may turn about a vertical axis while it climbs, any angle up to a full turn,
  written `G2`/`G3`; the lift's helix is one. Time, verify and pictures follow arcs exactly.
- Roles are three facts: the kind of bead (wall, dense line, sparse line), the face it makes
  (outside, top, bottom, none) and what it rests on (solid, sparse infill, air). A strand is a
  dense line on anything but solid; bottom surfaces, overhang walls, floating shells and
  internal bridges are named. `monotone` takes faces: the profile's bottom surface pattern now
  counts too.
- `melt` holds every bead to what the hot end melts; `deliver` says how many it slowed.

### Changed

- The program stage is layers of moves, each with its whole state (where, how, filament,
  speed, acceleration, fan, bead): no machine G-code, no summary, no progress in it. The frame
  (start, layer changes, end) is expanded when written, from the program as it stands; the
  summary and the printer's progress are read from the moves.
- Cooling is a pass of its own over any program (`cool`), after the program stage, which keeps
  full speeds.
- A layer's first move, from wherever the machine's own G-code leaves the nozzle, is one
  travel over at the hop height.
- The thumbnail is drawn from the program's beads, so a merged or read program has one.

### Fixed

- A bead too short to be written at three decimals no longer leaves a `G1 E0.00001` push in
  place: its filament, and every bead's rounding, carries to the next bead, the total exact.

## 0.3.1

### Fixed

- Bridge strands resting on sparse infill fill their layer exactly: they are spaced
  `π·w²/(4h)` apart (0.63 mm for a 0.4 mm nozzle on 0.2 mm layers) instead of touching, which
  laid 57 % more plastic than the layer holds and pushed it up into the top surface. Strands
  over air still touch, where they hang.

## 0.3.0

### Added

- Settings by place: `--tune FILE.json` takes a list of overrides, each a box of the build
  volume and part of the settings (`{"within": {"z": [0, 5]}, "set": {"motion": {"lift_radius":
  0.6}}}`). Every region, path and motion setting can vary by height band, by object or by any
  box: each shape, island and bead takes the settings at its centre, merged in order. Several
  variants of a test print on one plate need no editing of stage files. A misspelt setting or
  stage is an error.
- A flow per role, read from the profile for bridges (`bridge_flow`) and top surfaces
  (`top_solid_infill_flow_ratio`).
- Monotone surfaces: the top surface is laid in sweeps, each line against the one before and
  all the same way, as the profile's `top_surface_pattern` asks, instead of nearest first.

### Changed

- The stages take a `layercraft_tune::Field` of parameters instead of one set.
- A program's summary and the printer's progress are read from its moves (`finish`), so a
  program edited by hand and resumed with `--from` reports its own time and filament. The
  plastic per layer is what the extrusions push, the flow ratio included.

### Fixed

- The hot end's melt limit caps a bead's speed by the plastic it pushes, flow included, as
  `verify` checks it, not by its nominal section.
- A retraction is pushed back by exactly what was pulled, whatever the next bead's settings.

## 0.2.10

### Documentation

- The README shows what layercraft does: the five stages on the 3DBenchy, the bead's
  cross-section and the formulas behind the filament, and the stages seen in SolidCraft.

## 0.2.9

### Fixed

- A spiral lift turns a helix of 1.2 mm, the smallest radius that broke the end-of-print string
  in a series of test cubes on the X2D (0, 0.6, 1.2 and 2.4 mm), instead of 3.18 mm derived
  from the axes' speeds: as clean, faster, and gentler on top surfaces.

## 0.2.8

### Fixed

- A program's PLY picture names the role of the moves that lay no plastic `-1 none`, in the
  form every other label takes (`role: 0 outer wall, …`), so a viewer can read it as a legend.

## 0.2.7

### Added

- Any stage as a picture: `--emit STAGE=FILE.ply` writes the stage as a PLY file of points,
  segments and triangles with named properties, for SolidCraft or any viewer to colour and
  filter by. Sections and region areas are triangulated exactly on the lattice; walls, beads
  and moves are segments, a bead with its `width` and `height`, at its centre half its height
  under the nozzle; the program's points carry their time `t`, so `t ≤ t₀` replays the print.
  A role has the same code in every stage.

## 0.2.6

### Fixed

- No more string pulled up off the part at the end of a print, nor at each lift: the nozzle
  leaves the part as the profile's spiral lift asks, one turn of a helix (`G3 … P1`) back over
  where it started, moving sideways as it rises instead of straight up. Its radius makes the Z
  axis at its top speed keep pace with the travel speed (3.18 mm on the X2D). A filament's own
  lift height and type win over the printer's; a slope lift is not supported yet.
- No more `G1 E0` lines: a move that changes nothing at the precision written is left out.

## 0.2.5

### Fixed

- No region keeps a contour thinner than a micrometre: booleans no longer leave rounding
  debris, such as hundreds of fill scraps on the layer under raised letters.
- What stands on a layer decides its top shells, not only the air: under a thin rib or raised
  letters, all walls and no inside, the layers below are solid for the top shells' depth, as
  under a top surface, instead of a few sparse lines.

## 0.2.4

### Fixed

- Offsets drop the slivers, thinner than a nanometre, that rounding left where a mesh's very
  short edges meet: no more zero-length wall loops, nor scraps of top surface and solid infill
  next to thin features, each costing a travel and a retraction.

## 0.2.3

### Fixed

- A bridge over air crosses its gap the short way: its strands take the direction whose
  longest strand is shortest, and reach a bead's width onto the support at both ends. A bridge
  over sparse infill keeps the layer's direction, across the sparse lines below.

## 0.2.2

### Fixed

- A piece of sparse infill smaller than the process's `minimum_sparse_infill_area` (15 mm² on
  the X2D) is solid: a thin tower's core no longer gets a stray line or two.
- `only_one_wall_top` holds wherever a layer meets the air, not only on layers that are all top:
  inner walls stay under the layer above and the top surface reaches the outer wall elsewhere.

## 0.2.1

### Fixed

- Bridges only where a strand must span something: over air wider than a bead, over sparse
  infill wider than the gap between its lines. The stamp's bridges fall from 74 to 28 mm.
- Infill lines no longer lay plastic along the area's edge between them: the move from one
  line to the next stays inside the part, without retracting or lifting, so the density is
  what the spacing gives. The 20 mm square now takes 341 mm of filament, Bambu Studio 344.

## 0.2.0

### Added

- Bridges: solid areas over sparse infill or over air are laid as round strands a nozzle wide,
  at the bridge speed.
- Infill lines join into zigzags along the area's edge, so the nozzle no longer lifts and
  travels between neighbouring lines (63 lifts instead of 177 on the 20 mm test square).
- The first layer's beads take the first-layer width (0.5 mm on the X2D), to grip the bed.
- A layer that is all top surface keeps only its outer wall (`only_one_wall_top`).

### Changed

- Wall loops stop the seam gap (15 % of the nozzle) short of their start, so seams do not pile
  up into a dot. Stage files gain a `bridge` region and role: older stage files do not load.

## 0.1.4

### Fixed

- No more string at the end of a print or after a travel: each retraction wipes back over the
  bead just laid (the filament's wipe distance), pulling the filament at the retraction speed,
  and the nozzle lifts off the part before the machine's end sequence.
- Sparse infill no longer piles up where its lines crossed: one direction per layer, turning a
  quarter turn each layer, at the spacing that keeps the density.

## 0.1.3

### Fixed

- The X2D refused to start layercraft's `.gcode.3mf` (error 0500-4046): the plate summary now
  describes the extruders and nozzle, the filament order and the client version of the
  imported profiles, as the firmware expects. Run `layercraft machine import` again.

## 0.1.2

### Fixed

- Walls and solid infill no longer leave voids: beads sit their tiling spacing apart,
  `w − h·(1 − π/4)`, the width their rounded cross-section fills, instead of their full width.
  Solid layers had about 10 % too little plastic.

## 0.1.1

### Fixed

- Sections, regions and toolpaths no longer hold repeated points: every geometric operation
  works in integer nanometres on one lattice anchored at the origin, so consecutive points are
  at least a nanometre apart and results from different operations meet exactly.

## 0.1.0

### Added

- The project's mission (`SOUL.md`), agent instructions and repository hooks.
- `layercraft compile`: a 3MF through exact layer sections, walls, solid shells and sparse
  infill, toolpaths, timed moves and `verify`, to a `.gcode.3mf` for the Bambu Lab X2D.
- `--emit STAGE=FILE` and `--from STAGE=FILE` at every stage (mesh, sections, regions, paths,
  program).
- `layercraft machine import`: the printer, process and filament read once from an installed
  Bambu Studio's profiles; the printer's own G-code templates expanded by a strict interpreter.
- `layercraft view`: a SolidCraft script drawing the toolpaths, one sketch per layer and role.
- `layercraft compare`: two G-code files side by side, layer by layer and feature by feature.
