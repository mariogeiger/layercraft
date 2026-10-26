# Changelog

All notable user-visible changes to this project are documented here.

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
