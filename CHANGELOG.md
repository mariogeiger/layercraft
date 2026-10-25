# Changelog

All notable user-visible changes to this project are documented here.

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
