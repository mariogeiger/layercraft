# Changelog

All notable user-visible changes to this project are documented here.

## 0.21.0

### Added

- `compile --support-interface N` lays the supports' interface in filament N, in a pass of its
  own on the layers that have it: a filament that does not weld to the part, as PLA under PETG,
  lets the columns come off it by hand. The PETG supports of the USB-C cable support took
  pliers to remove, and the part broke.
- `--material` names one filament per material, comma separated, the last standing for those
  past them (`--material petg-tr,petg-tr,pla`). Each filament prints with its own settings: its
  temperatures, its own item in the printer templates' per-filament lists, its speeds, flow,
  fan and melt limit, and its type, code and density in the package. A change between two
  filaments of one colour but two kinds flushes.

### Changed

- `layercraft_motion::compile` takes one motion field per material and `limit` one set of move
  limits per material, the last standing for those past them. `Settings::heat_for`,
  `scope_for` and `flush` are `Filaments::heat`, `scope` and `flush`, over each filament's
  settings. `SupportParams` has `interface_material`.

## 0.20.1

### Changed

- The first pass after a filament change goes without the part fan, the nozzle as hot as on the
  first layer, back to the filament's own temperature on the next layer: it lays its beads on
  plastic the change, a minute or two, left to cool, and must melt into it. The USB-C cable
  support printed in two PETG broke at its colour boundary, a flat, smooth separation.

## 0.20.0

### Changed

- A support's column is a thin shell: one loop along its edge, stacked layer on layer, and no
  lines inside. Its interface is laid across it the short way, every line running from the loop
  to the loop, so that each rests on the shell at both ends; the first layer of the interface
  spans the column over air, slowly, as a strand on air hangs (`Support::Air`), the next rest on
  it. Before, the interface was hatched over the whole column while the lines under it, 2.5 mm
  apart, did not follow its edge: lines hung past the last one into the air.
- A roof standing on a column is laid as solid, a layer thick, `top_gap` above the column, all of
  it at one height. Before, its middle was laid as hanging strands and the column stopped a
  strand's depth lower, leaving the roof's walls 0.4 mm over the column instead of 0.2.
- The X2D's qualities let a strand held at both ends bridge any length (`bridge_no_support`), as
  the owner has seen the printer do slowly: only what hangs from one side stands on supports.
  A 1.4 mm slot bridged between two posts stood on its own columns before.
- `SupportLines` has no `gap` any more, and `support_base_pattern_spacing` is not read.

## 0.19.0

### Changed

- Supports are no option: whatever a layer cannot bridge stands on columns, always. The
  `--supports` switch of `compile` and `view` and the qualities' `enable_support` are gone, and
  the region and path stages require their support settings. A part that needed supports printed
  without them before, its overhangs laid into the air.
- A part that would hang where no support can stand is refused: when a roof is left over air past
  the columns under it (columns kept on the plate, a gap too wide), the regions stage fails
  naming the layer, its height and the area left hanging, instead of writing a print that would
  surely fail.

## 0.18.4

### Changed

- The X2D's filament change is layercraft's own (`printers/bambu-lab-x2d/filament-change.gcode`)
  instead of the maker's: the same commands for every setting tried, without the maker's
  comments and commented-out commands. Each material and the printer now state the settings it
  reads (the cooling before the new filament prints, the retractions when the filament is cut
  and when it leaves the extruder, the retraction at a change). No G-code of the maker's is used
  any more.
- A colour change flushes 60 mm³ (25 mm of filament) whatever the colours, only enough to prime
  the nozzle, instead of 250 mm³ plus up to 600 more towards a lighter colour: the old colour
  mixes into the new one's first beads, the inner walls, as the owner prefers to wasting
  filament. Grey to pink flushed about 450 mm³ before.
- A filament has no G-code of its own: the `filament_start_gcode` and `filament_end_gcode` hooks
  are gone from the frame, the printer's sequences reading the filament's settings instead. The
  frame no longer gives the variables only the maker's change read (`toolchange_count`, the
  position after the change, the travel points, the feed rates, the prime tower), nor the
  placeholders only its other templates read.

## 0.18.3

### Changed

- The X2D's start is layercraft's own (`printers/bambu-lab-x2d/start.gcode`) instead of the
  maker's: the same commands in the same order, the tune and the logo lamp kept, for every plate
  and its temperature, every part height and every filament type tried, without the maker's
  comments and commented-out commands. Printed on the X2D in PLA: every stage ran in order, from
  the plate checks to the purge line, and the print finished without error.

## 0.18.2

### Changed

- The X2D's layer change is layercraft's own (`printers/bambu-lab-x2d/layer-change.gcode`)
  instead of the maker's: at each layer the same commands, the layer counter, the auxiliary part
  fans held through the filament's first layers, then the printer's own chamber cooling, without
  the maker's comments and empty lines. Each material now states the three settings it reads:
  `close_additional_fan_first_x_layers`, `first_x_layer_fan_speed` and
  `additional_fan_full_speed_layer`. Printed on the X2D in PLA: the auxiliary fans ran at 40 %
  through layer 1 and at 70 % from layer 2, under the printer's chamber cooling.
- In the G-code templates, `{# … #}` is a comment for the template's reader that prints nothing,
  and a line holding only branches and comments leaves no empty line: the maker's start and
  filament change lose their empty lines.

## 0.18.1

### Changed

- The X2D's end is layercraft's own (`printers/bambu-lab-x2d/end.gcode`) instead of the maker's:
  the same heaters and fans off, filament back into the AMS, parked head, lowered plate (now to
  80 mm, or 10 mm under the top of a taller part), reset scales and detectors and air
  purification, with a tune of its own and without the timelapse. Printed on the X2D.

## 0.18.0

### Changed

- The X2D's settings, the printer's, two qualities' and two materials', are layercraft's own, in
  `printers/bambu-lab-x2d/`, and built into it: every computer prints with the same, and there is
  nothing to import or keep in step. `--quality 0.20|0.12` and `--material pla|petg-tr` replace
  `--machine` (`--machine x2d-hq-petg-tr` is now `--quality 0.12 --material petg-tr`), and
  `layercraft machine import` and `machine show` are gone. Until layercraft writes its own
  G-code, the maker's profiles, read from an installed Bambu Studio at each compile, bring the
  start, end, layer-change and filament-change templates. A part compiles to the same file, byte
  for byte, as with the descriptions imported before.

## 0.17.5

### Changed

- Sparse infill is convection rolls by default: `compile` and `view` lay what `--infill rolls`
  laid, and `--infill lines` lays straight lines as before. On a Benchy the print takes 42.5 min
  of motion against 42.6 and 3791 mm of filament against 3809; compiling it takes 108 s against
  92 and holds 1.54 GB against 1.42.

## 0.17.4

### Changed

- A bridge's strands now cross the air as slowly as a wall wholly over it, at the profile's
  last overhang speed (10 mm/s with `0.20mm Standard @BBL X2D`) instead of its bridge speed
  (50 mm/s): on a test of spans from 20 to 200 mm, the strands sagged at 50 mm/s while the walls
  beside them, at 10 mm/s, stayed straight. The bridge speed still caps them. A bridge layer
  takes longer: that test's, 47 min instead of 16.

## 0.17.3

### Fixed

- The nozzle and the bed now take each layer's temperature. The first layer's held for the whole
  print, as nothing set the others: PETG Translucent printed at 250 °C instead of 245 °C.
  The second layer sets the filament's and the plate's other-layer temperatures (`M104`,
  `M140`), and a filament change waits for the new filament's own (`M109`), where the X2D's
  change left the nozzle at the 235 °C its cut cooled it to for the rest of the print.
- A machine description whose nozzle temperatures lie outside the filament's own range is
  refused instead of printed.

## 0.17.2

### Changed

- The README points to SolidCraft's own `main` to look at the stages' `.ply` files, now that it
  draws their edges, faces and named numbers, instead of a branch of a fork.

## 0.17.1

### Changed

- Compiling is much faster again. The Benchy takes 29 s with `--flat` (45 s at 0.12.0, 479 s at
  0.16.1, same machine, same 3606 mm of filament) and 103 s with bent layers (about 8 min at
  0.17.0, same 3578 mm), and no longer spends its time offsetting. A disc offset now goes in
  stages of 0.25 mm, whose erosions add up, so that what is thin dies early instead of its edges
  all crossing; a curve's gentle turns keep one vertex each instead of doubling at every stage;
  and a band's insets each go on from the one before.
- An offset stays within a micrometre of the exact one on both sides, no longer only inside it.

### Fixed

- A `.ply` of the sections, regions or paths draws a bent layer on its bend, as the program's did:
  sections and regions halfway between the layer's top and the one below, beads half their height
  under the top. The README's pictures of the Benchy now show it with its bent layers.
- A band of a dense area could keep a needle a nanometre thin along the area's edge, up to 27 mm
  long, joining two bands into one and changing which beads filled them with the part's rotation.

## 0.17.0

### Fixed

- The 3DBenchy compiles with bent layers again, as any mesh whose top facets overlap on the plate
  or whose outlines run close together does. Since 0.15.0 such a mesh could stop with `a warp is
  not zero along its triangles' outer edges`: the leaves' triangles left gaps up to 13 mm long and
  a micrometre wide between neighbouring pieces, which simplified their shared edges each on its
  own, and overlapped elsewhere. The Benchy lays 3578 mm of filament in 37.8 min of moves (3606 mm
  and 37.6 min with `--flat`); its compile takes 8 min and 2.1 GB.

### Changed

- All the facets' edges now make one planar subdivision, computed in one pass, so that neighbours
  share every corner. Each chain of edges between two junctions is walked once in its fewest
  straight pieces, within a micrometre: Boolean operations move corners up to half of one, which
  a tolerance of a few nanometres mistook for corners of the part.
- Which Delaunay triangles lie on one circle follows from the in-circle determinant's own
  gradient, not a fixed distance: on the test parts, such corners and others lie a thousand times
  apart by this measure.

### Removed

- `layercraft_geom::triangulate_evenly` takes an outline, lines and what to fill, not cells, and
  `Triangles` says how many triangles lie in filled faces and a point inside each face.

## 0.16.2

### Documentation

- The README is rewritten around a table: each thing that sets layercraft apart, beside a
  screenshot of SolidCraft showing the `.ply` files layercraft wrote.

## 0.16.1

### Changed

- A top surface shows no joins it can avoid. There an inset now costs its bending energy,
  `(s/2)∫κ²`, about `π/2` at each corner, and a line's end against the wall a quarter of an end,
  up to a whole one on a bead as short as it is wide. A square or notched top about the crossing
  width, which lay in insets with a diagonal join at each corner, now lies in one family of
  lines. A Scrabble board's cells do, and the rings round their inlays; its ribs keep their insets.
  Solid infill and the first layer are laid as before.

## 0.16.0

### Changed

- A stage file holds its print's triangles once. The section, region, path and program stages
  hold one `carrier`, its `points` and `triangles`: those of the triangulation where some layer
  is bent. Each bent layer's `warp` is its value at each of the carrier's corners, a plain list;
  it held triangles of its own (0.15.0). The ramps' section stage takes 11 MB instead of 30
  (6.8 in 0.14.2), and a wavy plate 100 mm square's 66 MB instead of 176 (45). What is left is
  the values: one at every corner of the carrier for each bent layer, zero where that layer is
  flat. The section stage takes as long and holds as much as before, 289 MB on the wavy plate,
  most of it while the leaves are found.
- Older stage files with bent layers are refused, to be made again; those whose layers are all
  flat still load.
- `merge` joins the programs' carriers. A merged layer is bent where any of its pieces is, their
  warps added on the carriers joined, where their triangles are the same or lie apart on the
  plate. A program whose triangles overlap another's otherwise keeps its bends out, and its beads
  then fail verification.
- `verify` reports a layer whose warp does not fit its program's carrier, and checks its beads as
  on a flat layer.

### Added

- `layercraft_geom::Warp`: a layer's value at each corner of a carrier, flat when it holds none.
  `Carrier::relief` makes it a `Relief`; `Carrier::trim` keeps the triangles where some warp is
  not zero, with the warps on them; `Carrier::union` joins two carriers and tells where the
  other's corners went; `Carrier::len` counts the corners.
- `Sections`, `Regions`, `Paths` and `Program` gain `carrier`, and their layers' `warp` is a
  `Warp`. `MotionError::Geometry` reports a warp that does not fit its carrier.

### Removed

- `Relief::combine`: warps on one carrier add value by value.

## 0.15.0

### Changed

- Bent layers bend on triangles of the part's own, not on a grid laid on the plate. One
  triangulation per print holds the edges and corners of every top facet a leaf may follow and
  reaches as far past them as a leaf may bend; between them it is cut evenly, a cell (0.25 mm on
  the X2D) wide, from an equilateral lattice along each piece's own axes. Every leaf is linear on
  it, so a followed top is met exactly, its creases and ridges too, which the grid's triangles
  cut across: a wedge and a gable lie within a nanometre of their leaves. A part turned, moved or
  mirrored on the plate has its triangles, and its leaves, turned, moved or mirrored with it, to
  0.01 µm: runs of outline points along one line are merged, and ties between equally good
  triangles are broken by the shape itself, corners on one circle fanned from their centre,
  not by the order of the points.
- The mesh is cut along the leaves' triangles by an exact walk: each of its edges is traced
  through them with exact orientation tests, and each of its triangles cut into its parts over
  theirs from those traces alone, so the cut mesh is closed by construction. A triangle seen
  edge-on, its shadow a nanometre wide, is fanned through the points of its edges.
- The face's bounds between two leaves come from the triangles themselves. To a corner `l` off,
  the leaf below may fall `h − t + t·l²/R²` from the one above, `t` the thinnest slab and
  `R² = ρ² + S_v + S_w` the face's reach `ρ` widened by the spreads of the triangles at both
  corners, as the exact mean of the squared distances between two triangles' corners gives it;
  the bound was piecewise linear in `l`, its reach found by bisection on the grid (0.80 mm).
  Under a steep followed top the slabs thicken only as much as these pairs ask. The ironing
  holds a leaf to its depth over `R` per mm between any two corners under the face, and its
  slope to its depth over the coarsest triangles' `R`; it held the slope to its depth over the
  longest path along the grid within the face's reach.
- A top surface on a bent layer is hatched along the fall line of the triangles under it, each
  weighed by its area, where the slope was sampled at points.
- A layer's `warp` in the stage files holds `points`, `triangles` and `values`: the triangles
  where it is not zero, their corners and its value at each. Older files' blocks on a grid read
  as those squares split by their diagonals. Each bent layer holding its own triangles, the files
  grow: the ramps' section stage takes 30 MB instead of 6.8.
- The section stage takes about as long as before, 1.8 s against 1.9 on the ramps, run side by
  side, and 22 s against 25 on a wavy plate 100 mm square, where it holds 289 MB against 169.

### Added

- `layercraft_geom::Carrier`: triangles over the plate that any number of reliefs share
  (`Carrier::new`, `Carrier::relief`), each relief holding only its value at every corner.
  `Relief::new` and `Relief::from_grid` make a relief on a carrier of its own; `facets` and
  `facets_over` give its triangles and their slopes, `values` its values at its carrier's
  corners.
- `layercraft_geom::triangulate_evenly` (`Triangles`): cells of the plate, sharing edges, cut
  into even triangles that meet; `Buckets`, which find the triangles near a point or along a
  segment; and `orientation`, the exact side of a line a point lies on.

### Removed

- `layercraft_geom::Grid`, `Lines`, `Crossing`, `MAX_VALUES` and `levels`: reliefs are on
  triangles of their own.

## 0.14.2

### Fixed

- No more strings across infill: every travel longer than the retraction's minimum retracts and
  lifts, wherever it runs. Moves under the layer above used to go on with the filament under
  pressure, up to 118 mm at a time on a Scrabble board quarter, on the grounds that what oozed
  would be buried; it drew fine strings across the open cells of the sparse infill.
- The nozzle no longer strikes beads it has just laid: a travel that crosses a bead's centre line
  at its own height lifts over it, however short, and the tours avoid such moves first.

### Changed

- The path stage no longer marks moves as staying inside the part (`travel_inside`); it marks the
  moves that cross a laid bead (`travel_crosses`). Length decides retraction in the motion stage.
  
## 0.14.1

### Changed

- A layer's beads are put in order in a fraction of the time, with no more retractions and less
  travel. The search still tries every reversal of a run of beads and every move of up to three
  of them elsewhere, either way round, and now goes on until none shortens the tour, where it
  gave up after 32 passes; but each end of a bead looks only at the ends nearer than its own
  move, and, where that move retracts, at those a hidden move from it reaches, which misses no
  change that shortens the tour. On the board 120 mm square under 16 + 16 ridges, the paths
  stage takes 4 s instead of 169 s, run alongside, its moves retracting 1,651 times instead of
  1,654, over 21.9 m of travel instead of 29.4 m.

## 0.14.0

### Changed

- A lift's helix turns the way the nozzle turns from its last move to the travel it sets off
  into (to the left where it goes straight on or back), so a mirrored part's lifts are
  mirrored; the nozzle's first heading is set by its first move, not by the plate's x axis.
- A layer too short to cool waits at the point nearest the nozzle that lies the clearance from
  every bead, by the hull of its beads, not 10 mm past their x/y box along x or y: turned,
  moved or mirrored with the part, ties going the way the nozzle was heading.
- Convection rolls are made for each solid, over its own sparse areas and in its own frame,
  not one set over the whole plate: a part's rolls no longer change with what else stands on
  the plate, and two parts do not share a grid.
- Seams follow the part, not the plate: a wall loop starts at the concave corner nearest the
  seam of its solid on the layer below, else at the vertex nearest it; on a solid's first layer,
  at the point of its outline farthest along its frame's axis. Holes, inner walls and closed
  beads follow the island's outer seam, and the loops of a mirrored part run the other way. The
  next island is the one whose first bead starts nearest the nozzle, and equally long sweeps are
  ordered by their ends' distance to it, not by plate angle. A solid's islands on one layer
  follow the seams of the layer below, not each other's, each the nearest to its outline; the
  corners of a small round hole, all alike deep, are all candidates for the seam.
- Sparse lines, bridge strands, support lines and the region stage's roofs take their direction
  and phase from the part's own frame (a roof's piece, its own axis and centre), not the plate's
  axes and origin: a part turned, moved or mirrored on the plate has them turned, moved or
  mirrored with it. `bridge_angle` counts its extra 15° candidates from the preferred direction
  and takes the phase of its strands' grid.
- What a part's frame settles is settled by its solid's: what holds together, its islands
  touching on one layer and from each layer to the next, all its layers weighing at once, so it
  stays the same from layer to layer and changes with nothing else on the plate. A frame's
  second axis is the quarter turn its areas skew towards, so a mirrored part has a mirrored
  frame: the dense lines' ties and the line of a symmetric piece of gap fill follow it. Two
  ways along a line tie only where moving the boundary a micrometre could swap them, weighed by
  how far the boundary spreads along that line, so a part that skews only slightly, as two
  posts of unequal depth under a beam, still has its frame mirrored with it.

## 0.13.0

### Changed

- Dense areas are laid alike whichever way the part lies on the plate: every direction their
  beads take comes from the part. Each piece of a top surface or of solid infill takes the
  family of curves that leaves the fewest bead ends, at its edge and against its neighbours,
  and lays least along the dense beads of the layer below: a band narrower than its beads'
  crossing width `W* = v²/a`, across which a bead never reaches its speed (20 mm for the X2D's
  top surface, 6.25 mm inside), in insets of its edge; a wider piece in straight lines along or
  across its own edges, its neighbours' or the beads below it, crossing the layer below. Lines
  of one direction run on from piece to piece, placed by their part. On a Scrabble board each
  ridge was hatched at its own piece's angle, three lines along the ridges one way and dots
  across them the other; every ridge is now laid alike. This replaces the angle chosen for each
  piece among the layer's and its four longest edges' (0.4.10).
- What the insets leave, and any strip no longer than twice as thick as it is, takes one bead
  through its centroid along its own line: its axis of greatest spread or, where it spreads
  alike every way as a square's middle does, the side nearest its part's frame. A square's
  middle was laid in bits whose direction the lattice's rounding chose.
- A part's frame, which turns the convection rolls and settles the dense lines' ties, takes its
  edges' own lines where its areas spread alike every way, as a square's do, instead of the
  plate's axes, and of those the way its areas skew the most.
- On a board 120 mm square and 2 mm thick under 16 + 16 ridges 1.2 mm wide, the paths stage lays
  17,581 beads instead of 25,813 for 301.4 m of bead (303.6 m), in 154 s instead of 216 s run
  side by side: the tour of the plate's top layer, quadratic in its beads, takes nearly all of it.
- `PathParams` gains `crossing` (`Crossing { top, inside, first_layer }`), read from the
  process's speeds and accelerations; `resolution` no longer chooses hatch angles.
  `layercraft_geom::hatch` takes the phase of its lines, and `chordal_axis` the axis of the
  part's frame at a point (`axis_at`), which settles the line of a symmetric piece.

### Added

- `layercraft_geom::disc_offset` and `disc_opening`: a region grown or shrunk with round
  corners, its Minkowski sum with a disc, each arc cut into equal chords within
  `RESOLUTION_MM` of it, the same chords however the region is turned or mirrored.
- `layercraft_geom::Moments`: a region's area, centroid, own lines (`Axes`) and skew, from its
  moments; a moment vanishes where moving the boundary by `RESOLUTION_MM` could cancel it.

### Fixed

- `clip_lines` keeps a ring in one piece through its start.
- The chordal axis turns with its region: triangles whose four corners lie on one circle, as
  along an arc, make one cell, where the lattice's rounding chose the chord between them.

## 0.12.0

### Added

- Several colours on one plate. Each colour the 3MF gives its objects (base materials or colour
  groups, parts of one object included) is a material, printed with its own AMS filament: a
  layer is printed in one pass per material, each walled alone but judged against the whole
  layer (what another material covers is no top surface), and the X2D's own filament change runs
  between passes of different materials, a layer starting with the material the one before
  ended with. The flush grows with how much lighter the new colour is than the old. The plate's
  summary lists every filament, all fed to the first extruder; `--colour` takes one colour per
  material, the 3MF's by default. Merging programs in several materials is not supported yet.

### Changed

- A bare `[name]` of a per-extruder list in the machine's templates prints the item of the
  extruder in use, as the maker's templates expect, not the whole list.

## 0.11.2

### Fixed

- Supports came off only by force: their top stood where the roof's first strands, laid as a
  bridge and hanging a strand's width below the nozzle, land, so the roof was laid hot straight
  onto the interface. The gap `support_top_z_distance` is now air under the lowest strand: with
  0.4 mm strands on 0.2 mm layers, the columns end one layer lower.

## 0.11.1

### Fixed

- A mesh with cracks, where two faces meet along a seam cut into different edges (SolidCraft's
  exports along circles), no longer loses loops: the section chains that stop at a crack are
  joined end to start, the shortest gaps first, instead of being dropped. A layer crossing such a
  seam kept its outline and filled its holes; the clip's screw hole was closed on one layer and
  got supports. The warning names how wide the widest crack closed was.

## 0.11.0

### Added

- Supports. `--supports on` (on `compile` and `view`; by default the process's `enable_support`)
  stands columns under what nothing else holds up. A bead stands while its centre line lies over
  the layer below, so a layer may reach half a bead past it; beyond, it hangs. Strands are laid
  across each hanging piece the way a bridge's are: one held at both ends and no longer than
  the process's `max_bridge_length` bridges (any length with `bridge_no_support`; with the X2D's
  0, every strand is held up), and the band of any other is a roof. Under each roof a column
  stands down to the plate, or onto the part (only the plate with
  `support_on_build_plate_only`), its top `support_top_z_distance` under the roof, which is
  where a strand round from the nozzle, hanging below its layer, lands; `support_bottom_z_distance`
  over the part it stands on, and `support_object_xy_distance` beside it. The columns' top
  `support_interface_top_layers` are their interface. The body is laid in lines
  `support_base_pattern_spacing` apart past their tiling spacing, the same way on every layer,
  so each stands on the one below; the interface across them, `support_interface_spacing` apart;
  both `support_line_width` wide, at `support_speed` and `support_interface_speed`. The regions
  stage holds them as `supports` and `support_interface`, the paths and program stages as the
  roles `support` and `support interface` (codes 10 and 11 in the pictures), and the G-code
  names them as Bambu Studio does. Over the supports the part's first layer is laid as a bridge.
- `compare` and `merge` read Bambu Studio's G-code with supports: its `Support` and
  `Support interface` lines were unknown features.

### Changed

- The regions stage's `support`, the section of the layer below, is now `below`; files with
  `support` still read.

## 0.10.0

### Added

- Free layers. A program's layer may be `"free": true`: its beads follow no top, and each rests
  on the plate or on the beads of the layers before it. This is how a strand climbs over another:
  a plait, a weave, a bead laid across a bead. `z` is the highest the nozzle goes in it, and
  everything else of a layer stays. `verify` holds a free bead to what it rests on, at the end of
  each move and halfway: its tip one bead's height over it, or up to two where the bead is
  stretched across a gap. A free bead through a bead already laid, or hanging higher, is refused,
  and the beads that may be rested on are kept longer. The nozzle's clearance is the same as
  ever. Free layers are written by hand or by an agent as a program stage (`--from program=`);
  nothing in the chain makes them yet.

Releases 0.1.0 to 0.9.0, the pipeline built stage by stage, are in
[docs/changelog-0.1-to-0.9.md](docs/changelog-0.1-to-0.9.md).
