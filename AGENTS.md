# layercraft — instructions for agents

layercraft is an open-source, pure-Rust compiler from a solid to a print on a Bambu Lab X2D:
a 3MF mesh goes through layer sections, regions, toolpaths and motion down to the `.gcode.3mf`
the printer runs. Read [`SOUL.md`](SOUL.md) first; it holds the mission and outranks this file.
Sibling: [`../solidcraft`](https://github.com/tangency-labs/solidcraft) (the CAD program whose
3MF export is the first input, and the viewer for the toolpaths). The repository is
github.com/mariogeiger/layercraft.

## Never harm the machine
A wrong G-code line can drive the nozzle into the bed, overheat a part or ruin a print. **This
outranks feature work.**
- **Every emitted program passes `verify`.** No G-code leaves the compiler unless the verify
  stage accepts it: every position inside the build volume, the nozzle never below the layer
  being printed, temperatures and speeds within the machine's limits, extrusion finite and
  consistent with the volume it fills. A program that fails verify is an error, never a warning.
- **Sending and starting are two acts.** An agent may read the printer's state and copy a verified
  file to its storage. It starts, stops, pauses or changes a print **only when the owner
  explicitly says so in chat, for that print**. Never touch the printer's firmware or settings.
- **The LAN access code stays in its file.** Pass it with `--code-file ~/bambu.txt`; never print
  it in chat, logs, test output or commits.

## Never crash
- **No panics in non-test code:** no `unwrap()`, `expect()`, `panic!`, `unreachable!`, `todo!`,
  `unimplemented!`; no `unsafe` (`unsafe_code = "forbid"`). Every crate root carries
  `#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]`.
- **Errors are `Result<T, E>`** through the crate's error type and `?`. An unfinished feature
  returns an error ("not supported yet: …").
- **Input is hostile:** 3MF files, stage files read with `--from`, machine descriptions and the
  printer's replies. Use `get()` not `[i]`, check floats are finite, cap sizes and loop counts.
- **Prove it:** every crash or verify fix lands with a regression test in the existing test file
  for its stage, never in a new file named after the bug.

## Non-negotiables
- **Our own code.** layercraft is MIT OR Apache-2.0 and written from public specifications
  (3MF, G-code references, the printer's documented behaviour) and mathematics.
  **Never read, copy or link the sources of Bambu Studio, OrcaSlicer, PrusaSlicer, Slic3r,
  Cura or any other GPL/AGPL slicer** — including the clone in `~/git/OrcaSlicer`. Never
  disassemble the printer's firmware.
- **Bambu Studio is a black box.** You may run it, slice with it and read the files it writes,
  to compare results and to learn what the firmware expects of a `.gcode.3mf`. Its output is
  never committed. Its installed profiles are read only by `layercraft machine import`, which
  writes the machine description to `~/.config/layercraft/`, outside the repository; never copy
  its start or end G-code, presets or text into the repository.
- **Every stage is open.** Each stage reads and writes a readable file (`--emit <stage>`,
  `--from <file>`), so an agent can inspect or edit any stage and compile on from it. A stage
  depends only on the stages before it; nothing reaches around the chain.
- **Exact where exactness gives truth.** A layer is the exact section of the mesh; filament
  length follows from the cross-section of the bead and the length it covers. Name a magic
  number, derive it, or take it from the machine description.
- **Reference tests use analytic values.** Solids generated in code (cube, cylinder, rounded
  block, text) with known volumes, areas and section shapes. Bambu Studio's numbers are a
  comparison, not an oracle: report the difference, don't assert equality with it.
- **Private designs stay private.** The owner's designs (the pottery stamp in `~/Desktop/tampon/`
  carries his handwritten signature) are dogfood: tests that read them skip when the file is
  absent, and they are never committed, published or attached.
- **Quality gates** before every commit, through the pre-commit hooks: `cargo fmt --check`,
  `cargo clippy --all-targets -- -D warnings`, `cargo test`, and
  `scripts/check-file-length-and-header` (each file under 1,000 lines, with a header of at most
  20 lines stating its mission).

## Landing changes
- A single-file change may go straight to `main`. A multi-file change goes on a branch in a
  worktree: commit there with the hooks, merge onto `main`, then remove the branch and worktree.
- Bump the version in every push (0.x: a breaking change bumps the second number, anything else
  the third) and add the user-visible change to `CHANGELOG.md`.
- Push only with the owner's authority. Never commit build directories, sliced output or the
  owner's designs; check `git status` and `git ls-files` before pushing.
- Commit no design documents. The reason for a change goes in its commit message.

## Running and looking
- `layercraft compile part.3mf --out part.gcode.3mf` compiles; `--emit <stage>` writes a stage
  file, `--from <file>` resumes from one.
- **Experiment by place, not by script.** To compare settings, print several copies on one plate
  and vary them with `--tune` (boxes by object or height band, any region, path or motion key)
  rather than editing stage files by hand; edit a stage file only for what no setting reaches.
- **Look at any stage.** `--emit <stage>=FILE.ply` writes it as a PLY picture: points,
  segments and triangles with properties to colour and filter by (`layercraft_view`).
- **Look at the toolpaths in SolidCraft.** `layercraft view` writes a SolidCraft script (one sketch per layer,
  one layer per role); run it with `solidcraft-cli` or in the open SolidCraft window and take a
  screenshot (`../solidcraft/AGENTS.md`, “Running and looking at the app”).
- **Compare with Bambu Studio** headless, in a throwaway data folder so the owner's settings
  stay untouched (`--datadir "$(mktemp -d)"`), and compare layer count, filament length,
  extruded volume per layer and estimated time.
