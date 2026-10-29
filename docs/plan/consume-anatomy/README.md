---
status: complete
issue: branes-ai/racing-drone#65
---
# Plan: racing-drone consumes anatomy (ADR 0008, step 2)

Issue: #65. Implemented in 2c (#66). Anatomy's side: branes-ai/anatomy#1 (2a, the skeleton) and #3 (2b, the move), released as anatomy v0.1.0.

**Since anatomy v0.3.0** ([anatomy#10](https://github.com/branes-ai/anatomy/issues/10)), the mobility headers are their own library, `branes::anatomy_mobility`, with a fifth bridge, `anatomy_mobility.hpp`. `branes::anatomy_sim` is now `branes::anatomy_mobility_sim`. This plan records the v0.1.0 state.

## Goal

Step 2 of [ADR 0008](../../adr/0008-anatomy-mission-framework.md): the framework's neutral half moved to [anatomy](https://github.com/branes-ai/anatomy). racing-drone now fetches it and keeps only its racing part:
- the course and the gate observations;
- racing's slots and its `Stack`;
- its configs and its channel;
- the gate projector, its `Simulator`, the scoreboard, and the batch run.

What stays the same:
- **Behavior:** byte-identical.
- **Formats and interfaces:** the log format, YAML files, and command lines are unchanged.

## Acceptance (from #65, stated before any code)

4. Every test and gate passes in CI. The boundary manifest and its ctest are removed, because the boundary is now a repo boundary.
5. **Byte-identity:** `tools/compare_outputs.sh` with `gcc-release` gives all 39 files identical to main's before the split (f85ff73). `flight_stack`'s output is unchanged for the same cases as PR 1d.
6. **Log compatibility:** a log written before step 2, by `sitl_b0-gt-pid` at f85ff73, replays with `--verify` under the new `replay_b0-gt-pid`, and its `log_dump` output is byte-identical.
7. No file listed in the manifest remains in racing-drone. racing-drone fetches anatomy pinned to a commit.

## Design

- **The dependency.** `cmake/deps.cmake` fetches anatomy at its v0.1.0 commit. It is declared after racing-drone's own yaml-cpp, reflex and Universal, so where both repos name a dependency, racing-drone's declaration is the one used. A configure-time check fails if anatomy pins a different reflex commit.
- **The bridges.** `sdk/include/branes/racing/anatomy.hpp`, `anatomy_glue.hpp`, `anatomy_sim.hpp` and `anatomy_sweep.hpp` bring each namespace-scope name of one anatomy library into the matching `branes::racing` namespace.
  - Each name gets an explicit using-declaration. Unscoped enums also get `using enum`, for their enumerators, and namespace aliases are repeated.
  - They are **generated** by `tools/gen_anatomy_bridges.py` from Clang's AST of anatomy's headers, at the fetched commit, so the lists are complete. Rerun it after an anatomy bump.
  - Racing code includes the bridge in place of each moved header, and otherwise reads as before.
- **One bridge per library,** not one header. The plan had a single header. But a single header would make racing-drone's core library include the simulator and the sweep types, and so need reflex and Universal. One bridge per anatomy library keeps each racing target's dependencies as they were.
- **Targets:**
  - `branes::racing` now compiles only the course loader, and links `branes::anatomy`.
  - `branes::racing_glue` is gone; the baselines link `branes::anatomy_glue`.
  - `branes::racing_sim` links `branes::anatomy_sim`.
  - The baseline generator includes anatomy's `composition.cmake` through `ANATOMY_CMAKE_DIR`.
  - The slot registry names anatomy's headers for `ScriptedHopController` and `CascadedPid`.
- **The YAML helpers.** Anatomy does not export its private YAML parsing helpers, and racing's course loader needs them. racing-drone keeps a copy as `sdk/src/config/yaml_parse.hpp`; that was the choice left open in step 1.
- **Tests:**
  - racing-drone keeps the racing cases of its mixed tests: the gate codec, the course and racing's scenario and baseline files, racing's channel, the gate and course contracts, the gate projector, racing's `Simulator`, and the gate-observation cast.
  - The neutral cases, `util`, the compile-fail test, the neutral-mission tests, and the boundary check moved to anatomy, or were already there.
  - One test is new: `Simulator::create` rejects a scenario without racing's sim parameters.

## Results

**Acceptance:**
1. **Tests:** all pass on `gcc-debug` and on `clang-debug`. Anatomy runs 102 tests and racing-drone 132, so 234 across the two repos; racing-drone alone had 217 before the move.
2. **Byte-identity:** all 39 files are identical to main's before the split: 3 SITL logs and 3 replays, `score_log`, 12 `log_dump` files, a 20-seed `sitl_batch`, a 20-seed precision sweep (11 files), and the harnesses' stdout. `flight_stack` gives the same output and exit code for `null` and `b0-gt-pid` at 10 cycles, 0 cycles, and an overflowing cycle count.
3. **Log compatibility:** the pre-split B0 log replays with `--verify` (6000 outputs). The replay log it writes is byte-identical to the pre-split replay log, and `log_dump` of it gives byte-identical files.
4. **Nothing left behind:** none of the 53 moved files remains. The three build helpers listed in the manifest (`cmake/compile_all.cmake`, `warnings.cmake`, `deps.cmake`) were never meant to move: anatomy has its own.
5. **Lint:** clang-format 18 (CI's version) is clean on the 45 changed C++ files.

**Departures:**
- **Four bridge headers, generated,** rather than one hand-written header (above).
- **The YAML helpers are a copy** in racing-drone (above).
- **The bad-magic error** now reads "not a log (bad magic)" (anatomy 2b). Its test moved to anatomy.
