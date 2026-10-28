# Plan: split the framework's mixed files in place (ADR 0008, step 1)

Issue: #58. Status: in progress. PR 1a done; 1b and 1c to come.

## Goal

Step 1 of [ADR 0008](../../adr/0008-anatomy-mission-framework.md)'s migration: **split the mixed files of `branes::racing` in place**, along the lines of the [framework boundary audit](../../assessments/framework-boundary.md). After this step every file is either mission-neutral or racing-only, and the neutral ones include nothing racing. Step 2 is then a move: copy the neutral files to `anatomy` and rename the namespace.

There is **no behavior change**:
- the namespace stays `branes::racing`;
- the include paths of the files that don't split stay the same;
- the command lines stay the same;
- every log, replay and result stays byte-identical.

## Acceptance (stated before any code)

For each PR, and for the step as a whole:

1. **Tests:** all tests and gates pass in the CI matrix (GCC and Clang, Debug and Release, MSVC), as on main.
2. **Byte-identity.** Each command below is run on `main` (before) and on the branch (after), with the same build preset (`gcc-release`). Their output files compare equal byte for byte (`cmp`):
   - `sitl_null --scenario scenarios/null-smoke.yaml`, `sitl_scripted-hop --scenario scenarios/scripted-hop.yaml`, `sitl_b0-gt-pid --scenario scenarios/b0-determinism.yaml`: the `.brlog` files;
   - `replay_<name> --verify` of each of those logs: the replay `.brlog` files;
   - `sitl_batch_b0-gt-pid --scenario scenarios/b0-nominal.yaml --seeds 20`: stdout;
   - `precision_sweep_b0-gt-pid --scenario scenarios/b0-nominal.yaml --seeds 20 --out-dir <dir>`: every file but the wall-time column, which is off by default;
   - `log_dump` of the B0 log: every CSV;
   - `score_log` of the B0 log: stdout.

   The script that runs these is committed as `tools/compare_outputs.sh`, so the check can be repeated in step 2.
3. **Boundary.** A new ctest, `boundary.neutral_includes`, reads a manifest (`cmake/framework_boundary.cmake`) that lists every neutral file. It fails if a neutral file includes a racing-only header. Every file the audit classes as neutral, or as the neutral half of a split, is in the manifest.
4. **Public interface.** No harness command line, YAML key, log channel id, or log header field changes. Mission-supplied parts keep their racing values: `course:` in scenarios, channel 5 = `oracle_gates`, the `racing_` target names.

## Design

Each new stateful type (the channel table, the generic `SlotContext`, the generic `Runner`) goes through the model-invariant workflow (model, invariants, contracts) in its PR description before its code.

### PR 1a: contracts, log, codec, config

| Change | Files |
|---|---|
| `Geofence`, `StartPose` (and `contains`, `apply`) move to a new neutral `msg/region.hpp`; `course.hpp` includes it | `msg/course.hpp`, new `msg/region.hpp` |
| `IntruderDetection(s)` move to a new neutral `msg/intruders.hpp`; `perception.hpp` keeps the gate observations | `msg/perception.hpp`, new `msg/intruders.hpp` |
| The `GateObservations` codec moves to a racing `msg/gate_codec.hpp` | `msg/codec.hpp`, new `msg/gate_codec.hpp` |
| `ReferenceSample::progress` documented as the reference's path parameter | `msg/control.hpp` |
| **Log channels.** The framework's ids become a fixed table (1–4 and 6 inputs and sim; 16–18 outputs). A mission supplies a table of its own channels (id, name, class), and `LogReader`'s known-channel check and `is_sim`/`is_output` consult both. Racing registers 5 = `oracle_gates` (sim). The on-disk format does not change. | `log/log.hpp`, `src/log/log.cpp`, new racing `log/channels.hpp` |
| **Config.** `config.hpp` splits into neutral vehicle, baseline and scenario configs and a racing course config. The scenario's reference to its world is generic; the mission names its YAML key (`course:`) and parses its world-specific simulation parameters (`oracle_corner_sigma_px`). Emitted YAML stays byte-identical. The slot list (`kSlotNames`) moves to racing. | `config/config.hpp`, `src/config/config.cpp`, new racing `config/course.hpp` |

### PR 1b: slots, context, stack, harness

| Change | Files |
|---|---|
| `slots.hpp` splits into the neutral concepts (`Controller`, `SafetyFilter`, `DaaTracker`, `FcBridge`) and the racing ones (`Estimator` with gates, `GateDetector`, `Planner`) | `slots/slots.hpp`, new `slots/neutral.hpp` |
| `SlotContext` becomes `SlotContext<World>`; racing uses `SlotContext<CourseMap<double>>` under its current name | `slots/context.hpp` |
| The nulls split the same way; `GroundTruthEstimator`'s truth pass-through becomes neutral, with a racing adapter for the gate argument | `slots/null.hpp`, `slots/ground_truth.hpp` |
| **The harness boundary.** A neutral concept states what `Runner` needs from a stack (`on_imu`, `on_frame`, `on_tick`, `on_truth`, the outputs, its scalar type). `Runner` becomes generic over it; mission channels are decoded through a mission hook. `Stack` and `make_stack` stay racing. | `harness/harness.hpp`, `compose/stack.hpp` |

### PR 1c: simulator, batch, sweep types, build, apps

| Change | Files |
|---|---|
| **The simulator** splits into a neutral core (dynamics, FC, IMU, faults, camera timing, truth, `sim_config`) and a mission world (its start pose and its oracle sensors). Racing's world is the course and the gate projector. Event order and stream seeds are unchanged. | `sim/simulator.hpp`, `sim/vehicle.hpp` |
| **`batch.hpp`** becomes generic over the world and the scorer | `sim/batch.hpp` |
| The sweep type list stays neutral; `in_plane_ulp(gate)` moves to a racing header | `sim/sweep_types.hpp` |
| **The baseline generator** takes the slot list and the target prefix from the mission; racing passes its seven slots and `racing_` | `cmake/baselines.cmake`, `cmake/slot_registry.cmake` |
| **The harness mains** (`sitl`, `replay`, `sitl_batch`, `flight_stack`, `log_dump`): their mission-neutral logic moves into neutral library functions, and the mains become thin racing callers. `score` and `precision_sweep` stay racing. | `apps/*`, new neutral `harness/apps.hpp` |

Each PR updates the manifest and the tests that move with its files.

## Risks

- **Emitted YAML and event order** are where byte-identity breaks most easily: the `sim_config` record and the simulator's event interleaving. The comparison script covers both, through the logs.
- **Template genericity** (`SlotContext<World>`, `Runner<S>`, the simulator's world) can raise compile times and MSVC warnings (C4459, C4702, C4127). CI's MSVC jobs check them.
- **Scope.** This step only splits. Renaming, reorganizing directories and changing namespaces belong to step 2.

## Results

### PR 1a

**What changed:**
- **Contracts:**
  - `Geofence` and `StartPose` moved to `msg/region.hpp`.
  - `IntruderDetection(s)` moved to `msg/intruders.hpp`.
  - The gate codec moved to `msg/gate_codec.hpp`.
  - `ReferenceSample::progress` is documented as the reference's path parameter.
- **Log:** `Channel` holds the framework's ids only (1–4, 6, 16–18). Racing's `oracle_gates` (5) is in `log/channels.hpp` with `kRacingChannels`. `LogReader::open` takes the mission's channels; its new invariant I2 is that they contain no framework id and no repeat, and `open` returns an error if it fails. The unused `is_sim` is removed, and `is_framework` added.
- **Config:** `config.hpp` split into neutral `error.hpp`, `vehicle.hpp`, `scenario.hpp` and `baseline.hpp`, and a racing `course.hpp`. The sources split the same way, with the YAML helpers in the private `sdk/src/config/parse.hpp`.
  - The scenario loader takes a `ScenarioSchema`: the key of the mission's world file, and its own `sim:` parameters, each a number with a fallback and a minimum. `ScenarioConfig` names the world `world_ref`/`world_path` and holds the mission's parameters in `world_sim`, in schema order.
  - The baseline loader and `composition_hash` take the mission's slot list.
  - `config.hpp` became racing's aggregate: `kRacingScenario` (`course:`, `oracle_corner_sigma_px`), `kSlotNames`, and one-argument racing loaders, so racing call sites barely change.
  - `Simulator::create` returns an error for a scenario without racing's parameters, instead of reading past the end.
- **Boundary:** `cmake/framework_boundary.cmake` lists 29 neutral files, and the ctest `boundary.neutral_includes` checks them. `boundary.detects_violation` checks that the check fails on a manifest that lists the racing gate codec as neutral.
- **New tests:**
  - a mission channel reads only when named, and I2 is enforced;
  - racing's oracle noise parses, is checked, and round-trips;
  - the scenario and baseline loaders take another mission's schema and slot list.

**Acceptance:**
1. **Tests:** 208 of 208 pass on `gcc-debug` and on `clang-debug` (201 on main, plus 7 new). CI covers Release and MSVC.
2. **Byte-identity:** `tools/compare_outputs.sh` with `gcc-release`, on main and on the branch. All 39 output files are identical: 3 SITL logs, 3 replays, `score_log`, 12 `log_dump` CSV and YAML files, a 20-seed `sitl_batch`, a 20-seed precision sweep (11 files), and the harnesses' stdout.

   The first comparison differed only in the stdout lines that print the output directory, which differs between runs. The script now writes that directory as `<out>`, and with it every file compares equal.
3. **Boundary:** `boundary.neutral_includes` passes, and `boundary.detects_violation` passes.
4. **Interface:** no command line, YAML key, channel id, or on-disk header field changed. clang-format 18 (CI's version) finds nothing in the 41 changed C++ files.

**Departures:**
- **A seam the audit missed.** `LogHeader`'s `course_hash` and `course_yaml` were racing names in a neutral type. They are now `world_hash` and `world_yaml`. The on-disk fields and their order are unchanged, and `log_dump` still writes `course_hash:` and `course.yaml`.
- **The YAML helpers.** The audit didn't list them separately. They are neutral, but private to the loaders. In step 2, a racing course loader outside anatomy will need them, so anatomy will have to export them or racing will keep a copy. That choice belongs to step 2.

