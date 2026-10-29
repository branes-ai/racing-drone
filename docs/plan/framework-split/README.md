---
status: complete
issue: branes-ai/racing-drone#58
---
# Plan: split the framework's mixed files in place (ADR 0008, step 1)

Issue: #58. Implemented in PRs 1a (#59), 1b (#60), 1c (#62) and 1d (#64).

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

### PR 1a (#59)

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
- **Boundary:** `cmake/framework_boundary.cmake` lists 29 neutral files, and the ctest `boundary.neutral_includes` checks them. Include paths are normalized first, so `<./branes/...>` is checked too. `boundary.detects_violation` runs the check on manifests that list the racing gate codec, and a fixture that includes a racing header through `<./branes/...>`, as neutral. It requires a non-zero exit and both messages.
- **`is_framework`** is a switch over every `Channel` enumerator with no default, so a new enumerator left out of it fails the build (`-Wswitch`).
- **New tests:**
  - a mission channel reads only when named, and I2 is enforced;
  - racing's oracle noise parses, is checked, and round-trips;
  - the scenario and baseline loaders take another mission's schema and slot list.

**Acceptance:**
1. **Tests:** 208 of 208 pass on `gcc-debug` and on `clang-debug` (201 on main, plus 7 new). CI covers Release and MSVC.
2. **Byte-identity:** `tools/compare_outputs.sh` with `gcc-release`, on main and on the branch. All 39 output files are identical: 3 SITL logs, 3 replays, `score_log`, 12 `log_dump` CSV and YAML files, a 20-seed `sitl_batch`, a 20-seed precision sweep (11 files), and the harnesses' stdout.

   The first comparison differed only in the stdout lines that print the output directory, which differs between runs. The script now writes that directory as `<out>`, and blanks the precision sweep's printed wall times (the files it writes carry none without `--timing`). With that, every file compares equal.
3. **Boundary:** `boundary.neutral_includes` passes, and `boundary.detects_violation` passes.
4. **Interface:** no command line, YAML key, channel id, or on-disk header field changed. clang-format 18 (CI's version) finds nothing in the 41 changed C++ files.

**Departures:**
- **A seam the audit missed.** `LogHeader`'s `course_hash` and `course_yaml` were racing names in a neutral type. They are now `world_hash` and `world_yaml`. The on-disk fields and their order are unchanged, and `log_dump` still writes `course_hash:` and `course.yaml`.
- **The YAML helpers.** The audit didn't list them separately. They are neutral, but private to the loaders. In step 2, a racing course loader outside anatomy will need them, so anatomy will have to export them or racing will keep a copy. That choice belongs to step 2.

### PR 1b (#60)

**What changed:**
- **Slots:** `slots.hpp` keeps racing's three task-facing concepts (`Estimator`, `GateDetector`, `Planner`). The four neutral ones move to `slots/neutral.hpp`, and their nulls to `slots/neutral_nulls.hpp`. `scripted.hpp` now includes only the neutral concepts.
- **Slot context:**
  - `slots/slot_context.hpp` (neutral) defines:
    - `BasicSlotContext<World>` (vehicle, baseline, `world`);
    - the concept `MissionWorld`: a world carries the vehicle's start pose;
    - `ContextBuilt<S, Ctx>`, `make_slot`, `TruthConsumer` and `parameter_in`.
  - Racing's `SlotContext` (`slots/context.hpp`) is `BasicSlotContext<CourseMap<double>>`. Its `course` member is now `world`.
- **Ground truth:** `GroundTruthCore` (neutral, with the model and invariants) is the estimate. Racing's `GroundTruthEstimator` derives from it and adds only the gate-observation frame input.
- **PID glue:** `CascadedPid::from_context` is a template over any `MissionWorld`, and reads only the vehicle, the baseline and the start pose.
- **Composition:** `compose/tick.hpp` (neutral) holds `TickOutput` and the concept `RunnableStack`, which states what a harness needs from any stack. `Stack` and `make_stack` stay racing.
- **Harness:**
  - `harness.hpp` splits into neutral `args.hpp`, `files.hpp` (the baseline check, over the mission's slot list), `runner.hpp` and `synthetic.hpp`, plus racing's `harness.hpp` (its `check_baseline` overload and `make_stack`).
  - `Runner` is generic over `RunnableStack` and takes the mission's pass-through channels: its new invariant I2 says they hold no framework channel.
- **Boundary:** the manifest lists 40 neutral files, 11 of them new: the 10 neutral headers above and the PID glue. It also lists the new test `tests/sim/neutral_mission.cpp`. That test includes only neutral headers, and drives a made-up non-racing mission:
  - its world is a goal and a start pose, with no course;
  - its stack is built from neutral slots;
  - its own channel passes through the `Runner`, and racing's channel 5 is rejected;
  - `CascadedPid` builds from its slot context.

**Acceptance:**
1. **Tests:** 210 of 210 pass on `gcc-debug` and on `clang-debug` (208 before, plus the 2 neutral-mission tests).
2. **Byte-identity:** all 39 output files are identical to main's before the split (`tools/compare_outputs.sh`, `gcc-release`).
3. **Boundary:** `boundary.neutral_includes` passes over 40 files, and `boundary.detects_violation` passes.
4. **Interface:** no command line, YAML key, channel id or log field changed. clang-format 18 finds nothing in the 24 changed C++ files.

**Departures:**
- **Pass-through list, not a decoding hook.** The plan said mission channels would be decoded "through a mission hook". Racing's only mission channel is recorded but never consumed, so the `Runner` takes a list of pass-through channels instead. A mission channel the stack does consume (for example scout-drone's depth images) will need a decoding hook. Following ADR 0008, that is added when scout-drone needs it (step 3), not speculatively.
- **`MissionWorld`.** The plan didn't name this concept. It came from the audit's finding that `CascadedPid` reads the start pose from the course: making the start pose part of every world keeps the PID glue neutral without a separate context field.

### PR 1c (#62)

PR 1c of the plan is split in two. **1c** covers the simulator, the batch runner, the sweep types and the baseline generator. **1d** covers the harness programs (`apps/`), so each PR stays reviewable.

**What changed:**
- **Simulator:**
  - `sim/basic_simulator.hpp` (neutral) holds `BasicSimulator<Oracle>`: the reflex plant, the simulated FC, the IMU, faults, camera timing, truth and `sim_config`, as before.
  - The mission supplies the start pose and an `OracleSensor`: at each camera frame, it turns the true state into the log event to emit, or none. `NoOracle` emits none.
  - Racing's `simulator.hpp` defines `GateOracle` (`project_gates` onto `oracle_gates`) and a `Simulator` that derives from `BasicSimulator<GateOracle>`, with the same `create(vehicle, course, scenario, control_hz, options)` as before. Its check for racing's scenario parameters moved there.
- **Camera model:** `sim/camera_model.hpp` (neutral) holds the rotation helpers, the pixel projection and `to_camera`, moved out of the gate projector. The simulator uses them, and another mission's oracle will need them too.
- **Batch:**
  - `sim/closed_loop.hpp` (neutral) holds the concept `RunScorer`, `BasicRun<ScoreT>` and `run_closed_loop()`. The loop is the one from `run_one`, including the timing-fault check, generic over the simulator's oracle, the stack and the scorer.
  - It also holds the JSON line, `percentile` and `failure_reason`.
  - Racing's `batch.hpp` keeps `Run` (= `BasicRun<Score>`) and `run_one()`.
- **Sweep types:** the type list and `ulp_at` stay in the neutral `sweep_types.hpp`, and `in_plane_ulp(gate)` moves to the racing `gate_resolution.hpp`.
- **Simulator vehicle helpers:** `sim/vehicle.hpp` includes the neutral configs and `msg/region.hpp`, and is now neutral.
- **Baseline generator:**
  - `cmake/composition.cmake` (neutral) holds `composition_define_slots`, `composition_register_impl` and `composition_generate`: parse, check and hash a baseline's composition against a named registry, and configure the mission's header template.
  - Racing's `slot_registry.cmake` defines its seven slots on the `racing` registry. `baselines.cmake` calls `composition_generate` and adds racing's harness targets.
  - The template's variables are now `@BASELINE_NAME@`, `@BASELINE_HASH@`, `@BASELINE_INCLUDES@` and `@BASELINE_TYPES@`.
- **Boundary:** the manifest lists 50 files, adding the 7 neutral simulator headers and 4 neutral CMake modules. The boundary test only checks C++ includes, so for the CMake files it checks that they exist.
- **New tests:** in `tests/sim/neutral_mission.cpp`, with neutral headers only:
  - `BasicSimulator<NoOracle>` runs from another mission's start pose and emits only framework channels;
  - `run_closed_loop` runs that mission's stack and scorer, and stops when its scorer is complete.

**Acceptance:**
1. **Tests:** 212 of 212 pass on `gcc-debug` and on `clang-debug` (210 before, plus the 2 new).
2. **Byte-identity:** all 39 output files are identical to main's before the split. The generated baseline headers (`build/*/generated/`) are byte-identical to those generated before the change.
3. **Boundary:** `boundary.neutral_includes` passes over 50 files, and `boundary.detects_violation` passes.
4. **Interface:** no command line, YAML key, channel id, log field, or `Simulator::create` signature changed. clang-format 18 finds nothing in the 12 changed C++ files.

**Departures:**
- **1c split in two.** The harness programs are their own PR, 1d.
- **The camera model moved.** The audit counted it as part of the racing gate projector, but it is neutral camera geometry that any mission's oracle needs.
- **Order of checks in `Simulator::create`.** Racing's scenario-parameter check now runs before `BasicSimulator`'s rate and duration checks. A scenario with several faults may therefore report a different first error. Valid scenarios are unaffected.

### PR 1d

**What changed:** the harness programs keep their racing parts, and their mission-neutral logic moves into neutral library functions:
- **`sim/sitl_loop.hpp`:** `run_sitl()`, the SITL closed loop that writes the log (log every event, feed it, log every output, send each command back). `apps/sitl` calls it.
- **`harness/replay.hpp`:**
  - `replay_log()`: replay a log's inputs through the stack into a new log, keeping the outputs for verification.
  - `verify_outputs()`: compare them record for record.
  - `apps/replay` closes the new log between the two calls, as before, so errors come out in the same order.
- **`harness/parallel.hpp`:** `for_each_index()`, the batch runners' thread pool, used by `sitl_batch` and `precision_sweep`.
- **`harness/args.hpp`:** gains `parse_number<N>()`, which both batch runners had duplicated.
- **`harness/flight.hpp`:** `ticks_fit()` (the int64 bound on tick times) and `run_ticks()`, used by `apps/flight_stack`.
- **`log/dump.hpp`:**
  - the CSV formats of the framework's channels, and their formatting helpers;
  - `ChannelFormat`: a mission channel's file, header row and row function;
  - `dump_records()`: the dump loop;
  - `write_text()`.

  `apps/log_dump` keeps racing's `oracle_gates` format and the header files it names after the course (`course.yaml`, `course_hash:`).

`sitl_batch`'s summary (laps, gate errors, speeds, failures) and `precision_sweep` stay racing: they are about racing's score.

The manifest lists 56 files, adding the 5 new neutral headers above and the new test `tests/sdk/neutral_harness.cpp` (neutral headers only). That test covers verification messages, the tick bound, the pool, number parsing, and a dump with another mission's channel.

**Acceptance:**
1. **Tests:** 217 of 217 pass on `gcc-debug` and on `clang-debug` (212 before, plus 5 new).
2. **Byte-identity:** all 39 output files are identical to main's before the split. These are every output of `sitl`, `replay --verify`, `score_log`, `log_dump`, `sitl_batch` and `precision_sweep`. `flight_stack`, which the script doesn't run, gives the same output and exit code as before for `null` and `b0-gt-pid` at 10 cycles, 0 cycles, and a cycle count that would overflow.
3. **Boundary:** `boundary.neutral_includes` passes over 56 files, and `boundary.detects_violation` passes.
4. **Interface:** no command line, YAML key, channel id or log field changed. clang-format 18 finds nothing in the 13 changed C++ files.

## Step 1 result

Every file of the framework is now mission-neutral or racing-only. `cmake/framework_boundary.cmake` lists the 56 neutral files, and the ctest `boundary.neutral_includes` keeps them free of racing headers.

Across PRs 1a–1d, every harness output stayed byte-identical to main's before the split, and the tests grew from 201 to 217.

Step 2 of ADR 0008 is next: move the listed files to `anatomy` (namespace `branes::anatomy`), and have racing-drone consume them, with `tools/compare_outputs.sh` checking byte-identity again.

