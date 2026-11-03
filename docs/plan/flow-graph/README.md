---
status: complete
issue: branes-ai/racing-drone#98
---
# Plan: B0 as a graph (anatomy ADR 0004, step 3)

Issue: #98. Implemented in #99. Anatomy's side: the graph builder, the CPU stack and `log_compare`, released in anatomy v0.8.0 and v0.9.0 ([anatomy's plan](https://github.com/branes-ai/anatomy/blob/main/docs/plan/flow-graph/README.md)). scout-drone did the same for `s0-gt-pid` in step 2 ([scout-drone#25](https://github.com/branes-ai/scout-drone/pull/25)).

## Goal

Step 3 of [anatomy ADR 0004](https://github.com/branes-ai/anatomy/blob/main/docs/adr/0004-domain-flow-composition.md): racing's stack described as a graph of operators, which anatomy checks as it is built and runs on its CPU stack. Each baseline gets a second build, the graph's, beside its slot build. The graph build exists so that step 4 can lower the graph to other hardware. For now it must fly exactly as the slot build does.

What stays the same:
- **The slot builds:** their targets, their logs and their outputs are byte-identical to main's.
- **Formats:** the log format, the YAML files' existing keys, and the command lines.

## Acceptance (from #98, stated before any code)

1. **The graph:** racing's graph has seven composites, one per slot, and describes the slot `Stack` exactly: the same ports, one region per input, and the firings in `Stack`'s order. `null`, `scripted-hop` and `b0-gt-pid` declare `class: scoreboard` and a `graph:`. Each gets `sitl_<name>_graph`, `replay_<name>_graph` and `sitl_batch_<name>_graph`.
2. **Equal logs:** for each of the three baselines, on every scenario in `scenarios/`, the graph build's SITL log equals the slot build's except `composition_hash`, according to anatomy's `log_compare --except composition_hash`. Compared whole, the two logs must differ in that field, which shows the graph's hash is the one recorded.
3. **Equal batches:** on `b0-nominal`, with 10 seeds, the two builds' batches write byte-identical JSON lines and identical reports.
4. **Determinism:** `gate.b0.determinism.graph` (a rerun and two replays, byte-identical) passes on the graph build of `b0-gt-pid`.
5. **The slot builds are unchanged:** `tools/compare_outputs.sh` with `gcc-release` gives all 39 files identical to main's, and logs written by main's `sitl_null`, `sitl_scripted-hop` and `sitl_b0-gt-pid` replay with `--verify`.
6. Every test passes on the four Linux presets and on MSVC Debug and Release, with clang-format and commitlint clean.

## Design

The graph is C++, in `<branes/racing/flow/...>`, namespace `branes::racing::graph`. It follows scout-drone's, which follows anatomy's plan.

| Header | What it holds |
|---|---|
| `domains.hpp` | The domain of each racing message: `Gate`, `CourseMap`, `GateObservation`, `GateObservations` |
| `bindings.hpp` | The bindings of racing's three task composites: the estimator, the gate detector and the planner |
| `graph.hpp` | The graph's ports, the scoreboard graph, and `graph::Stack`, the CPU stack that runs it |

### The ports

| Port | Kind | Message |
|---|---|---|
| `imu` | input | `ImuSample` |
| `camera` | input, timed by the frame's mid-exposure | `CameraFrame` |
| `truth` | input, SITL only | `TruthState` |
| `tick` | the control tick | `Timestamp` |
| `course` | constant | `CourseMap` |
| `detections` | constant, empty: nothing produces intruder detections yet | `IntruderDetections` |
| `estimate`, `reference`, `command` | outputs, at the tick | `StateEstimate`, `Reference`, `ThrustRates` |

### The composites

Seven composites, one per slot. A composite's binding is the CPU calling convention of its operators: one handler per method of the slot's concept. So every slot implementation is an operator of its composite, unchanged, registered under its slot registry name (`cmake/flow_registry.cmake`).

| Composite | Binding | Handlers |
|---|---|---|
| `estimator` | `graph::EstimatorBinding` | `imu`, `frame` (camera, gates), `truth`, `estimate` (at a time) |
| `gate_detector` | `graph::GateDetectorBinding` | `detect` (camera, course, prior) |
| `planner` | `graph::PlannerBinding` | `plan` (course, estimate) |
| `controller` | anatomy's `ControllerBinding` | `step` |
| `safety_filter` | anatomy's `SafetyFilterBinding` | `filter` |
| `daa_tracker` | anatomy's `DaaTrackerBinding` | `update`, `conflicts` |
| `fc_bridge` | anatomy's `FcBridgeBinding` | `send` |

The slot `Stack` has two branches, and a graph has none, so each moves into a handler:
- **Truth.** The estimator's `truth` handler hands the true state to an estimator that reads it (`TruthConsumer`), and does nothing otherwise. So every graph build reads the SITL-only `truth` port, and is SITL-only (G9).
- **Corrupted frames.** `Stack::on_frame()` doesn't call the detector on a corrupted frame, and hands the estimator no gates. The detector's `detect` handler does the same.

`Stack::on_truth()` hands the truth to every slot that reads it. The graph hands it to the estimator only. That is the one slot that reads it today (`GroundTruthEstimator`), so the two builds agree. A non-estimator slot that reads the truth would need a `truth` handler in its binding.

### The regions

Each input fires one region. A region's firings run in the order the graph writes them, which is `Stack`'s order:

1. **truth:** the estimator takes the true state.
2. **imu:** the estimator takes the sample.
3. **frame:** the estimate at the frame's mid-exposure is read. The gate detector finds the gates with that prior, and none in a corrupted frame. The estimator takes the frame and the gates, and the DAA tracker is updated with the prior and the detections.
4. **tick:** at the tick's time, the estimate is read; the planner plans from the course and the estimate; the DAA tracker gives its conflicts; the controller steps; the safety filter filters the command, with the course's geofence; and the FC bridge sends it. The estimate, the reference and the filtered command are the tick's outputs.

### The builds

- **CMake.** `racing_add_baseline(<name> GRAPH)` reads the file's `class:` and `graph:` with anatomy's `flow_add_baseline()`, and generates `generated/<name>_graph/racing_baseline.hpp` from `cmake/racing_graph.hpp.in`. It adds `replay_<name>_graph`, `sitl_<name>_graph` and `sitl_batch_<name>_graph`, from the same `apps/` sources as the slot build. There is no `flight_stack_<name>_graph`, since every graph build is SITL-only. The precision sweep stays on the slot build.
- **The generated headers.** Both builds' headers define `baseline::name`, `baseline::Composition<T>`, `baseline::check_baseline(dir)` and `baseline::hash(stack)`. So the apps don't know which build they are. The slot build checks the file's composition hash. The graph build checks only the name, and its stack checks `class:` and `graph:` against the graph it builds (G8).
- **The hash.** The slot build's log records the composition hash, as before. The graph build's records the graph's hash. `replay --verify` checks the input log's hash against the stack's once the stack is built, since a graph's hash is known only then.

### The gates

- `gate.<name>.graph`, for `null`, `scripted-hop` and `b0-gt-pid` (`tests/gate/graph_gate.cmake`): acceptance 2 on every scenario, and acceptance 3 on `b0-nominal`.
- `gate.b0.determinism.graph`: acceptance 4, the determinism gate on the graph build of `b0-gt-pid`.

## Progress

| Part | What | PR | State |
|---|---|---|---|
| 3a | The graph, its builds and its gates, for `null`, `scripted-hop` and `b0-gt-pid` | #99 | complete |
