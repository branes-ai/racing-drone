# Framework boundary: what is mission-neutral in branes::racing

**Date:** 2026-10-28
**Purpose:** The audit behind [ADR 0008](../adr/0008-anatomy-mission-framework.md). For every header, source and build file of the composition framework, it records whether it is mission-neutral, racing-only, or mixed, and what a mixed file needs before its neutral part can move to `anatomy`.

The audit was made at commit 54ce20d by reading each file's includes and the racing types it uses. The racing types are `Gate`, `CourseMap`, `GateObservation(s)`, the oracle gate projector, the scorer, and the `gate_detector` slot.

## Summary

- **Neutral, moves as is:** the utilities, `result`, time and `TimeBase`, geometry, sensors, truth, the state estimate, control, flight-controller status, DAA conflicts, the controller and safety slots, the FC and IMU models, the log container, and the build helpers.
- **Racing-only, stays:** the course, the gate detector and waypoint planner, the gate projector, the scorer, the course config, and the racing baselines.
- **Mixed, needs a split first:** the rest. Racing enters them through four seams:
  1. **The task.** Three slot concepts (`Estimator`, `GateDetector`, `Planner`) and `SlotContext` take the course or gate observations.
  2. **The composition.** `Stack` wires the racing slot set, and the harness builds it from a `CourseMap`.
  3. **The record format.** One log channel (`oracle_gates`) and its codec are racing data, inside the framework's channel enum.
  4. **The simulator.** Its loop owns the course and calls the gate projector, and the batch runner calls the racing scorer.

## Per file

`sdk/include/branes/racing/` is abbreviated `sdk/`; `sim/include/branes/racing/sim/` is `sim/`.

| File | Class | Racing content | Before it moves |
|---|---|---|---|
| `sdk/result.hpp` | Neutral | — | — |
| `sdk/util/bytes.hpp`, `sha256.hpp`, `splitmix64.hpp`, `sdk/src/util/sha256.cpp` | Neutral | — | — |
| `sdk/msg/geometry.hpp` | Neutral | — | — |
| `sdk/msg/time.hpp` (`Timestamp`, `TimeBase`) | Neutral | — | — |
| `sdk/msg/sensors.hpp` (IMU, camera frame) | Neutral | — | — |
| `sdk/msg/truth.hpp` | Neutral | — | — |
| `sdk/msg/estimate.hpp` | Neutral | — | — |
| `sdk/msg/fc.hpp` | Neutral | — | — |
| `sdk/msg/daa.hpp` | Neutral | — | — |
| `sdk/msg/control.hpp` | Neutral | `ReferenceSample::progress` is documented as "along the course" | Reword it as the path parameter of the reference; it is generic |
| `sdk/msg/course.hpp` | Mixed | `Gate`, `CourseMap` | `Geofence` and `StartPose` move to a neutral header; the rest stays |
| `sdk/msg/perception.hpp` | Mixed | `GateObservation(s)` | `IntruderDetection(s)` moves; gate observations stay |
| `sdk/msg/codec.hpp` | Mixed | The `GateObservations` codec | The gate codec moves to a racing header |
| `sdk/log/log.hpp`, `sdk/src/log/log.cpp` | Mixed | `Channel::oracle_gates` in a closed enum; `is_sim`/`is_known` hard-code it | Framework channel ids become a fixed set and missions register their own. Id 5 stays `oracle_gates`, so existing logs read unchanged |
| `sdk/config/config.hpp`, `sdk/src/config/config.cpp` | Mixed | `CourseConfig`; `ScenarioConfig::course_ref`; `kSlotNames` lists the racing slots; `SimParams::oracle_corner_sigma_px` | Vehicle, baseline and scenario loaders move. The course loader, the scenario's world reference, the slot list and the oracle parameters become mission-supplied |
| `sdk/slots/slots.hpp` | Mixed | `Estimator` takes gate observations; `GateDetector`; `Planner` takes a `CourseMap` | `Controller`, `SafetyFilter`, `DaaTracker` and `FcBridge` move; the three task-facing concepts stay |
| `sdk/slots/context.hpp` | Mixed | `SlotContext::course` | `SlotContext` becomes generic over the mission's world type |
| `sdk/slots/null.hpp` | Mixed | `NullGateDetector`, `NullPlanner`, and `NullEstimator`'s gate argument | The nulls of the neutral slots move |
| `sdk/slots/ground_truth.hpp` | Mixed | Satisfies the racing `Estimator` (gate argument ignored) | Its truth pass-through moves; a thin racing adapter stays |
| `sdk/slots/scripted.hpp` | Neutral | — | — |
| `sdk/compose/stack.hpp` | Mixed | The racing slot set; the frame path detects gates and fuses them | The `Stack` stays a mission type. The framework defines what a stack must provide to the harness (a concept) |
| `sdk/harness/harness.hpp` | Mixed | `make_stack` takes a `CourseMap`; the replay switch decodes `oracle_gates` | `Runner` becomes generic over that stack concept; building the stack and the mission channels are supplied by the mission |
| `sdk/slots/waypoint.hpp` | Racing | Plans through the gates | Stays |
| `sim/fc.hpp`, `sim/imu.hpp` | Neutral | — | — |
| `sim/vehicle.hpp` | Neutral once `StartPose` moves | Includes `course.hpp` for `StartPose` | Follows `course.hpp`'s split |
| `sim/simulator.hpp` | Mixed | Owns the course; calls the gate projector; emits `oracle_gates` | The loop (dynamics, FC, IMU, faults, camera timing, truth) moves; the mission supplies its world and its oracle sensors |
| `sim/batch.hpp` | Mixed | Takes a `CourseMap`; calls the racing scorer | Becomes generic over the mission's world and scorer |
| `sim/sweep_types.hpp` | Mixed | `in_plane_ulp(gate)` | The type list moves; the gate helper stays |
| `sim/gates.hpp` (oracle projector) | Racing | — | Stays |
| `sim/score.hpp` | Racing | Laps, gates | Stays |
| `baselines/glue/` (`cascaded_pid.hpp`, `reflex_params.hpp`) | Neutral | — | Moves with the controller it adapts |
| `cmake/warnings.cmake`, `deps.cmake`, `compile_all.cmake` | Neutral | — | — |
| `cmake/baselines.cmake`, `slot_registry.cmake`, `racing_baseline.hpp.in` | Mixed | The slot list (`RACING_SLOTS`) and the `racing_` names | The generator moves; the slot list and the names become mission-supplied |
| `apps/sitl`, `replay`, `sitl_batch`, `flight_stack`, `log_dump` | Mixed | Through `Stack`, `CourseMap` and the scorer | The harness logic moves as library functions; each mission keeps thin mains |
| `apps/score`, `apps/precision_sweep` | Racing | The scoreboard; gate error by gate | Stay |
| `baselines/*.yaml`, `courses/`, `scenarios/`, `vehicles/` | Racing data | — | Stay; `vehicles/rp1.yaml` is copied by any mission that flies RP-1 |

## Observations

- **The mechanism was built neutral.** No neutral file reaches into a racing header except through the four seams. That makes the move a split along existing lines rather than a redesign.
- **The task is the real seam.** In each of the four places where racing enters, it enters as the mission's *task*: what the vehicle is asked to do and what it perceives of it. For racing that is a course of gates. For scout-drone it would be a goal and depth observations. A framework generic over the task removes all four seams.
- **The simulator core depends on reflex** (the dynamics model). The contracts, log and harness do not. Splitting the framework into two build targets keeps that dependency off consumers that need only the contracts.
