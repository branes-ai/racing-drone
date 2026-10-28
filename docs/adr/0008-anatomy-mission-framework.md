# ADR 0008: the mission framework moves to anatomy, generic over the mission's task

**Status:** Proposed
**Date:** 2026-10-28
**Related:** [ADR 0001](0001-baseline-composition-framework.md), [ADR 0007](0007-outdoor-obstacle-avoidance-placement.md) (Decision 4); [framework boundary audit](../assessments/framework-boundary.md)

## Context

[ADR 0007](0007-outdoor-obstacle-avoidance-placement.md) decided to lift the composition framework out of this repo before a second mission repo builds on it. Two repos now exist for that:
- **`anatomy`** will hold the framework;
- **`scout-drone`** is the outdoor mission of ADR 0007, and will be the framework's second client.

Other mission repos are planned: `nano-drone`, `isr-drone`, `interceptor-drone`, `counter-uas`.

The [framework boundary audit](../assessments/framework-boundary.md) sorted every file of `branes::racing` into mission-neutral, racing-only, and mixed. Racing enters the mixed files through four seams: the task (three slot concepts and the slot context take the course), the composition (`Stack`), the record format (the `oracle_gates` channel and its codec), and the simulator (the gate projector and the scorer). In each case what enters is the mission's *task*: what the vehicle is asked to do, and what it perceives of it.

A framework generalized from one mission bakes in that mission's assumptions. One generalized for missions not yet built grows abstractions nobody uses. The second client should drive the generalization.

## Decision

1. **`anatomy` is the mission-framework tier**, between the mission repos and the platform repos (cortex, synapse, reflex).
   - It holds no algorithms. It holds the mechanism every mission composes its vehicle with.
   - Namespace `branes::anatomy`; headers under `<branes/anatomy/...>`.
   - The name follows the platform's biology: the platform repos are the nervous system, and anatomy is the body plan they are assembled into.

2. **Anatomy owns the mechanism and the mission-neutral contracts:**
   - **Utilities:** `result`, bytes, SHA-256, splitmix64.
   - **Contracts:** time and `TimeBase`, geometry, sensors, truth, the state estimate, control (`Reference`, `ThrustRates`), FC status, DAA conflicts and intruder detections, and the geofence and start pose.
   - **The four neutral slot concepts:** `Controller`, `SafetyFilter`, `DaaTracker`, `FcBridge`, with their nulls and the scripted controller.
   - **Slot construction:** the `SlotContext` machinery and baseline parameters.
   - **The log:** its container, its codecs for the neutral contracts, and a channel registry. Framework channel ids are fixed, and each mission registers its own.
   - **The harness:** `Runner`, generic over a stack concept, and replay and verification.
   - **Config loaders:** the vehicle, baseline and scenario loaders.
   - **The baseline generator:** the CMake code that builds each baseline from its composition, with the slot list supplied by the mission.
   - **The simulator core:** reflex dynamics, the FC and IMU models, faults, camera timing, truth, and the batch runner.
   - **The precision-sweep type list**, and the reflex glue (`cascaded_pid`, `reflex_params`).

3. **Each mission owns its task and its composition:**
   - its world type (racing: `CourseMap`; scout-drone: a goal and its environment);
   - the slot concepts that mention the task (racing: `Estimator` with gate observations, `GateDetector`, `Planner`);
   - its `Stack`, its oracle sensors, its scorer, and its log channels;
   - its baselines, scenarios and vehicles.

   Anatomy takes the world type as a template parameter wherever the course appears today (`SlotContext`, the simulator, the batch runner). It does not define a task-facing slot.

4. **Three build targets** (amended with step 5.2; [plan](../plan/consume-anatomy/README.md)).
   - **`branes::anatomy`:** contracts, slots, composition, configs, log and harness. Its only dependency is yaml-cpp.
   - **`branes::anatomy_glue`:** the reflex adapters (`CascadedPid`, `reflex_params`), which add reflex.
   - **`branes::anatomy_sim`:** the simulator core, which adds reflex (and Universal, for the precision-sweep types).

   This is the split racing-drone had (`branes::racing`, `branes::racing_glue`, `branes::racing_sim`). As proposed, this ADR had two targets; the third keeps a flight stack that uses `CascadedPid` from linking the simulator.

5. **The migration runs in three steps, each one keeping racing-drone's tests and gates green.**
   1. **Split in place, in racing-drone.** Each mixed file is split along the audit's lines, inside `branes::racing`, with no behavior change. Logs, replays, `sitl_batch` summaries and precision-sweep results stay byte-identical; the reference-log fixture and gates P0 and P1 check that.
   2. **Create anatomy and move the neutral half.** Racing-drone then consumes anatomy through FetchContent, as it consumes reflex today. The namespace and include paths change; nothing else does, and the same byte-identity checks hold.
   3. **Bring up scout-drone on anatomy.** Each assumption it exposes becomes a change in anatomy, checked against both clients.

6. **The repo layouts follow the existing ones.**
   - **Anatomy, like the platform repos (reflex):**
     - CMake ≥ 4.0 with Ninja and the shared presets;
     - `sdk/include/branes/anatomy/` and `sdk/src/`, plus `tests/`, `cmake/` and `docs/`;
     - a `docs-site/` generated from `docs/`, as in racing-drone;
     - CI, commitlint, release-please, and a deploy workflow.
   - **Scout-drone, like racing-drone:** `apps/`, `baselines/`, scenarios, vehicles, `sim/` for its oracle sensors and scorer, `docs/` and `docs-site/`.

## Consequences

- **Each mission repo shrinks to its mission.** A new mission writes its world type, its task slots, its stack and scorer, and its data, and reuses the harness, log, replay, simulator and gates.
- **Mission repos compare like for like.** Every mission's baselines run through the same harness and the same deterministic log, so their results line up.
- **Racing-drone changes twice, and the byte-identity checks guard both changes.** A difference in any log or result is a migration bug, not a tolerance to accept.
- **The four task-facing seams become template parameters.** That adds compile-time genericity to `SlotContext`, `Runner`, the simulator and the batch runner. The audit found no other racing assumption, but scout-drone may expose more; Step 5.3 is where they are fixed.
- **Anatomy's version is a dependency of every mission.** A contract change in anatomy ships through its releases, like reflex's.
- **ADR 0001's slot registry is split.** The registry mechanism moves to anatomy; the list of slots becomes per mission.

**Current state:** proposed.
- **Step 5.1, the split in place, is done** ([plan](../plan/framework-split/README.md); #59, #60, #62, #64).
- **Step 5.2, the move, is done:**
  - anatomy v0.1.0 holds the framework (branes-ai/anatomy#1, #3);
  - racing-drone consumes it ([plan](../plan/consume-anatomy/README.md)) through generated bridge headers, with its outputs byte-identical.
- **Step 5.3, scout-drone, has not started.**
