# ADR 0001: racing drones are baselines composed into fixed slots

**Status:** Proposed
**Date:** 2026-10-23
**Related:** [framework architecture](../arch/framework-architecture.md) §1–3

## Context

This repo has to deliver one racing mission, and also be the framework that later racing baselines build on: new estimators, learned controllers, extra sensors, DAA. If each new idea is a fork of the flight stack, the results stop being comparable and the integration work multiplies.

## Decision

- Every swappable component role is a **slot**, defined as a C++20 concept in namespace `branes::racing`, declared in `<branes/racing/slots/*.hpp>`. There are seven slots: `Estimator`, `GateDetector`, `Planner`, `Controller`, `SafetyFilter`, `DaaTracker`, and `FcBridge`. This list is the registry; the architecture §3 defines the same seven concepts.
- The **mode manager and pilot arbiter** is *not* a slot. It is FC-resident safety logic in reflex, the same for every baseline. The baseline cannot replace it, only configure it (league or development profile, failsafe actions).
- A **baseline** is a named composition of slot implementations plus parameter files, described by `baselines/<name>.yaml`.
- The harnesses (replay, SITL, HIL, flight) take the baseline as their only *algorithm* input. Mission inputs are separate files, and the harness selects them as follows:

| Harness | Required inputs | Optional | Rules |
|---|---|---|---|
| `sitl_<name>` | `--scenario` | — | The scenario file names its course and vehicle and adds randomization and seed. `--course` / `--vehicle` are rejected, to keep one source of truth |
| `replay_<name>` | `--log` | `--course`, `--vehicle` | The log header records the course and vehicle hashes. Overrides are allowed only for what-if replays, and the result is marked *non-reference* |
| `flight_stack_<name>` (flight) | `--course`, `--vehicle` | — | No scenario file: nothing is simulated |
| `flight_stack_<name>` (HIL) | `--course`, `--vehicle`, `--scenario` | — | The scenario drives the simulated sensors. Its course and vehicle must match the explicit ones, or the stack refuses to start |

- Every run logs the hashes of the baseline, course, vehicle, and scenario files, so the scoreboard compares like with like.
- Every baseline is scored by the same scoreboard schema on the same scenario suite.
- Implementations live in cortex, reflex, or synapse. This repo holds only oracles (ground truth, oracle gates and intruders) and glue.

## Consequences

- Adding a baseline costs a YAML file plus any new component. Comparisons are like-for-like.
- Slot interfaces must be designed carefully up front. Changing one touches every implementation.
- Composition is resolved at compile time (concepts and templates) to keep the hot path free of virtual dispatch. Each baseline is therefore its own build target.

## YAML to build-target mapping

- `baselines/<name>.yaml` has two parts. A `composition:` block names one implementation per slot; a `parameters:` block holds runtime values.
- At configure time, CMake reads each baseline file's `composition:` block and generates `compose/<name>.hpp`, which instantiates the slots. It then adds one target per harness: `flight_stack_<name>`, `replay_<name>`, and `sitl_<name>`. HIL uses `flight_stack_<name>` on the real companion.
- Each binary embeds its baseline name and a hash of the `composition:` block. At startup it loads `baselines/<name>.yaml`, applies the `parameters:` block, and **refuses to start** if the name or the composition hash differs from its own.
- Changing a parameter means a restart. Changing the composition means a rebuild.

**Current state:** proposed; no code yet.
