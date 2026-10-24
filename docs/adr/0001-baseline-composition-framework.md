# ADR 0001: racing drones are baselines composed into fixed slots

**Status:** Proposed
**Date:** 2026-10-23
**Related:** [framework architecture](../arch/framework-architecture.md) §1–3

## Context

This repo has to deliver one racing mission, and also be the framework that later racing baselines build on: new estimators, learned controllers, extra sensors, DAA. If each new idea is a fork of the flight stack, the results stop being comparable and the integration work multiplies.

## Decision

- Every component role is a **slot**, defined as a C++20 concept in `branes::racing::slots`. The roles are estimator, gate detector, planner, controller, safety filter, DAA tracker, and FC bridge.
- A **baseline** is a named composition of slot implementations plus parameter files, selected by `baselines/<name>.yaml`.
- All harnesses (replay, SITL, HIL, flight) take a baseline and nothing else.
- Every baseline is scored by the same scoreboard schema on the same scenario suite.
- Implementations live in cortex, reflex, or synapse. This repo holds only oracles (ground truth, oracle gates and intruders) and glue.

## Consequences

- Adding a baseline costs a YAML file plus any new component. Comparisons are like-for-like.
- Slot interfaces must be designed carefully up front. Changing one touches every implementation.
- Composition is resolved at compile time (concepts and templates) to keep the hot path free of virtual dispatch. Each baseline is therefore its own build target.

**Current state:** proposed; no code yet.
