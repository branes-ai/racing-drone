# ADR 0006: the hot path runs in one process; Zenoh carries everything else

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements SW-01/02/06; cortex ADR 0008, 0014

## Context

The camera-to-motor budget is 27 ms p99 at speeds where each millisecond is 2.8 cm of travel. Cortex's architecture exposes components as Zenoh daemons (ADR 0014), and each middleware hop adds serialization, scheduling jitter, and copies.

## Decision

- The companion **flight stack** is one process. Camera, perception, estimator, planner, MPC, and safety filter communicate through pre-allocated, lock-free single-producer queues, with zero-copy image views.
- **Zenoh** carries telemetry, logs, the ground station, OSD content, and parameter upload, published from the hot path through non-blocking queues.
- Cortex SDK components are linked in as libraries (ADR 0014 already makes the SDK middleware-agnostic). Their daemon form is used for development and tooling.

## Consequences

- Latency is bounded and measurable per stage.
- A crash in one component takes down the whole flight stack. That is acceptable, because the FC failsafes cover it (ADR 0002) and fault tests exercise it.

**Current state:** proposed.
