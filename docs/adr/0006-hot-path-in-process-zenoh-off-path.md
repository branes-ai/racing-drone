# ADR 0006: the hot path runs in one process; Zenoh carries everything else

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements SW-01/02/06; cortex ADR 0008, 0014

## Context

The camera-to-motor budget is 29 ms p99 at speeds where each millisecond is 2.8 cm of travel. Cortex's architecture exposes components as Zenoh daemons (ADR 0014), and each middleware hop adds serialization, scheduling jitter, and copies.

## Decision

- The companion **flight stack** is one process. Camera, perception, estimator, planner, MPC, and safety filter communicate through pre-allocated, lock-free single-producer queues, with zero-copy image views.
- **Replay capture (SYS-08) does not use Zenoh.** Each hot-path channel writes to its own pre-allocated single-producer ring buffer. A recorder thread on a non-RT core drains the buffers to local storage (NVMe) as MCAP-style records.
  - **Never block.** A producer never waits on the recorder (SW-01).
  - **Sizing.** Each ring holds at least 2 s of that channel's peak rate, and the recorder's sustained write rate must exceed the total peak input rate by at least 2×. Both are checked in HIL.
  - **Overflow.** If a ring is full, the new record is dropped, and a per-channel drop counter with the drop's timestamp range is written into the log.
  - **Loss accounting.** A log with any dropped hot-path record is flagged *not replay-complete*. SYS-08 is met only by replay-complete logs, and any drop fails the SYS-08 test. Camera frames may be logged decimated or compressed by configuration. That is declared in the log header and is not counted as loss.
- **Zenoh** carries telemetry, the ground station, OSD content, and parameter upload, plus downsampled log streams for live viewing. All are published from the hot path through the same non-blocking, drop-on-full queues. Zenoh drops are acceptable and counted, but never affect replay.
- Cortex SDK components are linked in as libraries (ADR 0014 already makes the SDK middleware-agnostic). Their daemon form is used for development and tooling.

## Consequences

- Latency is bounded and measurable per stage.
- A crash in one component takes down the whole flight stack. That is acceptable, because the FC failsafes cover it (ADR 0002) and fault tests exercise it.

**Current state:** proposed.
