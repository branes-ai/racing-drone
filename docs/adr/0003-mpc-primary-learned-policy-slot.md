# ADR 0003: MPCC is the primary racing controller; learned policies are a comparison baseline

**Status:** Proposed
**Date:** 2026-10-23
**Related:** [state of the art](../assessments/state-of-the-art.md) §1, §2.4

## Context

The current competition winners (MonoRace at A2RL 2025, Swift) use learned control policies. MPCC matches them in lap time with perfect state estimation, and it handles explicit constraints, which we need for DAA separation, the geofence, and actuator limits. Mission owners asked for an MPC approach.

## Decision

- **MPCC** (with a real-time-iteration SQP implemented in reflex) is the primary controller for baselines B1, B2, and B4.
- A **learned-policy controller** (B3) is implemented behind the same `Controller` slot and benchmarked head-to-head on the full scenario suite (implementation plan, Phase 7).
- The primary controller is reviewed in a new ADR after the B2 vs. B3 scoreboard exists.

## Consequences

- DAA constraints have a natural place (MPCC constraints) from the start.
- We may trail the state of the art in lap time until the comparison is done. A hybrid (learned policy, MPC or CBF supervisor) stays open.

**Current state:** proposed.
