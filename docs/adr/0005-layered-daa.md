# ADR 0005: layered detect-and-avoid: MPC constraints plus an independent CBF safety filter

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements DAA-01..10; architecture §8

## Context

Collisions with other drones cost both champion systems races (MonoRace placed third after hitting another drone; most of Swift's losses were collisions). DAA must keep the drone racing when possible, and guarantee separation when not, even if the MPC misses its deadline or diverges.

## Decision

- **Detection:** DroneNet, a second head on the GateNet backbone, run through synapse, using mono vision only by default.
- **Tracking:** an IMM-EKF in cortex. It predicts tubes over the MPC horizon.
- **Strategic avoidance:** time-varying separation constraints in MPCC.
- **Tactical avoidance:** a control-barrier-function filter in reflex, run every cycle in bounded time. It changes the command only when separation would be violated.
- The **declared envelope** (DAA-06) limits guarantees to the forward hemisphere and to closing speeds up to 10 m/s (20 m/s for crossings). Everything else is reported, not guaranteed.

## Consequences

- The DAA guarantee does not depend on the optimizer converging.
- The sensor interface leaves room for extra DAA sensors in non-A2RL baselines, and the design maps onto ASTM F3442-style DAA for other missions.

**Current state:** proposed.
