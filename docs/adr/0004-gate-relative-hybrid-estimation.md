# ADR 0004: hybrid estimation with VIO, gate PnP, and a drag-aware model

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements EST-03..08; cortex ADR 0012

## Context

MonoRace won with gate PnP and a model-based EKF, and no VIO. It crashed after more than 25 m without valid images. Cortex already has an MSCKF VIO, but it is over-confident (#212) and five times too slow for 90 Hz. Accelerometers saturate at around 7 g.

## Decision

- Two estimator implementations behind the `Estimator` slot:
  - `GateEkf`: MonoRace-style. 16 states, drag-model prediction, IMU-saturation switching, delay compensation.
  - `VioGateEkf`: the cortex MSCKF fused with gate PnP and the same model and saturation handling.
- Both express their state in the **course frame**, anchored to the gate map.
- `VioGateEkf` must show a measurable gain over `GateEkf` on the camera-dropout and blackout scenarios to stay the primary estimator.

## Consequences

- Phase 3 does not depend on fixing VIO. B2m can fly on `GateEkf` alone.
- Cortex VIO work (NEES, latency, rolling shutter) is prioritized by a mission metric rather than by EuRoC ATE.

**Current state:** proposed.
