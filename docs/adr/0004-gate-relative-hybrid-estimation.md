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
- **Promotion criterion.** `VioGateEkf` is the primary estimator only if it beats `GateEkf` as follows:
  - **Comparison set:** the same baseline (B2 vs. B2m, identical controller and parameters) on the test-plan sensor-degradation group. That is camera corruption at 25 % and 50 % of frames, and 0.5 s and 1 s blackouts. Run 200 paired seeds per scenario in L3 (the same seed for both estimators).
  - **Required gain:** two-lap success rate at least **5 percentage points** higher, *or* true gate-crossing error (|bias| + 3σ) at least **20 %** lower, pooled over the set, with neither metric worse.
  - **No regression:** on the nominal group, success rate no more than 1 percentage point lower, and both estimators still meet EST-04 / EST-05.
  - **EST-04 / EST-05 during blackouts:** they apply at the **first gate crossed after the blackout ends**, not during it. NEES consistency (EST-05) must hold throughout, including the blackout. That is the point of VIO: a covariance that grows honestly.

## Consequences

- Phase 3 does not depend on fixing VIO. B2m can fly on `GateEkf` alone.
- Cortex VIO work (NEES, latency, rolling shutter) is prioritized by a mission metric rather than by EuRoC ATE.

**Current state:** proposed.
