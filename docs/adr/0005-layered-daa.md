# ADR 0005: layered detect-and-avoid: MPC constraints plus an independent CBF safety filter

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements DAA-01..10; architecture §8

## Context

Collisions with other drones cost both champion systems races (MonoRace placed third after hitting another drone; most of Swift's losses were collisions). DAA must keep the drone racing when possible and act to restore separation when not, including when the MPC misses its deadline or diverges.

## Decision

- **Detection:** DroneNet, a second head on the GateNet backbone, run through synapse, using mono vision only by default.
- **Tracking:** an IMM-EKF in cortex. It predicts tubes over the MPC horizon.
- **Strategic avoidance:** time-varying separation constraints in MPCC.
- **Tactical avoidance:** a control-barrier-function filter in reflex, run every cycle in bounded time. It changes the command only when the predicted separation would fall below the threshold.
- **What we claim.** There is no runtime guarantee of separation. The acceptance target is statistical and measured in simulation: DAA-05, separation ≥ 1.0 m in ≥ 99 % of in-envelope encounters. The **declared envelope** (DAA-06) is the forward hemisphere at closing speeds up to 10 m/s (20 m/s for crossings). Outside it, results are reported but no target applies.
- **Assumptions behind the target.** The 1 % residual and the envelope cover the cases where these do not hold:
  1. The intruder is detected and the track confirmed before it is closer than the DAA detection-range budget (requirements §6.3). Missed or late detections are not compensated.
  2. Tracker errors are within the IMM covariance. Under-estimated uncertainty makes the CBF constraint optimistic.
  3. The `ConflictSet` is fresh within the age limit in architecture §8. Beyond the limit, the stale-data fallback applies.
  4. The CBF problem is feasible within the vehicle's actuator authority and rate limits. If it is infeasible, the filter commands the maximum-authority escape maneuver for the predicted conflict and logs the event. Separation is not assured in that case.
  5. Intruders behave within the prediction model (bounded acceleration). An intruder that maneuvers beyond the bound can defeat the prediction.
  6. If the tracker loses a track, the track coasts with growing uncertainty for up to 0.3 s, then it is dropped. A dropped intruder is no longer avoided.

## Consequences

- The tactical DAA layer does not depend on the optimizer converging. It does depend on detection, tracking, and the assumptions above. They are tested individually (test plan §4) and together (test plan §7).
- The sensor interface leaves room for extra DAA sensors in non-A2RL baselines, and the design maps onto ASTM F3442-style DAA for other missions.

**Current state:** proposed.
