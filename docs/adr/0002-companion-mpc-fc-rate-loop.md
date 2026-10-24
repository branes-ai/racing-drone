# ADR 0002: MPC on the companion, INDI + rate loop on the flight controller

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements CTL-04/06, SAF-01, FPV-05; cortex `docs/arch/flight-controller-analysis.md`

## Context

NMPC needs more compute than an STM32H7 provides. The rate loop needs 2–8 kHz and hard real time, which a Linux companion cannot guarantee. Kill, takeover, and failsafes must work when the companion software has crashed.

## Decision

- The **companion** runs perception, estimation, planning, MPCC (100–200 Hz), and the DAA safety filter. It sends **collective thrust plus body rates** to the FC at ≥ 500 Hz.
- The **FC** runs IMU sampling, the attitude estimate, the INDI and rate loop, the mixer, DShot, RC input, and the mode manager and pilot arbiter.
- Safety functions are split by what they need:

| Function | Runs on | Why |
|---|---|---|
| Kill switch (FPV-05), arming, RC-loss action (SAF-04) | FC | Must work with the companion dead |
| Companion heartbeat and command-stream loss (SAF-01) | FC | Detects the companion's failure |
| Attitude and rate limits, motor saturation handling | FC | Needs only IMU and actuator state |
| Geofence (SAF-02), estimator-health ladder (SAF-03) | Companion | Need the position estimate, which only the companion has |
| DAA safety filter (DAA-04), flight-envelope filter | Companion | Need the state estimate and intruder tracks |

- **Loss-of-companion behavior.** The FC cannot enforce a fresh geofence, health, or DAA result once the companion is gone. Losing the companion therefore always ends in the SAF-01 failsafe, within 50 ms: level-and-descend, or disarm in a netted venue, per configuration. In the development profile, a pilot can take `MANUAL` from that failsafe. The physical safety barrier for the companion-side functions during loss of the companion is the venue (nets, flight-volume margins).
- To de-risk hardware bring-up, the first flights use a **Betaflight bridge**. The reflex FC firmware replaces it in Phase 8.
- Motor-level learned policies (MonoRace-style) are an exception that the FC accepts only in an explicitly configured mode.

## Consequences

- The thrust-and-body-rate interface is the standard one in the literature (Swift, Agilicious), so baselines and results can be compared with published work.
- The FC must time-stamp and relay the IMU to the companion. Clock synchronization is a requirement.
- Two firmware paths (Betaflight, then reflex) exist for a while.

**Current state:** proposed.
