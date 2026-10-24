# ADR 0002: MPC on the companion, INDI + rate loop on the flight controller

**Status:** Proposed
**Date:** 2026-10-23
**Related:** requirements CTL-04/06, SAF-01, FPV-05; cortex `docs/arch/flight-controller-analysis.md`

## Context

NMPC needs more compute than an STM32H7 provides. The rate loop needs 2–8 kHz and hard real time, which a Linux companion cannot guarantee. Kill, takeover, and failsafes must work when the companion software has crashed.

## Decision

- The **companion** runs perception, estimation, planning, MPCC (100–200 Hz), and the DAA safety filter. It sends **collective thrust plus body rates** to the FC at ≥ 500 Hz.
- The **FC** runs IMU sampling, the attitude estimate, the INDI and rate loop, the mixer, DShot, RC input, the mode manager, and every failsafe.
- To de-risk hardware bring-up, the first flights use a **Betaflight bridge**. The reflex FC firmware replaces it in Phase 8.
- Motor-level learned policies (MonoRace-style) are an exception that the FC accepts only in an explicitly configured mode.

## Consequences

- The thrust-and-body-rate interface is the standard one in the literature (Swift, Agilicious), so baselines and results can be compared with published work.
- The FC must time-stamp and relay the IMU to the companion. Clock synchronization is a requirement.
- Two firmware paths (Betaflight, then reflex) exist for a while.

**Current state:** proposed.
