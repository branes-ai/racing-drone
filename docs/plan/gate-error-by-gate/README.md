# Gate error by gate, and H4's distance prediction

**Status:** Approved 2026-10-27, implemented ([racing-drone#43](https://github.com/branes-ai/racing-drone/pull/43))
**Issue:** [racing-drone#35](https://github.com/branes-ai/racing-drone/issues/35), part of the Phase 2 epic [racing-drone#9](https://github.com/branes-ai/racing-drone/issues/9)
**Related:** [precision sweep plan](../precision-sweep/README.md), [time-base plan](../time-base/README.md), [mixed-precision assessment](../../assessments/mixed-precision.md) §9 (H4)

## Issues and pull requests

| Item | What | State |
|---|---|---|
| [racing-drone#35](https://github.com/branes-ai/racing-drone/issues/35) | This plan | Closed |
| [racing-drone#43](https://github.com/branes-ai/racing-drone/pull/43) | The per-gate statistics, the regenerated sweep, and H4's verdict; this document landed with it | Merged |
| [racing-drone#40](https://github.com/branes-ai/racing-drone/issues/40) | The controller's dt defect #35 found, fixed first | Closed |

## Problem

#35 had two questions.

1. **Why `posit<16,1>` flew B0 faster than `double`** and above the velocity cap. The cause was the controller's dt, which [#40](https://github.com/branes-ai/racing-drone/issues/40) fixed before this measurement.
2. **H4's second prediction:** gate errors grow with distance from the origin. The sweep's table didn't split gate error by position, so that prediction was unmeasured.

A gate's error is measured in the gate's plane. So rounding coarsens it only through the coordinates that span that plane, not through the distance from the origin. On a2rl-2025:
- the gates facing ±x (1–5, 7–10) resolve in y and z, whose values are small wherever the gate is;
- gate 6 faces y at x = 72 m, where `posit<16,1>` has 9 fraction bits.

## Plan

1. **`sim::ulp_at<T>(x)`** (`sim/sweep_types.hpp`): the gap from T(|x|) to the next value of T above it. Built-in floats use `std::nextafter`; Universal types step to the next encoding.
2. **`sim::in_plane_ulp<T>(gate)`:** the largest `ulp_at<T>` of the gate center's coordinates on the world axes that span the gate's plane. Those are the axes on which the gate's y or z axis has a component.
3. **Per-gate statistics** (`apps/precision_sweep/main.cpp`), for every built type and gate:
   - the passages, with true gate error p50 and p99;
   - the median **paired excess** over `double`, the difference in true error for the same seed, lap and gate;
   - the in-plane ULP, and ¼ of it.

   They go to `gate_error.jsonl` and a "Gate error by gate" section of `summary.md`.
4. **Smoke test** (`sweep.b0.smoke`):
   - `gate_error.jsonl` reruns byte-identically;
   - each built type has 11 lines;
   - its per-gate passages add up to its run's;
   - `double`'s paired excess is exactly 0 where it has pairs.
5. **Results:** `docs/assessments/data/precision-sweep-b0.*` is regenerated with `.gate_error.jsonl`, and H4's verdict goes into assessment §9.

## H4 distance criterion (approved 2026-10-27, before the run)

The comparison is `posit<16,1>` against `double`, paired by seed, lap and gate, on the controller fixed by #40.
- **Holds** iff gate 6 has a larger median paired excess true gate error than every other gate, and that excess is ≥ ¼ of its in-plane ULP.
- **Refuted** otherwise.
- This re-keys H4's "x > 32 m" to the in-plane coordinate, the mechanism H4 names. The assessment discloses it as a refinement of H4.

## Acceptance criteria (stated before the tests ran)

1. For every sweep type, `ulp_at<T>` equals the neighbor gap at the course's magnitudes. In `posit<16,1>`, `ulp_at(72)` is 0.125.
2. `in_plane_ulp` uses y and z for the gates facing ±x, and x and z for gates 6 and 11.
3. Per-gate passages add up to each type's passages.
4. A rerun is byte-identical.
5. `double`'s paired excess is exactly 0.
6. The H4 verdict follows from the generated columns alone.

## Implementation notes (departures disclosed)

- **The populations:** passages are counted over runs without a run error, the population of the summary's gate-error column. Pairs need the same seed, lap and gate in both runs, not two successful runs.
  - The plan said "successful seeds". With every `posit<16,1>` seed succeeding, the H4 numbers are the same either way.
  - The smoke scenario is shorter than two laps, so with "successful seeds" it would have had no pairs, and criterion 5 couldn't be checked.
- **Criterion 3** is checked in the smoke test, against the passages in that run's JSONL line.

## Out of scope

- **Why `posit<16,1>`'s excess grows along each straight** (assessment §9): the table shows it, but its cause isn't established here. Filed as [#44](https://github.com/branes-ai/racing-drone/issues/44).
