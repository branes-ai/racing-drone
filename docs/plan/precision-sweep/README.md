# Precision sweep of baseline B0

**Status:** Approved 2026-10-27
**Issue:** [racing-drone#30](https://github.com/branes-ai/racing-drone/issues/30), part of the Phase 2 epic [racing-drone#9](https://github.com/branes-ai/racing-drone/issues/9)
**Related:** [mixed-precision assessment](../../assessments/mixed-precision.md) (H1–H5), [B0 plan](../b0/README.md), [SITL plan](../sitl/README.md), cortex ADR 0015

## Issues and pull requests

| Item | What | State |
|---|---|---|
| [racing-drone#30](https://github.com/branes-ai/racing-drone/issues/30) | This plan | Open |
| [racing-drone#31](https://github.com/branes-ai/racing-drone/pull/31) | The assessment that derives H1–H5 | Merged |
| [racing-drone#17](https://github.com/branes-ai/racing-drone/issues/17) | B0 and gate P1, which the sweep reuses | Closed |

## Problem

Every harness builds `Composition<double>`, so no closed-loop run has used any other scalar type, although the stack, the slots and the message types are templated on `T`. The assessment predicts which types fly B0 through gate P1 (H1–H4). This plan measures it: B0 runs in each type, with the simulator, the simulated FC, the scorer and the log in `double`.

## What a probe found before planning

A throwaway program built B0 in all seven types with gcc and ran two ticks of each (not committed; it shaped the design below):

1. **All seven types compile.** `GroundTruthEstimator<T>`, `WaypointPlanner<T>` and `glue::CascadedPid<T>`, with reflex's cascade underneath, need no source change. reflex tests its cascade in `double` and `float` only, so this is the first instantiation in Universal types.
2. **binary16 fails construction, as H3 predicts.** The message is reflex's generic "does not give valid reflex model parameters", which doesn't say which parameter failed. H3's verdict needs the reason.
3. **`posit<16,1>` aborts on the course itself.** `is_unit` checks `|‖q‖² − 1| ≤ 1e-6` in every type. Gates 6 and 11 have orientation (0.7071, 0, 0, ±0.7071), and in `posit<16,1>` the cast gives ‖q‖² − 1 = 2.4e-4. bfloat16 and binary16 pass on this course only because their rounding of 0.7071 happens to square to exactly 0.5. A truth attitude in flight is an arbitrary unit quaternion, so every 16-bit type would trip it.

## Plan

### 1. Universal (`cmake/deps.cmake`)
- Universal 5.11.0, the pin reflex and cortex use, fetched header-only like reflex's test-only fetch, and defining `universal::universal` only if no one has yet.
- Linked only by the sweep tool and the tests. The SDK, slots and glue never name a Universal type (as in reflex).

### 2. A unit tolerance derived from `T` (`msg/geometry.hpp`)
`is_unit`'s default tolerance becomes `unit_tolerance<T>() = max(1e-6, 4ε_T)`, where ε_T is `numeric_limits<T>::epsilon()`.

Derivation, for a quaternion that is unit in `double` (its ‖q‖² − 1 is a few ε_64, negligible) cast to `T` and checked in `T`, with unit roundoff u = ε_T/2 (round to nearest):
- the cast gives q̂ᵢ = qᵢ(1 + δᵢ), |δᵢ| ≤ u, so ‖q̂‖² = Σqᵢ²(1 + δᵢ)² is within 2u + u² of 1;
- `norm_squared` in `T` is a 4-term sum of products of non-negatives, within γ₄ = 4u/(1 − 4u) of its exact value;
- `‖q̂‖² − 1` is then exact (Sterbenz, since the sum is in [½, 2]);
- in total |computed − 1| ≤ (1 + γ₄)(1 + u)² − 1 = 6u + O(u²), under 8u = 4ε_T for u ≤ 2⁻⁸ (every type in the sweep).

Posits have u only for |x| in [1/useed, useed). A component below 1/useed loses a bit per regime step, but its square shrinks by useed per step, so with useed ≥ 4 each such component adds at most u/4 to the bound: 6.75u + O(u²) in the worst case, still under 8u. A test checks the bound on 10,000 random unit quaternions per type.

For `double` and `float`, 4ε_T (8.9e-16 and 4.8e-7) is below 1e-6, so the tolerance, and so every existing check and log, is unchanged. For bfloat16 it is 3.1e-2, binary16 3.9e-3, `posit<16,1>` 9.8e-4. A looser check is the honest one: a format cannot certify unit norm more finely than it can represent it.

### 3. Conversions at the harness boundary (`msg/*.hpp`)
- `cast<To>(const Msg<From>&)` for `ImuSample`, `TruthState`, `StateEstimate`, `Reference`, `ThrustRates` and `GateObservations`, next to each type, following `CourseMap`'s existing cast: element by element, non-scalar fields copied.
- `GateObservations` doesn't cross the harness today (the detector feeds the estimator inside the stack). Its cast is added because the issue lists it, for the detector logs of later baselines.

### 4. The Runner in `T` (`harness/harness.hpp`)
- `Runner<StackT>` takes `T = StackT::scalar_type`. It casts decoded inputs (`double`) to `T` and the tick's outputs back to `double` before encoding. The log stays `double`.
- After the cast it checks `valid()` in `T` and reports a violation as an error, as it already does for the decoded `double`. Section 2 makes this unreachable for unit inputs; the check keeps a violation from reaching the stack's asserts.
- For `T = double` every cast is a copy, so outputs and logs are byte-identical.
- `make_stack<StackT>` casts the course to `T` and returns an error, not an assert, if `valid(cast<T>(map))` fails.

### 5. A parameter-level reason for construction failures (`glue/reflex_params.hpp`)
When reflex's validator rejects the parameters, `reflex_params_from<T>` appends every nonzero vehicle parameter whose conversion to `T` is zero, subnormal (below `numeric_limits<T>::min()`) or not finite. For RP-1 in binary16 it reads "rotor.thrust_coefficient = 8.9e-07 is subnormal in this scalar type". It only explains a rejection: what is accepted is still reflex's decision, and reflex's own message is unchanged.

### 6. One batch runner for `sitl_batch` and the sweep (`sim/batch.hpp`)
`run_one` moves out of `apps/sitl_batch/main.cpp` into a header, templated on the stack type, with the JSON line it writes and the percentile and failure-grouping helpers. `sitl_batch` calls it with `Composition<double>` and its output is unchanged. The sweep calls the same code, so for `double` its lines are `sitl_batch`'s by construction, and a test checks it (AC 1).

### 7. The tool, `precision_sweep_b0-gt-pid` (`apps/precision_sweep/main.cpp`)
- Built by `racing_add_baseline(b0-gt-pid PRECISION_SWEEP)`, the only baseline that opts in.
- `--scenario <yaml> --seeds <n> --out-dir <dir> [--first-seed <s>] [--jobs <j>] [--min-success <f>] [--commit <text>] [--timing]`.
  - `--min-success` (default 0.99, gate P1) only marks the table's P1 column.
  - `--commit` is recorded in the table as given. The tool can't know the commit it was built from without a build step that goes stale, so the run that produces committed results passes `$(git rev-parse HEAD)`.
  - `--timing` adds each type's wall time. It is off by default because it makes the files differ between runs (AC 3).
- Types, a compile-time list (`sim/sweep_types.hpp`, shared with the tests) in this order: `double`, `float`, `cfloat<32,8>`, `posit<32,2>`, bfloat16 (`cfloat<16,8>`), binary16 (`cfloat<16,5>`), `posit<16,1>`. The `cfloat`s have subnormals, no max-exponent values, no saturation (IEEE behavior).
- Per type: construct once. A failure is the type's result, `construction_failed` with its reason, and the tool moves on (AC 4). Otherwise run the seeds, as `sitl_batch` does.
- A run that errors (for example a command that doesn't decode because it is NaN) is that seed's result, `run_error` with its reason. It doesn't stop the tool.
- Files in `--out-dir`:
  - `<type>.jsonl`: one line per seed, in `sitl_batch`'s format (`{"seed":…,"score":…}`), or `{"seed":…,"run_error":"…"}`.
  - `lap_delta.jsonl`: per type and seed, each lap's time minus `double`'s, for seeds where both succeeded.
  - `summary.jsonl`: one line per type: construction status and reason, success count and rate, failures by reason, lap time p50/p95, true gate error p50/p99, max speed, and the lap-time difference from `double` (p50/p95 of |Δ|).
  - `summary.md`: the markdown table, generated from the summary, with the command line (without `--jobs` and `--out-dir`, which change no result), the commit and the seed range, then every failure reason.
- Exit 0 once every type has a result. Nonzero only on usage or setup errors.

### 8. Results
- Run once with 500 `b0-nominal` seeds. The generated `summary.md` is committed as `docs/assessments/data/precision-sweep-b0.md`, unedited, with `summary.jsonl` and `lap_delta.jsonl` beside it.
- The assessment page gets a Results section that cites the table and gives H1–H4 each a verdict against its prediction. Numbers in the prose come from the table, by reference.

### 9. CI
`sweep.b0.smoke`, on every PR: the tool on one `b0-determinism` seed, twice. A CMake script checks that every type has a summary line with a status, that the two runs' files are byte-identical (AC 3), and that `double.jsonl` equals `sitl_batch_b0-gt-pid --jsonl` for the same seed (AC 1).

## Model and contracts

### `Runner<StackT>` (stateful)
- Members: `stack_` (StackT&, non-owning), `last_tick_` (optional<Timestamp>). Unchanged.
- Invariant I1: `last_tick_` never decreases. Unchanged.
- `feed(rec)`: Pre `!is_output(rec.channel)`. Post: an input that doesn't decode, or isn't `valid()` after the cast to `T`, gives an error and leaves the stack unfed; a tick gives the three outputs, each `cast<double>` of the stack's. Preserves I1.

### `cast<To>(m)` (stateless)
- Post: each scalar field is `To(field)`; integers, timestamps, enums and bools are copied. For `To == From`, `cast(m) == m`.
- Round trip (AC 2): `|double(To(x)) − x|` is no larger than the distance from x to either neighbor of `To(x)` in `To`, the definition of round to nearest. The test checks the scalar conversion against the actual neighbors, found with `nextafter` for built-in types and the encoding increment for Universal types, so no tolerance is chosen. A second test checks that each message cast is exactly that conversion, field by field.
- Posits never round a nonzero value to zero (the posit standard): below minpos the result is ±minpos. There, and only there, the error exceeds ½ ULP; the test applies the posit rule (zero isn't a candidate).

## Acceptance criteria (from the issue, stated before implementation)

1. **Identity for double:** the sweep's `double.jsonl` equals `sitl_batch`'s `--jsonl` for the same seeds. `gate.p0.replay_determinism`, `gate.sitl.determinism` and `gate.b0.determinism` pass unchanged, and their logs are byte-identical to main's.
2. **Boundary casts:** `double → T → double` is exact for `T = double`, and within the round-to-nearest bound per field for the other types, on representative values for every type.
3. **Determinism:** two runs with the same arguments give byte-identical files for every type.
4. **Construction failures are results:** reported as `construction_failed` with the reason; the tool continues and exits 0.
5. **Build:** every type builds with warnings as errors on gcc, clang and MSVC, or the dropped type is listed in the PR with the compiler error.
6. **No hand-typed numbers:** the table is generated, and the assessment cites it.
7. **Verdicts:** H1–H4 each get a verdict from the generated table.

## Risks

- **Debug-build asserts.** A 16-bit type can drive a slot to NaN, and the slots' contract asserts would then abort a Debug build. The Runner's post-cast checks catch bad inputs, not bad outputs. If the smoke test aborts in a Debug preset, the fix is a checked return in the slot (reported as `run_error`), never a weaker assert; any such change is listed in the PR.
- **Runtime.** A `double` seed of `b0-nominal` takes about 0.1 s (gcc-release, one run). Emulated types are slower by an unmeasured factor; the measured sweep time goes in the results file.
- **MSVC and Universal.** Not yet built on MSVC in this repo. A toolchain failure is a disclosed departure (AC 5).

## Out of scope
Per-slot scalar types and H5 (the follow-up); the simulator, FC, scorer or log in anything but `double`; speed or energy on hardware.
