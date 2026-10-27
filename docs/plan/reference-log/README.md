# Planner reference in the log

**Status:** Approved 2026-10-26, in implementation
**Issue:** [racing-drone#25](https://github.com/branes-ai/racing-drone/issues/25), part of the Phase 1 epic [racing-drone#8](https://github.com/branes-ai/racing-drone/issues/8)
**Related:** [test plan](../test-plan.md) §9, [architecture](../../arch/framework-architecture.md), [B0 plan](../b0/README.md), [ADR 0006](../../adr/0006-hot-path-in-process-zenoh-off-path.md)

This plan records the planner's `Reference` as a log channel, so a run shows what the stack was trying to do, not only what it believed and commanded.

## Issues and pull requests

| Item | What | State |
|---|---|---|
| [racing-drone#25](https://github.com/branes-ai/racing-drone/issues/25) | This plan | Open |
| [racing-drone#26](https://github.com/branes-ai/racing-drone/pull/26) | The implementation; this document lands with it | Open |
| [racing-drone#23](https://github.com/branes-ai/racing-drone/pull/23) | `log_dump`, which gains `reference.csv` here | Merged |

## Problem

A log records the stack's state estimate (`state`, channel 16) and its command (`command`, channel 17), but not the planner's **reference**: the target the controller was asked to track. `Stack::on_tick` builds the reference and passes it to the controller (`sdk/include/branes/racing/compose/stack.hpp`, `Stack::on_tick`), but `TickOutput` carries only `{state, command}`, so the reference never reaches the log.

What this costs:
- A run can't be split into "the controller didn't track the reference" and "the planner aimed badly". Plotting reference against true position is the first diagnostic for tuning B0, and for comparing B1 with it.
- It is a gap against [test plan](../test-plan.md) §9 ("timestamped channels for all slot inputs and outputs") and against SYS-08.
- `replay --verify` checks the planner only indirectly, through the command. A non-deterministic planner whose effect the controller happens to mask would go unnoticed.

It follows [racing-drone#23](https://github.com/branes-ai/racing-drone/pull/23) (`log_dump`), which can't show the reference for this reason.

## Plan

### 1. Message and codec (`sdk/include/branes/racing/msg/codec.hpp`)
- `encode(const Reference<double>&)` and `decode_reference(span)`.
- Wire format: `count` u32, then only the `count` used samples. Each sample is `t` i64 followed by 11 f64: position, velocity and acceleration (3 each), yaw and progress. That is 8 + 11 × 8 = 96 bytes per sample.
- Decoding rejects:
  - `count > kMaxReferenceSamples`;
  - a non-finite value;
  - trailing bytes;
  - anything failing `valid(Reference)`: I2 (strictly increasing sample times) or I3 (non-decreasing progress).
- This matches how the other decoders treat a log as untrusted input.

### 2. Log channel (`sdk/include/branes/racing/log/log.hpp`)
- New output channel `reference = 18`, and `is_known()` accepts it.
- **`kLogVersion` 1 → 2.** The reader already rejects any version it doesn't know, so a v1 log gets "unsupported log version 1".
  - The repo stores no `.brlog` fixtures (checked), and every gate re-records its logs, so the only cost is re-recording local logs.
  - Rejected alternative: keep v1 readable and have `--verify` skip channels absent from the recording. That weakens the verification for every log: a log that lost a channel would still verify.

### 3. Stack and harness
- `TickOutput<T>` becomes `{state, reference, command}`, in pipeline order, and `on_tick` returns the reference it computed.
- `Runner::feed` emits three outputs per tick, in the order state, reference, command. Order matters because the gates compare logs byte for byte.
- `replay --verify` needs no change: it compares every output record, so it now also checks the reference.
- Call sites that build or read `TickOutput`: `tests/sdk/stack.cpp`, `tests/baselines/composition_hash.cpp`, `apps/flight_stack/main.cpp` (which discards it).

### 4. `log_dump`
- Writes `reference.csv`, one row per sample, with these columns:
  - `t_ns,t_s,count,index,sample_t_ns`
  - `p_x..p_z,v_x..v_z,a_x..a_z,yaw,progress`
- A tick with `count == 0` contributes no row, as in `oracle_gates.csv`.

### 5. Docs
- This plan document, landing with the implementation PR.
- The architecture doc gets a sentence that the log carries the reference.
- The `log_dump` header comment lists the new file.

## Size (derived from the format, not measured)
- A record header is 14 bytes (u16 + u32 + i64).
- One sample costs 4 + 96 = 100 payload bytes, so 114 bytes per tick. Every current planner (null, scripted-hop, B0's `WaypointPlanner`) emits exactly one sample.
- At B0's 250 Hz that is 28.5 kB/s.
- A planner filling all 64 samples would write 4 + 64 × 96 + 14 = 6,162 B per tick, or 1.54 MB/s at 250 Hz. That fits SITL. For flight it goes into the ADR 0006 ring sizing when such a planner exists. Decimating the reference is a later, separate decision.

## Acceptance criteria (stated before implementation)
1. **Codec round-trip:** `decode_reference(encode(r)) == r` for `count` 0, 1 and 64. Decoding rejects:
   - `count = 65`;
   - a NaN in any field;
   - decreasing sample times;
   - decreasing progress;
   - one trailing byte;
   - a payload truncated by one byte.
2. **Stack:** the `reference` returned by `on_tick` equals `planner().plan(course(), state)` for the same tick.
3. **Runner:** each tick yields exactly three outputs, with channels state, reference and command in that order, all stamped with the tick time.
4. **Record count:** a `b0-determinism` SITL log has exactly `ticks` more records than before. Before: 2000 ticks and 23,442 records. Expected after: 25,442. This is checked in `tools.log_dump` by reading SITL's printed counts: `records_after = records_before + ticks`.
5. **Existing gates pass unchanged:** `gate.p0.replay_determinism`, `gate.sitl.determinism`, `gate.b0.determinism` and `gate.p1.smoke`. The determinism gates now cover the reference, because they compare whole logs.
6. **`log_dump`:** `reference.csv` has one row per tick on the B0 log, and its rows match its header.
7. **Old logs:** opening a version-1 log fails with "unsupported log version 1".
8. Linux gcc and clang (Debug and Release) and MSVC (Debug and Release) build with warnings as errors, and all tests pass.

## Out of scope (possible follow-ups)
SYS-08 and test-plan §9 ask for **every** slot output. After this change, these remain unlogged:
- the gate detector's `GateObservations` (only the simulator's oracle gates are logged);
- the controller's command before the safety filter (the logged `command` is after it);
- the DAA conflict set;
- estimator health.

Each is a separate channel with its own size trade-off, and can be its own issue.

## Implementation notes

1. **Criterion 4 is checked differently from the issue's wording.** The issue said `tools.log_dump` would check `records_after = records_before + ticks`. A test can't run the pre-change binary, so it checks the equivalent property that a test can see: the `reference` channel holds exactly `ticks` records, and it is the only new channel. By hand, on `b0-determinism` with seed 1700, the SITL record count went from 23,442 on main to 25,442 with this change, which is 2,000 ticks more.
2. **Criterion 2 is tested on two planners.**
   - `NullPlanner`: its `plan()` is `const`.
   - B0's stateful `WaypointPlanner`: the test copies the planner just before the second tick, and checks that the copy's `plan()` on the same state gives the reference the tick returned.
3. **Decoding is canonical.** `decode_reference` value-initializes the unused samples. `Reference`'s `operator==` compares all 64 slots, so a reference with leftover data past `count` would not compare equal after a round trip. Every planner in the repo builds its reference from `Reference<T>{}`, so none has leftover data.
4. **`TickOutput` grows** by `sizeof(Reference<double>)` = 8 + 64 × 96 = 6,152 B (the `count` plus padding, then the samples), returned by value on each tick. The planner already builds this value on the stack, and the copy happens off the FC path. If a profile ever shows it, the fix is to return the planner's output by reference.
5. **The size figures were corrected in review.** The approved plan said a sample holds 12 doubles (104 B). The fields are 3 + 3 + 3 + 1 + 1 = 11 doubles, so a sample is 96 B. The code and its test (`codec: Reference round trip`) always used 96 B. The size section above now shows the corrected figures, and so does the #25 body.
6. **`log_dump`'s one-row-per-sample rule has its own test.** Every current planner emits one sample, so a SITL log can't tell one row per sample from one row per record. `tests/tools/reference_log_fixture.cpp` writes references of 0, 1 and 3 samples. `tools.log_dump` requires 3 records, 4 rows, and the (count, index) pairs (1,0), (3,0), (3,1), (3,2). It fails against an exporter that writes one row per record (checked).
