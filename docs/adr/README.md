# Architecture decision records

Format follows cortex `docs/adr/template.md`: Status / Date / Related, Context, Decision, Consequences, and a closing *Current state* line.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-baseline-composition-framework.md) | Racing drones are baselines composed into fixed slots | Proposed |
| [0002](0002-companion-mpc-fc-rate-loop.md) | MPC on the companion, INDI + rate loop and every failsafe on the FC | Proposed |
| [0003](0003-mpc-primary-learned-policy-slot.md) | MPCC primary; a learned policy as a comparison baseline | Proposed |
| [0004](0004-gate-relative-hybrid-estimation.md) | Hybrid VIO + gate PnP + drag-aware model estimation | Proposed |
| [0005](0005-layered-daa.md) | Layered DAA: MPC constraints plus an independent CBF filter | Proposed |
| [0006](0006-hot-path-in-process-zenoh-off-path.md) | Hot path in one process; Zenoh off the hot path | Proposed |
| [0007](0007-outdoor-obstacle-avoidance-placement.md) | Outdoor obstacle avoidance in the platform repos and its own mission repo; the framework lifted out before a second mission uses it | Proposed |
