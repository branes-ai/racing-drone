# racing-drone docs

| Folder | Contents |
|---|---|
| [requirements/](requirements/mission-requirements.md) | Mission profile, reference vehicle, requirements (SYS, EST, CTL, FPV, DAA, SAF, SW), derived budgets |
| [assessments/](assessments/state-of-the-art.md) | [State of the art](assessments/state-of-the-art.md) in autonomous racing and DAA, with sources, and the resulting baselines; [mixed precision](assessments/mixed-precision.md): what precision each component needs, the hypotheses the precision sweep tests, and why MTL5 and Universal |
| [arch/](arch/framework-architecture.md) | Framework architecture: slots, data contracts, baselines, data flow, modes, repo layout |
| [plan/](plan/implementation-plan.md) | [Implementation plan](plan/implementation-plan.md) and [test plan](plan/test-plan.md); per-issue plans, each with its issues and PRs: [SITL harness](plan/sitl/README.md), [precision sweep](plan/precision-sweep/README.md), [time base](plan/time-base/README.md), [gate error by gate](plan/gate-error-by-gate/README.md) |
| [guides/](guides/visual-studio.md) | [Working through the stack in Visual Studio](guides/visual-studio.md): one closed loop, its log, many seeds, the precision sweep, and the tests |
| [adr/](adr/README.md) | Architecture decision records |
| sessions/ | Dated session records (`YYYY-MM-DD-slug.md`) |
