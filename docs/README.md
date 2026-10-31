# Documentation map

These documents are published at <https://branes-ai.github.io/racing-drone/>. The site is generated from this folder on every merge to main ([plan](plan/docs-site/README.md)): edit the documents here, never the site. [`index.mdx`](index.mdx) is the site's landing page, and this file is its documentation map.

| Folder | Contents |
|---|---|
| [getting-started/](getting-started/introduction.md) | [Introduction](getting-started/introduction.md), [build & test](getting-started/build-and-test.md), [repository layout](getting-started/repository-layout.md) |
| [requirements/](requirements/mission-requirements.md) | Mission profile, reference vehicle, requirements (SYS, EST, CTL, FPV, DAA, SAF, SW), derived budgets |
| [assessments/](assessments/state-of-the-art.md) | [Autonomous drone racing](assessments/autonomous-drone-racing.md): why racing, its history from the IROS competitions to A2RL, the key results by problem solved, and the insights behind our baselines; [state of the art](assessments/state-of-the-art.md) in autonomous racing and DAA, with sources, and the resulting baselines; [mixed precision](assessments/mixed-precision.md): what precision each component needs, the hypotheses the precision sweep tests, and why MTL5 and Universal; [framework boundary](assessments/framework-boundary.md): which parts of the framework are mission-neutral, before they move to anatomy |
| [arch/](arch/framework-architecture.md) | Framework architecture: slots, data contracts, baselines, data flow, modes, repo layout |
| [plan/](plan/implementation-plan.md) | [Implementation plan](plan/implementation-plan.md) and [test plan](plan/test-plan.md); per-issue plans, each with its issues and PRs: [SITL harness](plan/sitl/README.md), [precision sweep](plan/precision-sweep/README.md), [time base](plan/time-base/README.md), [gate error by gate](plan/gate-error-by-gate/README.md), [docs site](plan/docs-site/README.md), [docs-site landing page](plan/docs-site-landing/README.md), [drone racing history](plan/drone-racing-history/README.md), [framework split](plan/framework-split/README.md), [consuming anatomy](plan/consume-anatomy/README.md) |
| [guides/](guides/visual-studio.md) | [Working through the stack in Visual Studio](guides/visual-studio.md): one closed loop, its log, many seeds, the precision sweep, and the tests; [watching a race replay](guides/mission-replay.mdx): a B0 run played in reflex's trace player, and how to export and play your own |
| [adr/](adr/README.md) | Architecture decision records |
| sessions/ | Dated session records (`YYYY-MM-DD-slug.md`) |

The plans and the decisions are published on the [architecture site](https://branes-ai.github.io/architecture/racing-drone/), alongside every other repo's plans and decisions and their status, rather than here.
