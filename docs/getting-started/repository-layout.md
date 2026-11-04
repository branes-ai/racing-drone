# Repository Layout

```text
racing-drone/
├── apps/         # harness sources (flight_stack, replay, sitl, sitl_batch, precision_sweep) built per baseline and per graph build, and the score / log_dump tools
├── baselines/    # one YAML per baseline: slot composition, the graph's class and bodies, parameters
├── cmake/        # deps, warnings, slot and operator registries, baseline target generation
├── courses/      # course files (gates, sequence, laps, geofence)
├── docs/         # requirements, assessments, architecture, plans, ADRs: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── scenarios/    # SITL scenarios (course + vehicle + duration + seed)
├── sdk/          # branes::racing: data contracts, slots, Stack, the graph (flow/), config loaders, log, harness pieces
├── sim/          # SITL: the simulator on the reflex dynamics, the simulated FC and sensors, the scoreboard
├── tests/        # Catch2 unit tests, baseline tests, phase gates, graph gates
└── vehicles/     # vehicle files (mass, inertia, rotors, drag, IMU, camera)
```

The mission-neutral framework (the data contracts, slots and composition, configs, the log, the harness, the reflex glue, and the closed-loop simulator) is not in this repo: it is [anatomy](https://github.com/branes-ai/anatomy), fetched at a pinned release (`cmake/deps.cmake`). `sdk/include/branes/racing/anatomy*.hpp` bring its names into `branes::racing` ([plan](../plan/consume-anatomy/README.md)).

Each baseline with a `graph:` also builds as a graph: `sdk/include/branes/racing/flow/` describes racing's stack as a graph of seven composites, which anatomy checks and runs on its CPU stack. Its builds are `replay_<name>_graph`, `sitl_<name>_graph` and `sitl_batch_<name>_graph`, and the `gate.<name>.graph` tests hold them equal to the slot builds ([plan](../plan/flow-graph/README.md)).

## Where the documents are

| Folder | Contents |
|---|---|
| [`requirements/`](../requirements/mission-requirements.md) | The mission profile, the reference vehicle, the requirements, and the derived budgets |
| [`assessments/`](../assessments/autonomous-drone-racing.md) | The history of the field, the state of the art, and the mixed-precision study |
| [`arch/`](../arch/framework-architecture.md) | The framework architecture |
| [`plan/`](../plan/implementation-plan.md) | The implementation and test plans, and one plan per issue |
| [`guides/`](../guides/visual-studio.md) | Walkthroughs |
| [`adr/`](../adr/README.md) | Architecture decision records |

The [documentation map](../README.md) lists every document.
