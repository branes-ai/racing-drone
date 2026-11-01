# ADR 0010: no GPL code; clean-room implementation from papers

**Status:** Accepted
**Date:** 2026-10-31
**Source:** the stack's no-GPL policy, set in cortex's [ADR 0002](https://github.com/branes-ai/cortex/blob/main/docs/adr/0002-no-gpl-clean-room-policy.md), which `cmake/deps.cmake` already cites
**Related:** [ADR 0009](0009-third-party-license-compatibility.md)

## Context

cortex's ADR 0002 rules out GPL code anywhere in cortex, for any purpose. The reason applies to racing-drone unchanged: partners redistribute the stack inside their own certified products, and any GPL obligation, even one confined to a test, is something their legal reviews would have to clear.

In drone racing the pull towards GPL code is strong. Much of the published research code is GPL-licensed, and so are the flight stacks racing quadrotors usually fly:
- UZH's racing and agile-flight code: `rpg_mpc` and `rpg_quadrotor_mpc` (MPC), `rpg_time_optimal` (time-optimal planning) and `agile_autonomy` ("Learning high-speed flight in the wild"), all GPL-3.0
- Betaflight, INAV and Cleanflight (GPL-3.0)
- ArduPilot (GPL-3.0)
- Paparazzi (GPL-2.0)

Permissive references exist too: PX4 (BSD-3-Clause), and UZH's `agile_flight` and `deep_drone_acrobatics` (MIT). The baselines' designs (MPCC, INDI, the gate-relative estimator, ADRs 0002 to 0004) come from papers.

## Decision

**No GPL-licensed code anywhere in racing-drone, for any purpose.** Not as a dependency, not in a test target, not as a build-time tool in what racing-drone builds, and not as a reference oracle. The scope is ADR 0009's: what racing-drone ships, the code that builds it and the tests, and not the docs site's toolchain.

- Everything is implemented **clean-room from papers, textbooks and specifications**, plus permissively licensed (MIT / BSD / Apache-2.0) references.
- **Check a reference's license before opening its code.** Reading GPL source and then writing the same thing "from scratch" is not clean-room. A GPL project's structure may be consulted, meaning its directory layout and how it splits the problem into modules, and nothing else: never its code, its algorithms' details, or its data, which includes tuning tables, gains, parameters and trained weights.
- **Test oracles are in-house or permissive:** racing-drone's own simulator and courses, recorded flights it made itself, results published in papers, and permissive code such as PX4. Never compare against the output of a GPL program (the GPL-3.0 UZH code above, Betaflight, ArduPilot, …), its simulator or its logs.
- **Conventions are not code.** Following a convention that a GPL project also uses (a frame, a rotor order, a unit) is fine, when it is taken from a permissive source or a specification.
- **LGPL, MPL, AGPL and SSPL code is rejected,** as a dependency and as a reference, as ADR 0009 rejects it. An exception would need an ADR of its own that supersedes this one.

## Consequences

- Each component costs more to build: the design comes from the literature, and validation uses racing-drone's own simulation and permissive references.
- A paper's results can be compared against; its GPL code cannot be read or run.
- Partners can redistribute racing-drone under MIT terms with no copyleft review.
- **Current state:** racing-drone's baselines, worlds and scorers are written in-house from papers. No GPL code or oracle is used, and no learned policy or third-party dataset is in the repo yet.
