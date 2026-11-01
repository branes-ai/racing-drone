# ADR 0009: third-party license compatibility

**Status:** Accepted (amended 2026-11-01: published facts)
**Date:** 2026-10-31
**Source:** the stack's license posture, set in cortex's [ADR 0001](https://github.com/branes-ai/cortex/blob/main/docs/adr/0001-third-party-license-compatibility.md), which `cmake/deps.cmake` already cites
**Related:** [ADR 0010](0010-no-gpl-clean-room-policy.md)

## Context

racing-drone is MIT-licensed (see the top-level [`LICENSE`](../../LICENSE)), like the rest of the Branes.AI stack. It is a mission repo: its baselines, worlds and scorers run on the platform repos and on anatomy. It is meant to be redistributed inside partners' products, so every dependency has to be compatible with MIT and carry its notices. cortex records this posture in its ADR 0001, and `cmake/deps.cmake` here says "permissive licenses only" and points there. This ADR records it in racing-drone itself.

## Dependency license matrix

| Dependency | License | Compatible with MIT racing-drone? | How it is used | Obligation |
|---|---|---|---|---|
| yaml-cpp | MIT | Yes | FetchContent: the vehicle, course, scenario and baseline files | Bundle its LICENSE in a distribution |
| reflex | MIT | Yes | FetchContent: the SITL plant and the simulated flight controller. It brings MTL5 (MIT) | Bundle its and MTL5's LICENSE in a distribution |
| anatomy | MIT | Yes | FetchContent: the mission framework (ADR 0008) | Bundle its LICENSE in a distribution |
| Universal | MIT | Yes | FetchContent: the precision-sweep types; only the sweep tool and its tests link it | Bundle its LICENSE in a distribution |
| Catch2 v3 | BSL-1.0 | Yes | FetchContent, tests only | None for object code; bundle with source |

The license is the dependency's however it arrives: a build that supplies its own copy (an installed package, a parent project's target) must supply one under the same license.

**A mission's data is someone's work too.** Worlds, recorded flights, datasets and trained policies made by this repo are its own. A third-party dataset or set of pretrained weights may be committed or fetched only under a license this table records; data without one, or under a non-commercial or share-alike license (CC BY-NC, CC BY-SA, …), stays outside the repo. A dataset's published facts are not its data: calibration parameters, sensor specifications and results printed in a paper may be written into the repo's own files, with a citation to their source. A file copied from a dataset is the dataset's, and follows the rule above.

No GPL, LGPL, AGPL, MPL or SSPL dependency is accepted. [ADR 0010](0010-no-gpl-clean-room-policy.md) is the wider policy: no GPL code for any purpose. An exception would need an ADR of its own that supersedes this one.

## Decision

1. **racing-drone stays MIT-licensed.** Every dependency must be compatible with MIT.
2. **A PR that adds a dependency must add it to this table**, with its license and its obligation. A dependency that is dual-licensed is used under its permissive branch.
3. **Any file of someone else's work committed to racing-drone** (copied, ported or generated from a third-party source) is listed, with its license text, in a top-level `THIRD_PARTY_NOTICES.md`, as synapse does. The PR that adds the first such file creates it.
4. **Third-party data and weights** are committed or fetched only under a license recorded in the table above; the rest stays outside the repo. Published facts (calibrations, specifications, a paper's results) may be quoted, with a citation.

## Consequences

- This table answers a partner's license review: what racing-drone links, and under which terms.
- **Scope.** This covers what racing-drone ships, the code that builds it and the tests. The docs site's toolchain is not part of it. Astro brings in sharp (Apache-2.0) to process images, and sharp brings prebuilt libvips binaries, the `@img/sharp-libvips-*` packages (LGPL-3.0-or-later). They only build the documentation, in CI and on developers' machines.
- **Current state:** racing-drone's dependencies are those in the table. It contains no third-party code and no third-party data, so it has no `THIRD_PARTY_NOTICES.md` yet. No release artifact bundles the dependencies' license files.
