---
status: complete
issue: branes-ai/racing-drone#50
---
# Plan: a history of autonomous drone racing

Issue: #50. Implemented in #51.

## Goal

The context for why this repo uses drone racing to push autonomy forward: what the field set out to solve, which results and algorithms solved which problem and when, the leagues that measured it (the IROS Autonomous Drone Racing competitions and the Abu Dhabi Autonomous Racing League, A2RL), and the insights that carry over to our baselines. `docs/assessments/state-of-the-art.md` is a component-level survey for choosing baselines; this document is the history and the argument behind it, and the survey links to it for background.

## The document

`docs/assessments/autonomous-drone-racing.md`:

1. **Why racing.** What a race forces on an autonomy stack (perception, estimation, planning and control at the vehicle's limits, on small onboard compute), and why it is a good benchmark: an unambiguous score, repeatable courses, and human pilots to compare against.
2. **Timeline, by problem solved.** From the first gate-racing work and the IROS ADR era, through AlphaPilot and Game of Drones, time-optimal planning, learned policies and the sim-to-real gap (Swift), one-camera racing (MonoRace), to multi-drone racing and collisions (A2RL 2026).
3. **The leagues.** IROS ADR (each edition: venue, format, rules, winners and their approaches) and A2RL (organizer, the drone championships, hardware, sensor rules, formats, results), with AlphaPilot and Game of Drones between them.
4. **Key results and algorithms**, as a table: problem, result, method, venue and year.
5. **Insights**, as the authors state them, and what is still unsolved.
6. **What this means for this repo**: how the insights map onto the baselines, ADRs and requirements.

## Evidence rules

- Every fact cites a source (DOI, arXiv, or an official page), with the access date for web pages.
- Numbers are quoted as the source states them.
- Our own judgments are marked *(assessment)*, as in the state-of-the-art survey.
- A fact that could not be verified is left out or marked unverified, never guessed.

## Acceptance

1. Every section above is present, and every factual claim carries a citation.
2. Where this document and `state-of-the-art.md` or `mission-requirements.md` state the same fact, they agree; any correction to those documents is made in this PR and listed below.
3. The docs site builds with the new page and no broken links.

## Result

- The document follows the six sections above, with 51 numbered sources; every number cited in the text is in the list, and every source is cited.
- No fact in it contradicts `state-of-the-art.md` or `mission-requirements.md`, so neither needed a correction. The survey now links to it for background. (The requirements' 76 × 18 × 5.4 m is MonoRace's track volume; the hall is 100 × 30 m. Both are right.)

## Departures

- **Facts left out rather than marked.** The research turned up claims that could not be confirmed from a primary or official source: the IROS 2018 runner-up, IROS 2019's rules and the winner's approach, the NeurIPS 2019 Game of Drones winners, the two other human pilots MonoRace beat in 2025, and Swift's exact race times. The document leaves them out, except where it says so (the IROS 2019 approach).
- **Conflicting sources are shown, not resolved.** AlphaPilot's times differ between MAVLab, UZH, Lockheed Martin and the press. The A2RL 2026 "benchmark lap time" of 12.032 s is slower than the laps one team reports for itself. Both values are given, each with its source.
- **Some facts rest on secondary sources**, which the evidence rule does not allow on its own. The date of A2RL's first car race comes from Wikipedia. Team FlyBy's lap times come from the team's own pages. One of AlphaPilot's conflicting times comes from HeroX, the challenge's host platform. Each is attributed in the text, and none is the only source for a result the document relies on. The press-only claim that AlphaPilot's human-beating bonus went unwon was replaced with Lockheed Martin's own account of that race.
- **The A2RL drone rulebook is still not public.** The rules come from organizer press material and team reports, as the document says, and the open action in the requirements stays open.
