# ADR 0007: outdoor obstacle avoidance lives in the platform and its own mission repo, not in racing-drone

**Status:** Proposed
**Date:** 2026-10-28
**Related:** [ADR 0001](0001-baseline-composition-framework.md), [ADR 0005](0005-layered-daa.md); requirements §2, SYS-03, DAA-09; the Branes.AI tiers in the [introduction](../getting-started/introduction.md#how-the-platform-fits-together)

## Context

Loquercio, Kaufmann et al., "Learning high-speed flight in the wild" [1], flies a quadrotor through unknown, unmapped, static clutter (dense forest, snow-covered terrain, a collapsed building, a derailed train) with onboard sensing and compute:

- **Sensing:** a depth image from stereo matching, plus inertial measurements.
- **Policy:** a CNN maps them directly to a set of short-horizon, dynamically feasible polynomial trajectories, each with a predicted cost, in a receding-horizon loop. There is no mapping and no separate planning stage.
- **Training:** in simulation only, by privileged learning. It imitates a sampling-based expert (Metropolis-Hastings) that sees the full 3D point cloud and the true state. Simulated sensor noise gives zero-shot transfer to the real world.
- **Results:** the policy runs onboard on a Jetson TX2 [1].
  - **Real world:** 56 runs in natural and human-made environments never seen in training. The paper reports these success rates by speed in a figure only [1].
  - **Simulation:** in a dense simulated forest, 100 % success up to 5 m/s and 60 % at 10 m/s. Across all the simulated environments, the average is 70 % at 10 m/s, where no baseline completes every run even at 5 m/s [1].

The question is where to develop these techniques: here, or elsewhere.

Three facts decide it:

1. **The problem is not this repo's DAA.** Our DAA ([ADR 0005](0005-layered-daa.md)) detects and tracks *other drones* with one camera on a known course. [1] avoids *static* obstacles in an *unknown* environment, from stereo depth, while flying toward a goal.
2. **It is a different mission.** The task is reaching a goal through terrain the vehicle has never seen, not a lap of a known gate course. Its score is success rate against speed, not lap time. Its sensors include stereo, which the A2RL rules exclude (requirements §2, SYS-03).
3. **The tiers already say where the pieces go.** Algorithms live in the platform repos: perception and planning in cortex, learned-model execution in synapse, control and safety in reflex. Each mission repo owns one vehicle and mission and composes those components. The README says it directly: "algorithms do not live here".

There is a second question underneath. The composition framework is mission-neutral in intent, but it lives in this repo as `branes::racing`: the slot concepts, data contracts, `Stack`, harnesses, log and replay, `TimeBase`, and the SITL core. A second mission repo would want all of it.

## Decision

1. **The algorithms go in the platform repos:**

   | Piece | Repo |
   |---|---|
   | Stereo depth, and the learned receding-horizon planner (a `Planner` implementation) | cortex |
   | Running the policy network as a static, allocation-free graph | synapse |
   | A safety filter under the learned planner (the CBF of [ADR 0005](0005-layered-daa.md), applied to obstacles) | reflex |

2. **The mission goes in its own mission repo.** This covers outdoor flight through unknown clutter: the privileged expert and its training pipeline, rendered-depth simulation with stereo noise, randomized outdoor scenes, scenarios, and success-rate-against-speed scoring. The mission belongs in an existing mission repo whose profile fits (for example `isr-drone`), or in a new one. That is decided when the work starts.

3. **Racing-drone takes the planner only as a comparison baseline, later.** Once the planner exists in cortex, a baseline may put it behind the `Planner` slot and compare it with the waypoint and MPCC planners on an obstacle course. A2RL Season 2 announced "dynamic obstacles and mission-style tasks" ([history](../assessments/autonomous-drone-racing.md) §3.3). Such a baseline adds a depth sensor through the existing sensor interfaces (requirements §2, DAA-09). It is therefore not an A2RL-legal baseline, and it is reported as such.

4. **The framework is lifted out before a second mission repo is built on it, not copied.**
   - **Until then:** keep mission-neutral code free of racing-only types.
   - **Racing-only:** gates, courses, the oracle gate projector, the scorer.
   - **Mission-neutral:** slots, contracts, `Stack`, harness, log, time base, the simulator core.
   - Where the framework goes (a new repo, or a platform repo) is decided in its own ADR when the second mission repo is created.

## Consequences

- **Racing-drone's scope stays the race.** It doesn't grow outdoor rendering, a privileged expert, or a training pipeline.
- **Each platform piece is reusable.** The planner, the network and the filter serve any mission, including a later racing obstacle-course baseline, behind the same slot.
- **The cost is coordination.** The outdoor work spans three platform repos and one mission repo, and it needs the framework decision before the mission repo can use slots, harnesses and replay without copying them.
- **The rule in Decision 4 constrains racing-drone now**, at little cost: racing-specific types stay out of the mission-neutral headers. That boundary is what makes the later lift a move, not a rewrite.

**Current state:** proposed.

## Sources

1. Loquercio, Kaufmann, Ranftl, Müller, Koltun, Scaramuzza, "Learning high-speed flight in the wild," *Science Robotics* 6(59), abg5810 (2021), doi:10.1126/scirobotics.abg5810. Accepted version: <https://rpg.ifi.uzh.ch/docs/Loquercio21_Science.pdf> (accessed 2026-10-28). Code: <https://github.com/uzh-rpg/agile_autonomy>.
