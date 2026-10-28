# Autonomous drone racing: why, what, how, and what it taught

**Date:** 2026-10-28
**Purpose:** The context behind this repo. Why drone racing is used to push autonomy forward, what each generation of systems solved and how, the leagues that measured it (the IROS Autonomous Drone Racing competitions and the Abu Dhabi Autonomous Racing League, A2RL), and the insights that carry over to our baselines. The component-by-component survey used to choose the baselines is the [state of the art](state-of-the-art.md); this document is the history and the argument behind it.

Facts drawn from a paper or article are cited by number; web pages were accessed on 2026-10-28. Numbers are quoted as their source states them. Where sources disagree, both values are given with their sources. Statements marked *(assessment)* are our own judgment.

## 1. Why racing

A race is a small, closed problem that touches every part of an autonomy stack at once. The drone must fly "through a sequence of gates and avoid obstacles as quickly as possible using onboard sensors and limited computational power", at speeds above 80 km/h and accelerations above 4 g, which raises "significant challenges across perception, planning, control, and state estimation" [1]. It must "perceive, reason, plan, and act on the tens of milliseconds scale, all onboard a computationally limited platform" [1].

What makes it a good benchmark:

- **One score.** Drone racing "offers a single measure of the progress of the state-of-the-art in autonomous flying robotics: lap time" [1]. Success rate is the second number.
- **No hiding place.** "At the edge of a drone's performance envelope, all parts—perception, state-estimation, planning, and control—need to meticulously work together" [1]. A weak component shows up as lap time or a crash.
- **A human reference.** Expert pilots fly at "well over 100kph with only a first-person view camera", adapt to a new track "in a matter of minutes", and train for years [1]. Human-vs-AI races give an absolute bar.
- **Harder than the lab.** All sensing and computing is onboard, one gate is not enough, and gates may move or sit among other obstacles [8]. Competitions add unknown venues, gates placed off the published map, "lighting effects optimized for spectators rather than for drones, and large crowds" [15].
- **Transfer.** The same capabilities serve inspection, delivery and search and rescue, where "faster flight increases the operating radius achievable with a given battery" [1].

*(assessment)* For this repo, racing is the forcing function for the Branes.AI platform ([README](../../README.md)): every weakness in perception latency, estimation accuracy, control bandwidth or compute efficiency becomes a number on a scoreboard.

## 2. Timeline, by problem solved

| Years | Problem | What solved it | Evidence |
|---|---|---|---|
| 2016–2017 | Fly a gate course at all, without external positioning | Waypoints from the published course map, corrected by gate detection: stereo visual servoing (KAIST), monocular SLAM (INAOE), VIO (UZH) | IROS 2016 and 2017 winners passed 10 of 26 and 9 of 13 gates, at about 0.6 and 0.7 m/s [1][5][8] |
| 2017 | Fly fast against a human on a known course | Vision-based navigation built with Google's Tango (NASA JPL) | The human pilot won, 11.1 s against 13.9 s average lap [14] |
| 2018 | See gates robustly, and fly despite map error | A CNN predicting gate poses with uncertainty, fused into a gate map by an EKF, flown by MPC (UZH, "Beauty and the Beast") | Won IROS 2018: 8 gates in 31.8 s, against 61 s for second [10] |
| 2018–2020 | Learn perception without real labels | Domain-randomized simulation, zero-shot transfer to the real drone [23][24] | Moving-gate tracks flown from a CNN waypoint output [23] |
| 2019 | A full course, on a standard drone, in a public arena | Gate segmentation (GateNet), a model-based state estimator, risk-based control (MAVLab); VIO + multi-gate corner EKF (UZH) | MAVLab won AlphaPilot; no AI beat the human pilot [15][16][17] |
| 2021–2022 | Use the whole actuator envelope | Time-optimal planning with complementary progress constraints (CPC) [25]; model predictive contouring control (MPCC) [26] | Faster than expert human pilots in a motion-capture arena [25][26] |
| 2021–2023 | Robustness to model mismatch at the limit | Deep RL on a task-level objective [29][30]; residual models of perception and dynamics fitted from real flights (Swift) [31] | Swift won 15 of 25 races against three champions [31] |
| 2025 | Champion-level racing on one camera, on a fixed drone, in a league | Gate segmentation → multi-gate PnP → EKF with a drag model; an RL-trained network sending motor commands at 500 Hz (MonoRace) [33] | Won A2RL 2025, beating all AI teams and three champion human pilots [33] |
| 2025– | Racing other drones | Opponent-aware RL, multi-agent self-play [39][40] | Open: collisions decided both Swift's and MonoRace's losses [31][33] |

## 3. The leagues

### 3.1 IROS Autonomous Drone Racing, 2016–2019

The ADR competition "took place during the IROS conference between 2016 and 2019" [1]. It was organized by Hyungpil Moon (Sungkyunkwan University) and Hyunchul Shim (KAIST), joined by Si Jung Kim (UNLV) in 2017 [4][6]. The rule did not change: flight is fully autonomous, the drone that passes the most consecutive gates wins, and when gate counts tie or the track is completed, the fastest drone wins [8]. Teams received the course layout in advance, so the successful teams flew "waypoint tracking and visual gate recognition", and, by that 2019 review, no team "could complete the whole racing track" [2]. No ADR edition after 2019 was found.

| Edition | Field | Winner and result | Winning approach |
|---|---|---|---|
| **2016, Daejeon** (12 Oct [3]) | 11 teams competed [1] | KAIST: 10 of 26 gates in 1 min 26 s; a human flew all 26 in 1 min 31 s [1]. 2nd TU Delft MAVLab (also 10 gates, slower), 3rd ETH Zurich [3] | Stereo gate detection, direct visual servoing to the gate center [5] |
| **2017, Vancouver** | 14 registered, 5 flew in person [1] | INAOE (team QuetzalC++): 9 of 13 gates [1] in 194 s [9]. 2nd UZH [7] | Monocular SLAM with relative waypoints, gate detection correcting drift [8] |
| **2018, Madrid** (3 Oct [11]) | 8-gate track [12] | UZH: all 8 gates in 31.8 s; second 61 s, third (University of Maryland) 90.1 s [10][12] | CNN gate poses + EKF gate map + MPC, on VIO [10] |
| **2019, Macau** (5 Nov) | 6 teams; as many runs as possible in five minutes [13] | University of Tsukuba: 15 runs in 4:47. 2nd UMD: 14 runs in 4:42 [13] | Not published in a source we found |

The 2016 rules allowed any safe sensor and offered an onboard-compute division and a remote-compute division; GPS was expected to be unavailable [4]. The 2016 track had five test elements (a high-speed straight, sharp turns, a zig-zag, a spiral, and a dynamic obstacle) [4].

What the series showed: in three years the winning speed rose from about 0.6 m/s [1] to 2.0 m/s [1] (one participant reports 2.5 m/s [15]) as teams combined learned perception with optimal control [1]. The approaches that worked used the known map and corrected it with gate detections; *(assessment)* every later winner cited here kept that pattern.

### 3.2 Between the leagues: JPL, AlphaPilot, Game of Drones

- **NASA JPL vs. Ken Loo, Oct 2017.** A time trial between three JPL drones (navigation built with Google's Tango) and a world-class pilot. Loo averaged 11.1 s per lap, the drones 13.9 s. The drones flew the same line every lap; motion blur at high speed caused tracking loss [14].
- **Lockheed Martin AlphaPilot / AI Robotic Racing (AIRR), 2019**, run with the Drone Racing League.
  - 424 teams registered; nine finalists were selected by a computer-vision task, a simulated race in MIT's FlightGoggles, and a written approach [15][22].
  - Every team flew the same drone: about 0.7 m across, 3.4 kg, thrust-to-weight 1.4, an NVIDIA Jetson Xavier, two forward stereo pairs, an IMU and a laser rangefinder [16].
  - Gate positions were given in advance with up to about ±3 m of horizontal uncertainty, and the final course was about 74 m long [16].
  - MAVLab (TU Delft) won the $1 million prize, ahead of UZH and KAIST [17]. MAVLab gives its time as 12 s, at a 6.75 m/s average and 9.19 m/s maximum [15]. Other sources give 11 s [1] and 11.5 s [18]; UZH reports its own full-course time as 11.36 s [16].
  - The human pilot Gabriel "Gab707" Kocher beat MAVLab on the same drone, in 6 s [17] (6.9 s per [18]). The $250,000 bonus for beating a human was not won [19].
  - MAVLab used one of the four cameras, "as this setup matches the challenge that human pilots have to face" [15].
- **NeurIPS 2019 Game of Drones** (Microsoft Research). A simulation-only, head-to-head race in AirSim, in three tiers: planning only, perception only, and full autonomy against an opponent [20][21].

### 3.3 A2RL: the Abu Dhabi Autonomous Racing League

A2RL is organized by ASPIRE, part of Abu Dhabi's Advanced Technology Research Council (ATRC) [43][46]. It started with autonomous cars: the first race was held on 27 Apr 2024 at Yas Marina Circuit [43]. A2RL and the Drone Champions League (DCL) announced a drone partnership in Dec 2023 [44], and in Jul 2024 A2RL launched a drone championship with a US$1 million prize purse, qualified in simulation, with the finale in April 2025 [45].

**What makes A2RL different from the leagues before it** *(assessment)*: one standard drone for every team, the strictest sensor rule yet (one camera and an IMU), and four race formats that include human pilots and other drones. It measures not only whether autonomy can fly fast, but whether it can do so reliably, with minimal sensing, among other aircraft.

| | Season 1: A2RL x DCL, 2025 | Season 2, 2026 |
|---|---|---|
| Dates, venue | 11–12 Apr 2025, ADNEC Marina Hall, Abu Dhabi [33][46] | 21–22 Jan 2026, UMEX, Abu Dhabi [48] |
| Prize | US$1 million across four challenges [47] | US$600,000 [48] |
| Field | 14 teams in finals week [46] | Returning teams plus eight new ones, through virtual rounds and qualification in Nov 2025 [49] |
| Drone | Identical for all teams: 966 g, 5.1-inch props; STM32H743 flight controller; NVIDIA Jetson Orin NX; one rolling-shutter camera, 155° × 115°, 820 × 616 at 90 Hz [33] | The official A2RL drone; hardware modifications prohibited [51] |
| Sensors | "A single forward-facing camera and an inertial measurement unit" [47]; no external localization [33] | "A single forward-facing monocular RGB camera and an inertial measurement unit"; no LiDAR, stereo vision, GPS or external positioning [48] |
| Formats | AI Grand Challenge (two-lap time trial), AI vs Human, multi-drone race (four AI drones), AI Drag Race [33][47] | AI Speed Challenge, multi-drone Gold and Silver races, Human vs AI (best of nine) [48] |
| Results | MAVLab won the Grand Challenge (16.56 s two-lap time), AI vs Human (against three champion pilots) and the Drag Race; TII Racing won the multi-drone race [33][46] | TII Racing won the Speed Challenge, 12.032 s, ahead of MAVLab, 12.832 s; MAVLab won the Gold race, FlyBy the Silver; champion pilot Minchan Kim beat TII Racing's AI in the deciding ninth race, after the AI hit a gate [48] |

The 2025 track had 11 gates with a 1.5 m opening, including two double gates and a split-S, inside a 76 × 18 × 5.4 m volume of a 100 × 30 m hall [33]. MonoRace placed third in the multi-drone race after a collision, because it had no drone detection or avoidance [33].

Two cautions on the 2026 numbers. The organizer calls 12.032 s "a benchmark lap time" [48], while Team FlyBy reports its own laps of 8.0 s and 7.8 s [50][51]; the two evidently measure different things, and we quote the organizer's figure. And **no official A2RL drone rulebook is public**: the rules above come from organizer press material and team reports [33][47][48][51], and the A2RL site lists regulations for the car races only [52] (an open action in the [mission requirements](../requirements/mission-requirements.md), §7).

## 4. Key results and algorithms

| Problem | Result | Method | Venue |
|---|---|---|---|
| Agile flight on one camera and an IMU | 250 g quadrotor through narrow windows at 4.5 m/s [41] | Monocular VIO, onboard planning and control | Loianno et al., RA-L 2017 [41] |
| Keep the target in view while flying | The camera's point of interest kept in view inside the optimization [42] | Perception-aware MPC (PAMPC) | Falanga et al., IROS 2018 [42] |
| Gates that move, without a map | A CNN outputs a waypoint and speed; a planner and controller fly it [23] | Learned perception + classical control | Kaufmann et al., CoRL 2018 [23] |
| Training data | Zero-shot sim-to-real transfer of the racing CNN [24] | Domain randomization | Loquercio et al., T-RO 2020 [24] |
| Map uncertainty on a real course | Won IROS 2018 by a factor of two [10] | CNN gate poses with uncertainty → EKF gate map → MPC | Kaufmann et al., ICRA 2019 [10] |
| A standard drone in a public arena | Won AlphaPilot 2019 [15] | GateNet segmentation, model-based estimation, risk-based control | De Wagter et al., Field Robotics 2022 [15] |
| Same, with VIO | Second in AlphaPilot 2019; 100 % success at 5 m/s, 60 % at 8 m/s in tests [16] | VIO + multi-gate corner detections in an EKF, motion-primitive planning | Foehn et al., RSS 2020 / Auton. Robots 2022 [16] |
| Time-optimal trajectories through waypoints | Faster than expert human pilots (motion capture) [25] | Complementary progress constraints (CPC) | Foehn et al., Science Robotics 2021 [25] |
| Time-optimal flight online | Beat a world-class pilot's lap time at up to 60 km/h [26] | Model predictive contouring control (MPCC) | Romero et al., T-RO 2022 [26] |
| Optimal control at network speed | State-to-command networks imitating optimal control [27], then RL straight to motor commands [28] | Guidance-and-control networks (G&CNet) | Li et al., ICRA 2020 [27]; Ferede et al., ICRA 2024 [28] |
| RL vs. optimal control | 108 km/h peak, above 12 g [30] | Deep RL on a task-level objective | Song et al., IROS 2021 [29]; Science Robotics 2023 [30] |
| Champion-level racing | 15 of 25 races won against three champions, and the fastest race time [31] | Deep RL policy, VIO + gate detections, residual models from real flights | Kaufmann et al. (Swift), Nature 2023 [31] |
| No explicit state estimate | Flight from pixels to thrust and body rates, up to 40 km/h [32] | End-to-end RL | Geles et al., RSS 2024 [32] |
| Champion-level racing on one camera, in a league | Won A2RL 2025 [33] | Gate PnP + drag-model EKF; RL G&CNet to motor commands at 500 Hz | Bahnam et al. (MonoRace), Science Robotics 2026 [33] |
| Fast obstacles | 3.5 ms latency, obstacles at up to 10 m/s relative speed [36] | Event cameras | Falanga et al., Science Robotics 2020 [36] |
| Racing another drone | From a 2D game-theoretic planner [38] to multi-agent RL that beat a champion with 50 % fewer collisions [40] | Game theory; competitive and self-play RL [39][40] | Spica et al., T-RO 2020 [38]; arXiv 2025 [39]; preprint 2026 [40] |

The tools behind them: AirSim [35], FlightGoggles [22] and Flightmare [34] for simulation and RL at scale, and the UZH-FPV dataset and its VIO competitions for estimation at racing speed [37]. The survey by Hanover et al. covers the field to 2024 [1].

## 5. Insights

In the authors' words, and what they mean:

1. **Learning did not replace models; it joined them.** "No method with competitive performance in the real world is fully data-driven" [1]. Each winning system cited here from 2018 on used a learned component (gate detection, or the controller) inside a model-based structure (an EKF, a map, a dynamics model) [10][15][31][33]. MAVLab's AlphaPilot team judged that a monolithic network trained purely in simulation "likely requires too many training samples" [15].
2. **RL wins on the objective, not the optimizer.** "The fundamental advantage of RL over OC is not that it optimizes its objective better but that it optimizes a better objective" [30]. Optimal control's split into a planned trajectory and a tracking controller limits what the controller can express when the model is wrong; RL optimizes the task (progress through gates) directly, under randomized models [30].
3. **The sim-to-real gap is closed with data from the real track.** Swift fitted Gaussian-process residuals for perception and k-nearest-neighbour residuals for dynamics from real flights, then fine-tuned in the augmented simulator [31]. MonoRace relied on "extensive domain randomization" [33].
4. **State estimation is the binding constraint.** Classical VIO struggles with motion blur, low texture and high dynamic range at racing speed [1]. UZH's AlphaPilot crashes came from "the combination of VIO tracking failures and no visible gates" [16]. *(assessment)* Each of those winners anchored its estimate to the gates.
5. **Low-level control authority matters.** MonoRace's network sends motor commands directly, at 500 Hz "with full throttle authority" [33], removing the inner loops between the policy and the actuators.
6. **Reliability, not speed, is now the gap.** MonoRace's winning policy completed 55 of 77 flights [33] ([state of the art](state-of-the-art.md#1-where-the-field-is)). Humans crash and carry on; Swift "was not trained to recover after a crash" [31]. The 2026 Human vs AI race was decided when the AI hit a gate and could not recover [48].
7. **Other drones are the open problem.** Of Swift's 10 losses, 40 % were collisions with the opponent and 40 % with a gate; Swift was "unaware of its opponent" [31]. MonoRace lost the 2025 multi-drone race to a collision [33]. Multi-agent RL is the current answer in research [39][40].

What is still unsolved [1] *(and assessment)*: reliable high-speed estimation from one camera; recovery after contact; racing among other drones; and transfer to venues the team has not mapped.

## 6. What this means for this repo

*(assessment)* The baselines and decisions follow from the history:

| Insight | Where it lands |
|---|---|
| Lap time and success rate are the score | Every baseline is scored the same way, over seeds ([test plan](../plan/test-plan.md) §6); reliability counts as much as speed (insight 6) |
| A2RL's sensor rule is the bar | One mono camera and an IMU is the minimum sensor set for every baseline ([mission requirements](../requirements/mission-requirements.md) §2, SYS-03) |
| Models plus learning, anchored to gates (insights 1, 4) | Hybrid estimation with VIO, gate PnP and a drag model ([ADR 0004](../adr/0004-gate-relative-hybrid-estimation.md)); B2m tests MonoRace's estimator without VIO |
| RL vs. optimal control is still open off motion capture (insight 2) | MPCC is the primary controller, with a learned-policy slot to compare like for like ([ADR 0003](../adr/0003-mpc-primary-learned-policy-slot.md), B3) |
| Control authority (insight 5) | MPC on the companion, INDI and the rate loop on the flight controller ([ADR 0002](../adr/0002-companion-mpc-fc-rate-loop.md)) |
| Other drones decide races (insight 7) | Layered detect-and-avoid: MPC constraints plus an independent safety filter ([ADR 0005](../adr/0005-layered-daa.md), B4) |
| Small onboard compute | The [mixed-precision](mixed-precision.md) study of what precision each component needs |

## Sources

1. Hanover et al., "Autonomous Drone Racing: A Survey," *IEEE T-RO* 40, 3044–3067 (2024), doi:10.1109/TRO.2024.3400838. <https://arxiv.org/abs/2301.01755>
2. Moon et al., "Challenges and implemented technologies used in autonomous drone racing," *Intelligent Service Robotics* 12, 137–148 (2019), doi:10.1007/s11370-018-00271-6 (abstract). <https://doi.org/10.1007/s11370-018-00271-6>
3. TU Delft MAVLab, "TU Delft becomes second in first autonomous drone race" (2016). <https://mavlab.tudelft.nl/tu-delft-first-autonomous-drone-race>
4. IROS 2016 Autonomous Drone Racing call for participation (2016). <https://gsyc.urjc.es/pipermail/jde-developers/2016-April/004152.html>
5. Jung, Cho, Lee, Lee, Shim, "A direct visual servoing-based framework for the 2016 IROS Autonomous Drone Racing Challenge," *J. Field Robotics* 35(1), 146–166 (2018), doi:10.1002/rob.21743.
6. IROS 2017 Autonomous Drone Racing. <https://ewh.ieee.org/conf/iros/2017/iros2017.org/competitions/iros-2017-autonomous-drone-racing-in-mos-eisley-vancouver-arena.html>
7. UZH RPG, "RPG ranked 2nd in the IROS 2017 Autonomous Drone Race" (2017). <https://www.ifi.uzh.ch/en/rpg/news/RPG-ranked-2nd-in-the-IROS-2017-Autonomous-Drone-Race.html>
8. Li, van der Horst, Duernay, De Wagter, de Croon, "Visual model-predictive localization for computationally efficient autonomous racing of a 72-g drone," *J. Field Robotics* 37, 667–692 (2020), doi:10.1002/rob.21956. <https://arxiv.org/abs/1905.10110>
9. Li, Ozo, De Wagter, de Croon, "Autonomous drone race: A computationally efficient vision-based navigation and control strategy," *Robotics and Autonomous Systems* (2020), doi:10.1016/j.robot.2020.103621. <https://arxiv.org/abs/1809.05958>
10. Kaufmann et al., "Beauty and the Beast: Optimal Methods Meet Learning for Drone Racing," ICRA 2019. <https://arxiv.org/abs/1810.06224>
11. UZH RPG, "RPG won the IROS 2018 Autonomous Drone Race" (2018). <https://www.ifi.uzh.ch/en/rpg/news/RPG-won-the-IROS-2018-Autonomous-Drone-Race.html>
12. University of Maryland, "UMD Team's Autonomous Drone Takes 3rd in International Race" (2018). <https://aero.umd.edu/news/story/umd-teams-autonomous-drone-takes-3rd-in-international-race>
13. University of Maryland, "Student autonomous drone racing team takes 2nd place at IROS" (2019). <https://aero.umd.edu/news/story/student-autonomous-drone-racing-team-takes-2nd-place-at-iros>
14. NASA JPL, "Drone Race: Human Versus Artificial Intelligence" (2017). <https://www.jpl.nasa.gov/news/drone-race-human-versus-artificial-intelligence/>
15. De Wagter, Paredes-Vallés, Sheth, de Croon, "The sensing, state-estimation, and control behind the winning entry to the 2019 Artificial Intelligence Robotic Racing Competition," *Field Robotics* 2, 1263–1290 (2022), doi:10.55417/fr.2022042. <https://arxiv.org/abs/2109.14985>
16. Foehn et al., "AlphaPilot: Autonomous Drone Racing," RSS 2020; *Autonomous Robots* 46, 307–320 (2022), doi:10.1007/s10514-021-10011-y. <https://arxiv.org/abs/2005.12813>
17. Lockheed Martin, "AlphaPilot AI Drone Innovation Challenge." <https://lockheedmartin.com/en-us/news/events/ai-innovation-challenge.html>
18. HeroX, AlphaPilot update (2019). <https://www.herox.com/alphapilot/update/3013>
19. Fortune (9 Dec 2019). <https://www.fortune.com/2019/12/09/ai-pro-drone-racing-prize-drl>
20. Microsoft Research, "Game of Drones – Competition at NeurIPS 2019." <https://www.microsoft.com/en-us/research/academic-program/game-of-drones-competition-at-neurips-2019/>
21. Madaan et al., "AirSim Drone Racing Lab," PMLR 123 (2020). <https://arxiv.org/abs/2003.05654>
22. Guerra et al., "FlightGoggles," IROS 2019. <https://arxiv.org/abs/1905.11377>
23. Kaufmann et al., "Deep Drone Racing: Learning Agile Flight in Dynamic Environments," CoRL 2018. <https://arxiv.org/abs/1806.08548>
24. Loquercio et al., "Deep Drone Racing: From Simulation to Reality with Domain Randomization," *IEEE T-RO* 36(1), 1–14 (2020), doi:10.1109/TRO.2019.2942989. <https://arxiv.org/abs/1905.09727>
25. Foehn, Romero, Scaramuzza, "Time-optimal planning for quadrotor waypoint flight," *Science Robotics* 6, eabh1221 (2021), doi:10.1126/scirobotics.abh1221. <https://arxiv.org/abs/2108.04537>
26. Romero, Sun, Foehn, Scaramuzza, "Model Predictive Contouring Control for Time-Optimal Quadrotor Flight," *IEEE T-RO* 38, 3340–3356 (2022), doi:10.1109/TRO.2022.3173711. <https://arxiv.org/abs/2108.13205>
27. Li, Öztürk, De Wagter, de Croon, Izzo, "Aggressive online control of a quadrotor via deep network representations of optimality principles," ICRA 2020. <https://arxiv.org/abs/1912.07067>
28. Ferede, De Wagter, Izzo, de Croon, "End-to-end reinforcement learning for time-optimal quadcopter flight," ICRA 2024. <https://arxiv.org/abs/2311.16948>
29. Song, Steinweg, Kaufmann, Scaramuzza, "Autonomous Drone Racing with Deep Reinforcement Learning," IROS 2021. <https://arxiv.org/abs/2103.08624>
30. Song, Romero, Müller, Koltun, Scaramuzza, "Reaching the limit in autonomous racing: Optimal control versus reinforcement learning," *Science Robotics* (2023), doi:10.1126/scirobotics.adg1462. <https://arxiv.org/abs/2310.10943>
31. Kaufmann et al., "Champion-level drone racing using deep reinforcement learning," *Nature* 620, 982–987 (2023), doi:10.1038/s41586-023-06419-4. <https://pmc.ncbi.nlm.nih.gov/articles/10468397>
32. Geles, Bauersfeld, Romero, Xing, Scaramuzza, "Demonstrating Agile Flight from Pixels without State Estimation," RSS 2024. <https://arxiv.org/abs/2406.12505>
33. Bahnam et al., "MonoRace: Winning Champion-Level Drone Racing with Robust Monocular AI," arXiv:2601.15222 (2026); *Science Robotics*, doi:10.1126/scirobotics.aee9250. <https://arxiv.org/abs/2601.15222>
34. Song et al., "Flightmare: A Flexible Quadrotor Simulator," CoRL 2020. <https://arxiv.org/abs/2009.00563>
35. Shah, Dey, Lovett, Kapoor, "AirSim," Field and Service Robotics 2017. <https://arxiv.org/abs/1705.05065>
36. Falanga, Kleber, Scaramuzza, "Dynamic obstacle avoidance for quadrotors with event cameras," *Science Robotics* 5, eaaz9712 (2020), doi:10.1126/scirobotics.aaz9712.
37. UZH-FPV drone racing dataset, previous competitions. <https://fpv.ifi.uzh.ch/previous-competitions/>
38. Spica, Cristofalo, Wang, Montijano, Schwager, "A real-time game theoretic planner for autonomous two-player drone racing," *IEEE T-RO* 36, 1389–1403 (2020), doi:10.1109/TRO.2020.2994881.
39. Pasumarti, Bianchi, Loquercio, "Agile Flight Emerges from Multi-Agent Competitive Racing," arXiv:2512.11781 (2025). <https://arxiv.org/abs/2512.11781>
40. Geles, Bauersfeld, Wulfmeier, Scaramuzza, "Superhuman Safe and Agile Racing through Multi-Agent Reinforcement Learning," preprint (2026). <https://rpg.ifi.uzh.ch/docs/Arxiv26_Geles_MARL.pdf>
41. Loianno, Brunner, McGrath, Kumar, "Estimation, control and planning for aggressive flight with a small quadrotor with a single camera and IMU," *IEEE RA-L* 2(2), 404–411 (2017). <https://ieeexplore.ieee.org/document/7762111>
42. Falanga, Foehn, Lu, Scaramuzza, "PAMPC: Perception-aware model predictive control for quadrotors," IROS 2018. <https://arxiv.org/abs/1804.04811>
43. "Abu Dhabi Autonomous Racing League," Wikipedia. <https://en.wikipedia.org/wiki/Abu_Dhabi_Autonomous_Racing_League>
44. Drone Champions League, "DCL partner with A2RL" (2023). <https://dcl.aero/press/dcl-partner-with-a2rl/>
45. "A2RL Soars to New Heights: Launches Autonomous Drone Racing Championship Worth US $1Mn Prize Pool," Business Wire (25 Jul 2024). <https://www.silicon.co.uk/press-release/a2rl-soars-to-new-heights-launches-autonomous-drone-racing-championship-worth-us-1mn-prize-pool>
46. Abu Dhabi Media Office, "A2RL X DCL Autonomous Drone Championship advances global expertise in AI drone technology" (17 Apr 2025). <https://www.mediaoffice.abudhabi/en/technology/a2rl-x-dcl-autonomous-drone-championship-advances-global-expertise-in-ai-drone-technology/>
47. Drone Champions League, "Autonomous Drone Championship and DCL Falcon Cup Finals 2025" (2025). <https://dronechampionsleague.com/1m-global-prize-pool/>
48. A2RL, "A2RL Drone Championship Sets the Pace for AI in Autonomous Flight" (Jan 2026). <https://a2rl.io/news/45/A2RL-Drone-Championship-Sets-the-Pace-for-AI-in-Autonomous-Flight>
49. A2RL, "From Selection to Qualification: The Road to A2RL Drones Season 2." <https://a2rl.io/blog/17/From-Selection-to-Qualification-The-Road-to-A2RL-Drones-Season-2>
50. robo4you, "A2RL 2026 Season 2." <https://robo4you.at/posts/A2RL_2026_Season_2/>
51. TU Wien Informatics, Team FlyBy at A2RL 2026. <https://informatics.tuwien.ac.at/news/3102>
52. A2RL, autonomous drone race page and regulations. <https://a2rl.io/autonomous-drone-race>, <https://a2rl.io/>
