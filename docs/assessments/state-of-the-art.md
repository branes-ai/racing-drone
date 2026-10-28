# State of the art: autonomous drone racing and detect-and-avoid

**Date:** 2026-10-23
**Purpose:** Identify the best known solutions for each component of the [mission requirements](../requirements/mission-requirements.md), and what that implies for our baselines.

Facts drawn from a paper or article are cited. Statements marked *(assessment)* are our own judgment.

For the history behind this survey (why racing, the IROS competitions, AlphaPilot and A2RL, and what each generation of systems solved), see [autonomous drone racing](autonomous-drone-racing.md).

## 1. Where the field is

| System | Year | Sensing | Estimation | Control | Result |
|---|---|---|---|---|---|
| AlphaPilot / Game of Drones teams | 2019 | Cameras + IMU | VIO + gate detection | Classical / MPC | First vision-based racing leagues. Far slower than humans |
| **Swift** (UZH + Intel) [1] | 2023 | Stereo VIO + camera | VIO fused with gate-corner detections (Kalman filter) | Deep-RL policy → collective thrust + body rates | Won 60 % of races against three human champions. Losses mostly from collisions |
| MPCC (UZH) [3, 4] | 2021–22 | Mocap in the main experiments | — | Model predictive contouring control | Up to 60 km/h and > 5 g. Faster than professional pilots on a mocap-measured course |
| **MonoRace** (TU Delft MAVLab) [2] | 2025–26 | **One rolling-shutter camera + IMU** | GateNet segmentation → line-based corners → multi-gate PnP → 16-state EKF with a drag model | G&CNet: a 3 × 64 MLP sending **motor commands directly**, at 500 Hz on the FC, trained with PPO | **Won A2RL 2025**, beating all AI teams and three world-champion human pilots. 28.2 m/s peak; fastest two-lap completion 16.56 s |
| SkyDreamer (TU Delft) [5] | 2025 | Mono camera | Implicit (end-to-end) | Model-based RL (Dreamer-style) | Claims champion-level, end-to-end, onboard |
| A2RL 2026 field [6] | 2026 | One mono RGB camera + IMU (rule) | — | — | Fastest lap 12.03 s (TII Racing). A human narrowly beat the AI in the human-vs-AI race |

**Takeaways (assessment):**

1. **Vision-only, onboard, champion-level racing is solved** at the research-prototype level (MonoRace, Swift). Reliability is not: MonoRace's winning policy completed 55 of 77 flights (71 %), and its most reliable policy completed 88 %.
2. **The winners used learned control, not MPC.** MPC (MPCC, NMPC) matches them in lap time when state estimation is perfect (mocap) [3]. With vision-only estimation, learned policies have so far been more robust to estimation error and model mismatch. Our MPC-first plan is defensible (interpretable, constraint-aware, and the basis for DAA constraints), but the framework must let us benchmark a learned policy against it (CTL-09).
3. **Collision with other drones is the open problem.** MonoRace hit another team's drone in the multi-drone race and placed third, because it had no collision avoidance [2]. Swift's losses were mostly collisions [1]. DAA is therefore a competitive advantage, not only a safety feature.

## 2. Component-level best known solutions

### 2.1 State estimation

| Approach | Used by | Strengths | Weaknesses |
|---|---|---|---|
| VIO (MSCKF / sliding window) + gate-PnP fusion | Swift [1] | Smooth between gates; drift is bounded by the gates | VIO struggles with motion blur at > 20 m/s and the rolling shutter |
| **Gate PnP + model-based EKF, no VIO** | MonoRace [2] | Simple, fast, monocular. Drag and thrust model in the prediction step | Needs gates in view. Dead-reckons between gates (crashed after > 25 m with no valid images) |
| End-to-end implicit state | SkyDreamer [5] | No hand-built pipeline | Hard to verify. Hard to feed DAA or MPC |

Techniques worth adopting from MonoRace [2]:

- **IMU-saturation handling.** When the filtered IMU acceleration and the model-predicted acceleration differ by more than σ = 22 m/s², switch to the model and inflate the covariance. With IMU-only filtering, 22 of 44 flights succeeded; with model correction, 3 of 3 did.
- **Delay compensation.** In SITL, with 17 ms image delay, RMS error drops from 0.289 m to 0.103 m.
- **Joint PnP over two gates.** Full PnP is used only at 2–5 m range with ≥ 6 corners. Otherwise, use translation-only PnP with the EKF attitude.
- **Offline refinement of the camera extrinsics** from flight logs against the known gate geometry.

Recommendation *(assessment)*: a **hybrid**. Cortex's MSCKF VIO bridges gaps between gates and camera dropouts. Gate PnP gives the course-frame fix. The drag-aware prediction model handles saturation. This is strictly more information than MonoRace uses, and it suits a pluggable estimator slot: a MonoRace-style "gate-PnP EKF only" estimator becomes one baseline, and "VIO + gate" another.

Datasets for replay testing: UZH-FPV drone racing dataset (aggressive FPV flights with ground truth), EuRoC (already in cortex CI), and our own logs.

### 2.2 Gate detection

- **Segmentation + geometric corners** (MonoRace GateNet). A U-Net at 384×384 trained on 3,500 synthetic and 500 real images, with Dice + 2·BCE loss. Corners come from LSD line intersections with RANSAC outlier rejection [2].
- **Direct corner / keypoint detection with part-affinity association** (Swift [1]; AlphaPilot-era detectors).

Recommendation: start with segmentation plus geometric corners. It is robust to partial gates, and the network is small enough for a KPU. This needs **conv2d, pooling, and upsampling ops in Synapse**, which do not exist today. Until they do, run the network through a vendor runtime on the companion, behind the same detector interface.

### 2.3 Trajectory planning

| Approach | Notes |
|---|---|
| Complementary progress constraints (CPC) | Truly time-optimal, but offline (minutes of compute). The reference lap-time bound |
| Sampling-based point-mass time-optimal paths + MPCC [4] | Online replanning with moving gates. Robust to 68 km/h wind |
| Perception-aware time-optimal planning [8] | Adds field-of-view and uncertainty constraints so gates stay visible. 9.8 m/s real-world |
| MPCC (no separate planner) [3] | Plans and tracks in one optimization by maximizing progress along the track |

Recommendation: offline CPC for the lap-time bound (SYS-10). Online, a point-mass sampler with perception-aware costs, feeding MPCC.

### 2.4 Control

| Approach | Notes |
|---|---|
| **MPCC** [3] | Maximizes progress and minimizes contour error. Near time-optimal in real time |
| MPCC++ [7] | Adds a track constraint and terminal set for safety guarantees. Removes the gate-in-cost trade-off |
| Tracking NMPC + INDI inner loop | Comparative studies (UZH) found NMPC with an INDI inner loop the most accurate at agile flight. INDI uses RPM and accelerometer feedback to reject model error |
| MPPI with neural signed distance fields (SDFs) [9] | Sampling-based MPC, easily handles arbitrary gate poses and non-convex obstacles. GPU-friendly |
| Learned G&CNet / RL policy [1, 2] | Fastest and most robust in competition. 500 Hz on the MCU [2] |

Recommendation: **MPCC on the companion, at 100–200 Hz.** It sends collective thrust plus body rates to an **INDI + rate loop on the FC**. Implement real-time iteration (RTI) SQP in reflex (Phase 3 of its roadmap). acados/HPIPM (BSD-2) can serve as a permissively licensed reference solver for validation. The DAA conflict constraints (§3) enter as time-varying constraints in the same MPC.

### 2.5 FPV and pilot interface

*(assessment, industry practice)*

- **Video.** Low-latency digital FPV systems (HDZero, DJI O-series, Walksnail) or analog 5.8 GHz. A dedicated FPV camera and VTX keep the pilot's latency independent of the navigation pipeline.
- **OSD.** MSP DisplayPort carries autonomy state onto the pilot's goggles.
- **RC.** ExpressLRS on the CRSF protocol (up to 1 kHz packet rates). The FC owns the RC input, so kill and takeover work even if the companion crashes.
- **Companion → FC command path.** Betaflight-style MSP override or a dedicated serial protocol sending thrust and body rates. The PX4 equivalent is offboard mode via uXRCE-DDS / MAVLink.

### 2.6 Simulation and frameworks

| Tool | Use for | Notes |
|---|---|---|
| Agilicious (UZH) | Reference architecture for an agile research quadrotor stack (estimator / planner / controller / bridge modules) | Check its license against cortex ADR 0002 before reading code. Use it as an architecture reference only |
| Flightmare | Fast RL training and photorealistic rendering | License check required |
| Aerial Gym (Isaac Gym) | GPU-parallel RL training | License check required |
| Isaac Sim / Pegasus, Gazebo | Photoreal rendering, HIL | Heavier |
| Own dynamics in reflex | Ground-truth rigid body, rotor, drag, motor, battery | Required anyway for MPC models and SITL determinism |

Recommendation *(assessment)*: the dynamics model is ours, in reflex, deterministic and fast. Rendering comes from an external engine through a narrow camera-image interface, so the renderer is swappable.

## 3. Detect and avoid

| Approach | Notes |
|---|---|
| Learned dynamic-obstacle risk (OmniRisk [10]) | Onboard avoidance at relative speeds up to 15 m/s |
| Vision-transformer end-to-end avoidance [11] | High-speed flight in dense clutter |
| Event-camera dodging | Low-latency detection of fast obstacles (UZH, 2020). Event cameras are banned under A2RL rules, allowed elsewhere |
| Obstacle-aware racing RL [12] | Racing through obstacles with a single end-to-end network. Notes that prior racing work assumed obstacle-free tracks |
| **Detect → track → constrain MPC → safety filter** | Classical layered DAA; the structure of ASTM F3442-style DAA for sUAS |

Recommendation *(assessment)*: a **layered** design.

1. **Detect** other drones with a small CNN on the navigation camera, in the same Synapse graph as GateNet (a second head on a shared backbone).
2. **Track** with an IMM-EKF using bearing and apparent size. Range comes from the known drone size; uncertainty is large at range.
3. **Avoid strategically** with time-varying separation constraints in MPCC. The drone keeps racing and picks a different line.
4. **Avoid tactically** with a control-barrier-function (CBF) safety filter in reflex. It is independent of the MPC solve and has a guaranteed bounded runtime.

The pixel-budget analysis in requirements §6.3 shows a drone ≈ 9 px wide at 10 m. Range-limited detection is the binding constraint, not avoidance dynamics.

## 4. Implications for the baselines

| Baseline | Composition | Purpose |
|---|---|---|
| **B0** `gt-pid` | Ground-truth state, waypoint reference, cascaded PID | Bring-up of SITL and the framework |
| **B1** `gt-mpcc` | Ground-truth state, time-optimal reference, MPCC + INDI | Control upper bound. Lap-time reference |
| **B2** `vio-gate-mpcc` | VIO + gate-PnP EKF, MPCC + INDI | **The primary racing baseline** (SYS-01), *conditional*: only if `VioGateEkf` passes the ADR 0004 promotion criterion |
| **B2m** `gate-ekf-mpcc` | MonoRace-style gate-PnP EKF (no VIO), MPCC | Ablation: what VIO buys us. Becomes the primary racing baseline if B2 fails the ADR 0004 comparison |
| **B3** `<est>-gcnet` | The estimator selected under ADR 0004 (`vio-gate` or `gate-ekf`), learned G&CNet controller | Head-to-head comparison against SOTA learned control. Like-for-like with the selected primary baseline |
| **B4** `<est>-mpcc-daa` | The selected primary baseline (B2 or B2m) + detect, track, MPC constraints, CBF filter | Multi-drone racing and DAA |

`<est>` is fixed once by the ADR 0004 decision at P3, for example `gate-ekf-mpcc-daa` if B2m is selected. B3 and B4 always use the selected estimator, so their comparisons with the primary baseline change one slot only.

## Sources

1. Kaufmann et al., "Champion-level drone racing using deep reinforcement learning," *Nature* 620, 982–987 (2023). <https://pmc.ncbi.nlm.nih.gov/articles/10468397>
2. Bahnam et al., "MonoRace: Winning Champion-Level Drone Racing with Robust Monocular AI," arXiv:2601.15222 (2026); *Science Robotics*, doi:10.1126/scirobotics.aee9250. <https://arxiv.org/abs/2601.15222>
3. Romero, Sun, Foehn, Scaramuzza, "Model Predictive Contouring Control for Time-Optimal Quadrotor Flight," arXiv:2108.13205. <https://arxiv.org/abs/2108.13205v3>
4. Romero, Penicka, Scaramuzza, "Time-Optimal Online Replanning for Agile Quadrotor Flight," arXiv:2203.09839. <https://arxiv.org/abs/2203.09839v2>
5. "SkyDreamer: Interpretable End-to-End Vision-Based Drone Racing with Model-Based Reinforcement Learning," arXiv:2510.14783. <https://arxiv.org/pdf/2510.14783>
6. A2RL 2026 coverage: <https://interestingengineering.com/ai-robotics/a2rl-ai-drones-autonomy>, <https://tbreak.com/a2rl-drone-championship-human-ai-abu-dhabi-2026/>; league overview <https://en.wikipedia.org/wiki/Abu_Dhabi_Autonomous_Racing_League>
7. Krinner et al., "MPCC++," RSS 2024. <https://roboticsproceedings.org/rss20/p109.html>
8. "Perception-aware time-optimal planning for quadrotor waypoint flight," arXiv:2603.04305. <https://arxiv.org/pdf/2603.04305.pdf>
9. "Vision-Guided MPPI for Agile Drone Racing: Navigating Arbitrary Gate Poses via Neural Signed Distance Fields," arXiv:2603.07199. <https://arxiv.org/pdf/2603.07199>
10. "OmniRisk: Omnidirectional Trajectory-Risk Learning for Agile Quadrotor Dynamic Avoidance," arXiv:2609.18191. <https://arxiv.org/pdf/2609.18191>
11. Bhattacharya et al., vision-transformer high-speed avoidance, arXiv:2405.10391. <https://arxiv.org/abs/2405.10391v1>
12. Vision-based curriculum RL for racing through obstacles, arXiv:2602.24030. <https://arxiv.org/abs/2602.24030>

Not re-verified for this document, cited from the established literature: Foehn et al., CPC time-optimal planning (*Science Robotics*, 2021); Foehn et al., Agilicious (*Science Robotics*, 2022); Sun et al., NMPC vs. differential-flatness control comparison (*IEEE T-RO*, 2022); Tal & Karaman, INDI for aggressive quadrotor flight (*IEEE TCST*, 2021); Falanga et al., event-camera dynamic obstacle avoidance (*Science Robotics*, 2020); Delmerico et al., UZH-FPV drone racing dataset (ICRA 2019); ASTM F3442/F3442M (DAA performance for sUAS).
