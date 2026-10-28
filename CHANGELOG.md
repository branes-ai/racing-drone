# Changelog

## [0.8.4](https://github.com/branes-ai/racing-drone/compare/v0.8.3...v0.8.4) (2026-10-28)


### Code Refactoring

* **apps:** move the harness programs' neutral logic into the framework ([#64](https://github.com/branes-ai/racing-drone/issues/64)) ([9b9a525](https://github.com/branes-ai/racing-drone/commit/9b9a525ef91796cd99de83f74828a05371b5a57e))
* **sim:** split the simulator, batch runner, sweep types and generator in place ([#62](https://github.com/branes-ai/racing-drone/issues/62)) ([09721c8](https://github.com/branes-ai/racing-drone/commit/09721c8af454b7de295d2a95df3679bf70f08081))

## [0.8.3](https://github.com/branes-ai/racing-drone/compare/v0.8.2...v0.8.3) (2026-10-28)


### Code Refactoring

* **sdk:** split the slots, slot context, stack and harness in place ([#60](https://github.com/branes-ai/racing-drone/issues/60)) ([b4241e0](https://github.com/branes-ai/racing-drone/commit/b4241e0c07c7a09992495ff41807acb7a85505dc))

## [0.8.2](https://github.com/branes-ai/racing-drone/compare/v0.8.1...v0.8.2) (2026-10-28)


### Documentation

* **adr:** 0008, the mission framework moves to anatomy ([#55](https://github.com/branes-ai/racing-drone/issues/55)) ([f85ff73](https://github.com/branes-ai/racing-drone/commit/f85ff73a7a3870942e02266a53f60a33e775e20b))

## [0.8.1](https://github.com/branes-ai/racing-drone/compare/v0.8.0...v0.8.1) (2026-10-28)


### Documentation

* **adr:** 0007, where outdoor obstacle avoidance lives ([#54](https://github.com/branes-ai/racing-drone/issues/54)) ([fe2e6bd](https://github.com/branes-ai/racing-drone/commit/fe2e6bd5221d5d0b79a14cc4fcfd6903536691d5))

## [0.8.0](https://github.com/branes-ai/racing-drone/compare/v0.7.0...v0.8.0) (2026-10-28)


### Features

* **docs-site:** a landing page and getting started, like cortex's and reflex's ([#53](https://github.com/branes-ai/racing-drone/issues/53)) ([54ce20d](https://github.com/branes-ai/racing-drone/commit/54ce20d50e41253494db4c115465487af96d2ce1)), closes [#52](https://github.com/branes-ai/racing-drone/issues/52)
* **docs-site:** publish ./docs as a github pages site ([#49](https://github.com/branes-ai/racing-drone/issues/49)) ([db44a16](https://github.com/branes-ai/racing-drone/commit/db44a16e723a23cc84704ef035f4c06d06ef995b))


### Documentation

* **assessments:** a history of autonomous drone racing, iros adr and a2rl ([#51](https://github.com/branes-ai/racing-drone/issues/51)) ([59ba30e](https://github.com/branes-ai/racing-drone/commit/59ba30eb4a9bebf6515890c52b95d2597ad912ad))
* **guides:** work through the stack in visual studio ([#46](https://github.com/branes-ai/racing-drone/issues/46)) ([81ab841](https://github.com/branes-ai/racing-drone/commit/81ab84118ec3f0c17fa9aef7e91877473bfcf21f))

## [0.7.0](https://github.com/branes-ai/racing-drone/compare/v0.6.2...v0.7.0) (2026-10-28)


### Features

* **sim:** gate error by gate in the precision sweep, and h4's distance verdict ([#43](https://github.com/branes-ai/racing-drone/issues/43)) ([b4b5f27](https://github.com/branes-ai/racing-drone/commit/b4b5f27d7f691813ee69584e91e759d7ce300ad4))

## [0.6.2](https://github.com/branes-ai/racing-drone/compare/v0.6.1...v0.6.2) (2026-10-28)


### Bug Fixes

* **baselines:** checked time base for the controller's dt ([#41](https://github.com/branes-ai/racing-drone/issues/41)) ([d9c200d](https://github.com/branes-ai/racing-drone/commit/d9c200dbbf7bb67a23f574cd5b5adc693b850c35))

## [0.6.1](https://github.com/branes-ai/racing-drone/compare/v0.6.0...v0.6.1) (2026-10-28)


### Documentation

* **plan:** mark the precision-sweep plan implemented ([#38](https://github.com/branes-ai/racing-drone/issues/38)) ([1229d55](https://github.com/branes-ai/racing-drone/commit/1229d55364dbd3ebaca3c06360f8d092668e025b))

## [0.6.0](https://github.com/branes-ai/racing-drone/compare/v0.5.2...v0.6.0) (2026-10-27)


### Features

* **sim:** precision sweep of baseline b0 across scalar types ([#33](https://github.com/branes-ai/racing-drone/issues/33)) ([fb06aec](https://github.com/branes-ai/racing-drone/commit/fb06aec1fac2c387431ca5a46f4c2abb53d364f8))

## [0.5.2](https://github.com/branes-ai/racing-drone/compare/v0.5.1...v0.5.2) (2026-10-27)


### Documentation

* **assessments:** mixed precision in the racing stack ([#31](https://github.com/branes-ai/racing-drone/issues/31)) ([6ef5682](https://github.com/branes-ai/racing-drone/commit/6ef56822217841000ef09770c40041bfe873b7dc))

## [0.5.1](https://github.com/branes-ai/racing-drone/compare/v0.5.0...v0.5.1) (2026-10-27)


### Documentation

* **plan:** mark the b0 and reference-log plans implemented ([#28](https://github.com/branes-ai/racing-drone/issues/28)) ([2a01ef0](https://github.com/branes-ai/racing-drone/commit/2a01ef0004332a46f814a9210a29ee54391505c6))

## [0.5.0](https://github.com/branes-ai/racing-drone/compare/v0.4.0...v0.5.0) (2026-10-27)


### Features

* **log:** record the planner reference as a log channel ([#26](https://github.com/branes-ai/racing-drone/issues/26)) ([343596a](https://github.com/branes-ai/racing-drone/commit/343596a71b1c16b929352e4195b390ab0cbc8d07))

## [0.4.0](https://github.com/branes-ai/racing-drone/compare/v0.3.0...v0.4.0) (2026-10-26)


### Features

* **tools:** log_dump writes a log's channels as csv ([#23](https://github.com/branes-ai/racing-drone/issues/23)) ([c76d5cc](https://github.com/branes-ai/racing-drone/commit/c76d5ccf85bb68d861632209a8d2f3db6ae02104))


### Build System

* **cmake:** visual studio debugger defaults for the harnesses ([#22](https://github.com/branes-ai/racing-drone/issues/22)) ([0059d11](https://github.com/branes-ai/racing-drone/commit/0059d1164496c4bc94639de2a4cae8596e767efe))

## [0.3.0](https://github.com/branes-ai/racing-drone/compare/v0.2.0...v0.3.0) (2026-10-26)


### Features

* **baselines:** b0 gt-pid baseline and gate p1 ([#20](https://github.com/branes-ai/racing-drone/issues/20)) ([876b9f1](https://github.com/branes-ai/racing-drone/commit/876b9f19deec08fef3c8802a9e7e3603cee6045b))

## [0.2.0](https://github.com/branes-ai/racing-drone/compare/v0.1.0...v0.2.0) (2026-10-26)


### Features

* **sim:** closed-loop sitl on the reflex dynamics ([#18](https://github.com/branes-ai/racing-drone/issues/18)) ([eeb38e9](https://github.com/branes-ai/racing-drone/commit/eeb38e985a4e10526b7fc7c24ba3224b853a39b1))

## 0.1.0 (2026-10-25)


### Features

* phase 0 framework skeleton with the null baseline ([#5](https://github.com/branes-ai/racing-drone/issues/5)) ([968efa9](https://github.com/branes-ai/racing-drone/commit/968efa9272f3e5a646a57e295984de21ac40102b))


### Documentation

* racing mission requirements, state of the art, architecture, and plans ([#4](https://github.com/branes-ai/racing-drone/issues/4)) ([adf27b5](https://github.com/branes-ai/racing-drone/commit/adf27b5aa609e8e3d85ad2bf5f57cf9811c93010))
* **readme:** four-tier stack diagram ([#2](https://github.com/branes-ai/racing-drone/issues/2)) ([0d478bb](https://github.com/branes-ai/racing-drone/commit/0d478bb2c8067487a8afa3b98d3fb45155aa403b))
* **readme:** qualify hardware targets and explain the stack diagram arrows ([#3](https://github.com/branes-ai/racing-drone/issues/3)) ([17ba72c](https://github.com/branes-ai/racing-drone/commit/17ba72c9319035d6ee6249954e9119612afc4bd8))
* record the racing-drone vision in the readme ([#1](https://github.com/branes-ai/racing-drone/issues/1)) ([cbcdd05](https://github.com/branes-ai/racing-drone/commit/cbcdd05b2c97e1d3d6c6499ac92a90b332a6bdc1))
