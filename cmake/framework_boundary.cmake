# The mission-neutral files of the framework (docs/plan/framework-split,
# docs/assessments/framework-boundary.md).
#
# Every file listed here moves to anatomy in step 2 of ADR 0008. A neutral file
# may include only other neutral files (and standard or third-party headers);
# the ctest boundary.neutral_includes (tests/tools/boundary_test.cmake) checks
# that. The list grows as the framework split proceeds (PRs 1a, 1b, 1c).
#
# Paths are relative to the source root.

set(RACING_NEUTRAL_FILES
    # Utilities and result
    sdk/include/branes/racing/result.hpp
    sdk/include/branes/racing/util/bytes.hpp
    sdk/include/branes/racing/util/sha256.hpp
    sdk/include/branes/racing/util/splitmix64.hpp
    sdk/src/util/sha256.cpp
    # Data contracts
    sdk/include/branes/racing/msg/codec.hpp
    sdk/include/branes/racing/msg/control.hpp
    sdk/include/branes/racing/msg/daa.hpp
    sdk/include/branes/racing/msg/estimate.hpp
    sdk/include/branes/racing/msg/fc.hpp
    sdk/include/branes/racing/msg/geometry.hpp
    sdk/include/branes/racing/msg/intruders.hpp
    sdk/include/branes/racing/msg/region.hpp
    sdk/include/branes/racing/msg/sensors.hpp
    sdk/include/branes/racing/msg/time.hpp
    sdk/include/branes/racing/msg/truth.hpp
    # The log
    sdk/include/branes/racing/log/log.hpp
    sdk/src/log/log.cpp
    # Configs: vehicle, scenario, baseline
    sdk/include/branes/racing/config/baseline.hpp
    sdk/include/branes/racing/config/error.hpp
    sdk/include/branes/racing/config/scenario.hpp
    sdk/include/branes/racing/config/vehicle.hpp
    sdk/src/config/baseline.cpp
    sdk/src/config/parse.hpp
    sdk/src/config/scenario.cpp
    sdk/src/config/vehicle.cpp
    # Slots: the neutral concepts, the slot context, nulls, and neutral slots
    sdk/include/branes/racing/slots/ground_truth_core.hpp
    sdk/include/branes/racing/slots/neutral.hpp
    sdk/include/branes/racing/slots/neutral_nulls.hpp
    sdk/include/branes/racing/slots/scripted.hpp
    sdk/include/branes/racing/slots/slot_context.hpp
    # Composition and harness: what a harness needs from a stack, and the runner
    sdk/include/branes/racing/compose/tick.hpp
    sdk/include/branes/racing/harness/args.hpp
    sdk/include/branes/racing/harness/files.hpp
    sdk/include/branes/racing/harness/runner.hpp
    sdk/include/branes/racing/harness/synthetic.hpp
    # Simulator models
    sim/include/branes/racing/sim/fc.hpp
    sim/include/branes/racing/sim/imu.hpp
    # Reflex glue
    baselines/glue/include/branes/racing/glue/cascaded_pid.hpp
    baselines/glue/include/branes/racing/glue/reflex_params.hpp
    # Tests of the neutral pieces, driven by a mission that is not racing
    tests/sim/neutral_mission.cpp
)
