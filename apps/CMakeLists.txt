# apps/ — the harness executables, generated per baseline (cmake/baselines.cmake).
#
# Each baseline in baselines/ yields flight_stack_<name>, replay_<name>,
# sitl_<name>, and sitl_batch_<name>, built from the sources in apps/<harness>/
# against that baseline's generated composition header. DEBUG_SCENARIO is the
# scenario the Visual Studio debugger runs by default. B0 also gets
# precision_sweep_b0-gt-pid (docs/plan/precision-sweep/).

racing_add_baseline(null DEBUG_SCENARIO scenarios/null-smoke.yaml)
racing_add_baseline(scripted-hop DEBUG_SCENARIO scenarios/scripted-hop.yaml)
racing_add_baseline(b0-gt-pid PRECISION_SWEEP DEBUG_SCENARIO scenarios/b0-determinism.yaml)

# score_log: the scoreboard for any baseline's SITL log (sim/score.hpp).
add_executable(score_log ${CMAKE_CURRENT_SOURCE_DIR}/score/main.cpp)
target_link_libraries(score_log PRIVATE branes::racing branes::racing_sim)
racing_set_warnings(score_log)
set_target_properties(score_log PROPERTIES
    FOLDER "Tools"
    VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    VS_DEBUGGER_COMMAND_ARGUMENTS "--log \"${CMAKE_BINARY_DIR}/b0-gt-pid.brlog\"")

# log_dump: a log's channels as CSV, for plotting (any baseline's log).
add_executable(log_dump ${CMAKE_CURRENT_SOURCE_DIR}/log_dump/main.cpp)
target_link_libraries(log_dump PRIVATE branes::racing)
racing_set_warnings(log_dump)
set_target_properties(log_dump PROPERTIES
    FOLDER "Tools"
    VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    VS_DEBUGGER_COMMAND_ARGUMENTS
        "--log \"${CMAKE_BINARY_DIR}/b0-gt-pid.brlog\" --out-dir \"${CMAKE_BINARY_DIR}/b0-gt-pid-dump\"")

# trace_export: a SITL log as a reflex-trace/1 replay, for the docs site's
# trace player (sim/replay_trace.hpp, tools/gen_replay.sh).
add_executable(trace_export ${CMAKE_CURRENT_SOURCE_DIR}/trace_export/main.cpp)
target_link_libraries(trace_export PRIVATE branes::racing branes::racing_sim)
racing_set_warnings(trace_export)
set_target_properties(trace_export PROPERTIES
    FOLDER "Tools"
    VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    VS_DEBUGGER_COMMAND_ARGUMENTS
        "--log \"${CMAKE_BINARY_DIR}/b0-gt-pid.brlog\" --scenario scenarios/b0-determinism.yaml --out \"${CMAKE_BINARY_DIR}/b0-gt-pid.json\"")
