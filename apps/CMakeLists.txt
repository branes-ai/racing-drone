# apps/ — the harness executables, generated per baseline (cmake/baselines.cmake).
#
# Each baseline in baselines/ yields flight_stack_<name>, replay_<name>, and
# sitl_<name>, built from the sources in apps/<harness>/ against that
# baseline's generated composition header.

racing_add_baseline(null)
racing_add_baseline(scripted-hop)
racing_add_baseline(b0-gt-pid)

# score_log: the scoreboard for any baseline's SITL log (sim/score.hpp).
add_executable(score_log ${CMAKE_CURRENT_SOURCE_DIR}/score/main.cpp)
target_link_libraries(score_log PRIVATE branes::racing branes::racing_sim)
racing_set_warnings(score_log)
set_target_properties(score_log PROPERTIES FOLDER "Tools")
