# sim/ — the SITL harness library (docs/plan/sitl/).
#
# `branes::racing_sim` is header-only: racing's simulator (the gate oracle on
# anatomy's BasicSimulator), the oracle gate projector, the scoreboard, and the
# racing batch run. It runs in double, like the logs. The closed-loop simulator
# itself is branes::anatomy_mobility_sim.

add_library(branes_racing_sim INTERFACE)
add_library(branes::racing_sim ALIAS branes_racing_sim)

target_include_directories(branes_racing_sim INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_racing_sim INTERFACE cxx_std_20)
target_link_libraries(branes_racing_sim INTERFACE branes::racing branes::anatomy_mobility_sim)
