# sim/ — the SITL harness library (docs/plan/sitl/).
#
# `branes::racing_sim` is header-only: the reflex plant and simulated flight
# controller, the IMU model, the oracle gate projector, the closed-loop event
# source (Simulator), and the scoreboard. It runs in double, like the logs.

add_library(branes_racing_sim INTERFACE)
add_library(branes::racing_sim ALIAS branes_racing_sim)

target_include_directories(branes_racing_sim INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_racing_sim INTERFACE cxx_std_20)
target_link_libraries(branes_racing_sim INTERFACE branes::racing branes::racing_glue branes::reflex)
