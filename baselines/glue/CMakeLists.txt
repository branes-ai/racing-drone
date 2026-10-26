# baselines/glue/ — adapters from platform components to the slots.
#
# `branes::racing_glue` is header-only: the reflex parameter mapping for a
# vehicle file and the slot adapters around reflex components (CascadedPid,
# baseline B0, docs/plan/b0/). Every harness links it, so any baseline may
# name these implementations.

add_library(branes_racing_glue INTERFACE)
add_library(branes::racing_glue ALIAS branes_racing_glue)

target_include_directories(branes_racing_glue INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_racing_glue INTERFACE cxx_std_20)
target_link_libraries(branes_racing_glue INTERFACE branes::racing branes::reflex)
