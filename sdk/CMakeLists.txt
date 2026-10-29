# sdk/ — racing's part of the framework.
#
# `branes::racing` carries racing's data contracts (the course and the gate
# observations), its slots and stack, its configs and channels, and the bridge
# headers that bring anatomy's names into branes::racing (branes/racing/anatomy*.hpp).
# It compiles the course loader (yaml-cpp). The mission-neutral framework is
# branes::anatomy (docs/adr/0008-anatomy-mission-framework.md), which it links.

add_library(branes_racing STATIC
    src/config/course.cpp
)
add_library(branes::racing ALIAS branes_racing)

target_include_directories(branes_racing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_racing PUBLIC cxx_std_20)
# yaml-cpp stays an implementation detail of the loaders.
target_link_libraries(branes_racing PUBLIC branes::anatomy branes::anatomy_mobility PRIVATE yaml-cpp::yaml-cpp)
racing_set_warnings(branes_racing)
set_target_properties(branes_racing PROPERTIES FOLDER "SDK")
