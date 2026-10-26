cmake_minimum_required(VERSION 4.0)

# MSVC debug-info format is governed by CMAKE_MSVC_DEBUG_INFORMATION_FORMAT
# (CMP0141, NEW at our cmake_minimum_required) instead of a hardcoded /Zi.
# CMAKE_POLICY_DEFAULT_CMP0141 carries the NEW default into the FetchContent
# deps' own project() calls (Catch2, yaml-cpp), so the msvc-ninja /Z7 build
# doesn't hit C1041 (many cl.exe contending on one shared .pdb under Ninja).
set(CMAKE_POLICY_DEFAULT_CMP0141 NEW)

project(branes_racing LANGUAGES CXX)

# C++20 is required (concepts, std::span, std::bit_cast); do not lower it.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Organize targets into folders in IDEs (Visual Studio, Xcode).
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# MSVC opt-ins that gcc/clang give by default: the conformant preprocessor
# (Catch2 raw-string macro arguments) and UTF-8 sources.
if(MSVC)
    add_compile_options(/Zc:preprocessor /utf-8)
endif()

option(RACING_BUILD_TESTS "Build the tests/ regression suite" ${PROJECT_IS_TOP_LEVEL})

if(RACING_BUILD_TESTS)
    include(CTest)
endif()

# Third-party deps (yaml-cpp; Catch2 only when testing). Permissive licenses only.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/deps.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/warnings.cmake)

# The framework library: data contracts, slots, config loaders, log, harnesses.
add_subdirectory(sdk)

# The SITL harness library: reflex dynamics, sensor models, oracle gates,
# faults, and the scoreboard (docs/plan/sitl/).
add_subdirectory(sim)

# Baselines: one set of harness executables per baselines/<name>.yaml
# (docs/adr/0001-baseline-composition-framework.md).
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/slot_registry.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/baselines.cmake)
add_subdirectory(apps)

if(RACING_BUILD_TESTS AND BUILD_TESTING)
    add_subdirectory(tests)
endif()
