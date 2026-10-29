# cmake/deps.cmake — FetchContent declarations for third-party deps.
#
# Permissive licenses only (MIT / BSL / BSD / Apache-2.0); see cortex ADR 0001
# and 0002. Pinned versions are intentional; bumps go via dedicated PRs so the
# release-please changelog records dependency churn separately from feature work.
#
# The platform repos (cortex, reflex, synapse) and the Stillwater math
# libraries (MTL5, Universal) join here as racing-drone first consumes one of
# their components (docs/plan/implementation-plan.md). reflex joined first, for
# the SITL harness (docs/plan/sitl/). The three repos pin the same MTL5
# (5.20.0) and Universal (5.11.0), and each defines MTL5::mtl5 only if no one
# has yet, so whichever is fetched first provides it for all.

include(FetchContent)

# Don't re-run `git fetch` on every build. Not FORCEd, so a dependency bump in an
# existing build tree can re-enable fetching with -DFETCHCONTENT_UPDATES_DISCONNECTED=OFF.
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Skip FetchContent update steps")

# ── jbeder/yaml-cpp — vehicle / course / scenario / baseline files (MIT) ──
# Same pin as cortex. SYSTEM so its headers don't trip our warnings-as-errors
# targets.
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL       OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        yaml-cpp-0.9.0
    GIT_SHALLOW    TRUE
    SYSTEM
)
FetchContent_MakeAvailable(yaml-cpp)

# ── catchorg/Catch2 v3 — testing framework ──────────────────────────
if(RACING_BUILD_TESTS AND BUILD_TESTING)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.15.0
        GIT_SHALLOW    TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

# ── branes-ai/reflex — dynamics model, rate loop, mixer, filters (MIT) ──
# The SITL plant and the simulated flight controller (sim/). Pinned to a commit
# on reflex main; bumps go via dedicated PRs. Its tests and benches are off
# because it is not the top-level project. It brings MTL5 5.20.0 with it.
# SYSTEM, like yaml-cpp: reflex is built warning-clean under its own flags, but
# racing-drone's names can collide with its locals (MSVC C4459 sees a test's
# `using namespace branes::racing` make racing's `ok` hide reflex locals).
FetchContent_Declare(
    reflex
    GIT_REPOSITORY https://github.com/branes-ai/reflex.git
    GIT_TAG        33ad1bbea37cfe05f22286c2525d7a751da2b482  # main after reflex#33
    SYSTEM
)
FetchContent_MakeAvailable(reflex)

# ── stillwater-sc/universal — number systems for the precision sweep (MIT) ──
# The posits and cfloats that precision_sweep_<baseline> runs a stack in
# (docs/plan/precision-sweep/). Header-only; the SDK, slots, and glue never
# name a Universal type, so only the sweep tool and its tests link it. Pinned
# to v5.11.0, as in reflex and cortex, and defined only if no one has yet (the
# same targets as reflex's test-only fetch). SYSTEM, as for the other deps:
# its headers are not warning-clean under our flags (stillwater-sc/universal#1265).
if(NOT TARGET universal::universal)
    FetchContent_Declare(
        universal
        GIT_REPOSITORY https://github.com/stillwater-sc/universal.git
        GIT_TAG        v5.11.0
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  _header_only_no_build
    )
    FetchContent_MakeAvailable(universal)
    # Universal's headers include both <sw/universal/...> and <universal/...>.
    add_library(universal INTERFACE)
    add_library(universal::universal ALIAS universal)
    target_include_directories(universal SYSTEM INTERFACE
        ${universal_SOURCE_DIR}/include
        ${universal_SOURCE_DIR}/include/sw)
    target_compile_features(universal INTERFACE cxx_std_20)
endif()

# ── branes-ai/anatomy — the mission framework (MIT) ──────────────────
# The data contracts, slots and composition, configs, the log, the harness, the
# reflex glue and the closed-loop simulator (docs/adr/0008-anatomy-mission-framework.md,
# docs/plan/consume-anatomy/). Pinned to a release commit; bumps go via
# dedicated PRs, which regenerate the bridge headers (tools/gen_anatomy_bridges.py).
# Declared after racing-drone's own dependencies, so where both name one
# (yaml-cpp, reflex, Universal) the first declaration, racing-drone's, is used;
# the check below fails the configure if anatomy pins reflex differently. Its
# tests are off because it is not the top-level project. SYSTEM, as for reflex.
FetchContent_Declare(
    anatomy
    GIT_REPOSITORY https://github.com/branes-ai/anatomy.git
    GIT_TAG        67adfb1dfd715cb7e74981546d76906fe455539d  # v0.3.0
    SYSTEM
)
FetchContent_MakeAvailable(anatomy)

# anatomy and racing-drone must pin the same reflex (the first declaration
# wins, so a mismatch would build anatomy against racing-drone's reflex).
file(STRINGS ${anatomy_SOURCE_DIR}/cmake/deps.cmake anatomy_reflex_tag
     REGEX "GIT_TAG +[0-9a-f]+ +# main after reflex")
file(STRINGS ${CMAKE_CURRENT_LIST_FILE} racing_reflex_tag
     REGEX "GIT_TAG +[0-9a-f]+ +# main after reflex")
string(REGEX MATCH "[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]+" anatomy_reflex_tag "${anatomy_reflex_tag}")
string(REGEX MATCH "[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]+" racing_reflex_tag "${racing_reflex_tag}")
if(NOT anatomy_reflex_tag STREQUAL racing_reflex_tag)
    message(FATAL_ERROR "anatomy pins reflex ${anatomy_reflex_tag}, racing-drone ${racing_reflex_tag}: "
                        "bump them together (cmake/deps.cmake)")
endif()
