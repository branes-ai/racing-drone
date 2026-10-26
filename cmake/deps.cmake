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
FetchContent_Declare(
    reflex
    GIT_REPOSITORY https://github.com/branes-ai/reflex.git
    GIT_TAG        33ad1bbea37cfe05f22286c2525d7a751da2b482  # main after reflex#33
)
FetchContent_MakeAvailable(reflex)
