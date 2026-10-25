# cmake/deps.cmake — FetchContent declarations for third-party deps.
#
# Permissive licenses only (MIT / BSL / BSD / Apache-2.0); see cortex ADR 0001
# and 0002. Pinned versions are intentional; bumps go via dedicated PRs so the
# release-please changelog records dependency churn separately from feature work.
#
# The platform repos (cortex, reflex, synapse) and the Stillwater math
# libraries (MTL5, Universal) join here in Phase 1, when racing-drone first
# consumes one of their components (docs/plan/implementation-plan.md). Their
# MTL5 / Universal pins must be aligned before that.

include(FetchContent)

# Don't re-run `git fetch` on every build.
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)

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
