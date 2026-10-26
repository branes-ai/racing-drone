# Regression tests. Each .cpp under a layer subdirectory becomes a Catch2 test
# executable automatically — see cmake/compile_all.cmake. To add a test, drop a
# `<name>.cpp` into the matching tests/<layer>/ subdirectory; no edit here is
# needed. File stems must be globally unique (they name the targets).

include(${PROJECT_SOURCE_DIR}/cmake/compile_all.cmake)

# ── SDK layer ──────────────────────────────────────────────────────────────
file(GLOB SDK_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sdk/*.cpp)
compile_all(FOLDER "Tests/sdk" LIBS branes::racing SOURCES ${SDK_SRC})
foreach(source ${SDK_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE RACING_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── SITL harness (sim/) ────────────────────────────────────────────────────
file(GLOB SIM_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp)
compile_all(FOLDER "Tests/sim" LIBS branes::racing_sim SOURCES ${SIM_SRC})
foreach(source ${SIM_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE RACING_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Baselines ──────────────────────────────────────────────────────────────
# Tests against a baseline's generated composition header (null for now).
file(GLOB BASELINE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/baselines/*.cpp)
compile_all(FOLDER "Tests/baselines" LIBS branes::racing SOURCES ${BASELINE_SRC})
foreach(source ${BASELINE_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_include_directories(${name} PRIVATE ${CMAKE_BINARY_DIR}/generated/null)
    target_compile_definitions(${name} PRIVATE RACING_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Phase gate P0 (test plan §6) ───────────────────────────────────────────
# sitl_null writes a log; replay_null reproduces its outputs (--verify), twice;
# the two replay logs must be byte-identical; flight_stack_null starts.
add_test(NAME gate.p0.replay_determinism
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_null>
        -DREPLAY=$<TARGET_FILE:replay_null>
        -DFLIGHT=$<TARGET_FILE:flight_stack_null>
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/gate_p0
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/p0_gate.cmake)

# ── SITL determinism gate (docs/plan/sitl/) ────────────────────────────────
# The scripted-hop fixture flies on the reflex dynamics with every random
# stream in use: a rerun and two replays must be byte-identical.
add_test(NAME gate.sitl.determinism
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_scripted-hop>
        -DREPLAY=$<TARGET_FILE:replay_scripted-hop>
        -DSCORE=$<TARGET_FILE:score_log>
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/gate_sitl
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/sitl_gate.cmake)
