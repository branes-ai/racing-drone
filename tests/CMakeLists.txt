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

# ── Scalar types (docs/plan/precision-sweep/) ──────────────────────────────
# The harness casts and the contracts in every type of the precision sweep;
# these tests link Universal, as the sweep tool does.
file(GLOB PRECISION_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/precision/*.cpp)
compile_all(FOLDER "Tests/precision" LIBS branes::racing_sim universal::universal SOURCES ${PRECISION_SRC})
foreach(source ${PRECISION_SRC})
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

# ── Baseline B0 and phase gate P1 (docs/plan/b0/) ──────────────────────────
# B0 is the first stateful stack: one short run must rerun and replay
# byte-identically, like the scripted-hop fixture.
add_test(NAME gate.b0.determinism
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_b0-gt-pid>
        -DREPLAY=$<TARGET_FILE:replay_b0-gt-pid>
        -DSCORE=$<TARGET_FILE:score_log>
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DSCENARIO=${PROJECT_SOURCE_DIR}/scenarios/b0-determinism.yaml
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/gate_b0
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/sitl_gate.cmake)

# log_dump writes every record of a B0 log as CSV (apps/log_dump/main.cpp).
# The fixture writes multi-sample references, which no current planner emits.
add_executable(reference_log_fixture ${CMAKE_CURRENT_SOURCE_DIR}/tools/reference_log_fixture.cpp)
target_link_libraries(reference_log_fixture PRIVATE branes::racing)
racing_set_warnings(reference_log_fixture)
set_target_properties(reference_log_fixture PROPERTIES FOLDER "Tests/tools")
add_test(NAME tools.log_dump
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_b0-gt-pid>
        -DDUMP=$<TARGET_FILE:log_dump>
        -DFIXTURE=$<TARGET_FILE:reference_log_fixture>
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/tools_log_dump
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tools/log_dump_test.cmake)

# The precision sweep on one seed (docs/plan/precision-sweep/): every type has
# a result, a rerun is byte-identical, and double matches sitl_batch.
add_test(NAME sweep.b0.smoke
    COMMAND ${CMAKE_COMMAND}
        -DSWEEP=$<TARGET_FILE:precision_sweep_b0-gt-pid>
        -DBATCH=$<TARGET_FILE:sitl_batch_b0-gt-pid>
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/sweep_b0
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tools/precision_sweep_test.cmake)

# P1 smoke, on every PR (test plan L2 PR tier): 10 nominal seeds, all succeed.
add_test(NAME gate.p1.smoke
    COMMAND sitl_batch_b0-gt-pid
        --scenario ${PROJECT_SOURCE_DIR}/scenarios/b0-nominal.yaml --seeds 10 --min-success 1)
set_tests_properties(gate.p1.smoke PROPERTIES LABELS "gate")

# P1 in full (test plan §6): 500 nominal seeds, >= 99 % succeed. Nightly only
# (.github/workflows/nightly.yml configures with RACING_NIGHTLY_TESTS=ON).
option(RACING_NIGHTLY_TESTS "Register the nightly gate tests (gate.p1.full)" OFF)
if(RACING_NIGHTLY_TESTS)
    add_test(NAME gate.p1.full
        COMMAND sitl_batch_b0-gt-pid
            --scenario ${PROJECT_SOURCE_DIR}/scenarios/b0-nominal.yaml --seeds 500 --min-success 0.99)
    set_tests_properties(gate.p1.full PROPERTIES LABELS "gate;nightly" TIMEOUT 3600)
endif()
# The compile-fail check of TimeBase and the framework's own tests moved to anatomy
# (docs/plan/consume-anatomy/).
