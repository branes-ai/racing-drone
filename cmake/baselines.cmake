# cmake/baselines.cmake — turn baselines/<name>.yaml into build targets.
#
# Racing-only (docs/assessments/framework-boundary.md); steps 1–4 are the
# mission-neutral composition_generate() (anatomy cmake/composition.cmake), over
# racing's registry (cmake/slot_registry.cmake).
#
# docs/adr/0001-baseline-composition-framework.md: a baseline file has a
# `composition:` block (slot -> implementation, resolved at compile time) and a
# `parameters:` block (loaded at startup). For each baseline this module:
#
#   1. parses the `composition:` block (two-space-indented `slot: Impl` lines),
#   2. checks that all seven slots are named exactly once with an implementation
#      registered for that slot (cmake/slot_registry.cmake),
#   3. hashes the canonical form "slot=Impl\n" (slots sorted) with SHA-256 —
#      the same canonical form branes::racing::config::composition_hash()
#      computes at startup,
#   4. generates <build>/generated/<name>/racing_baseline.hpp, which defines
#      branes::racing::baseline::{name, composition_hash, Composition<T>},
#   5. adds flight_stack_<name>, replay_<name>, sitl_<name>, and
#      sitl_batch_<name> (many seeds in-process, for the phase gates), and with
#      PRECISION_SWEEP precision_sweep_<name> (the stack in each scalar type of
#      docs/plan/precision-sweep/; links Universal).
#
#   racing_add_baseline(<name> [PRECISION_SWEEP] [DEBUG_SCENARIO <scenarios/file.yaml>])
#
# Visual Studio starts every harness in the source root, so relative paths such
# as scenarios/<file>.yaml resolve. DEBUG_SCENARIO (relative to the source root)
# also pre-fills the debugger arguments: sitl_<name> writes
# <build>/<name>.brlog, which replay_<name> then verifies.
include_guard(GLOBAL)
include(${ANATOMY_CMAKE_DIR}/composition.cmake)

function(racing_add_baseline name)
    cmake_parse_arguments(PARSE_ARGV 1 arg "PRECISION_SWEEP" "DEBUG_SCENARIO" "")
    if(arg_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "racing_add_baseline(${name}): unexpected arguments '${arg_UNPARSED_ARGUMENTS}'")
    endif()
    if(arg_KEYWORDS_MISSING_VALUES)
        message(FATAL_ERROR "racing_add_baseline(${name}): ${arg_KEYWORDS_MISSING_VALUES} needs a value")
    endif()
    set(file ${PROJECT_SOURCE_DIR}/baselines/${name}.yaml)
    if(NOT EXISTS ${file})
        message(FATAL_ERROR "racing_add_baseline(${name}): ${file} not found")
    endif()

    # ── 1–4. the composition: parsed, checked, hashed, and its header ───
    set(gen_dir ${CMAKE_BINARY_DIR}/generated/${name})
    composition_generate(racing ${name}
        FILE ${file}
        TEMPLATE ${PROJECT_SOURCE_DIR}/cmake/racing_baseline.hpp.in
        OUTPUT ${gen_dir}/racing_baseline.hpp)

    # ── 5. harness targets ──────────────────────────────────────────────
    set(harnesses flight_stack replay sitl sitl_batch)
    if(arg_PRECISION_SWEEP)
        list(APPEND harnesses precision_sweep)
    endif()
    foreach(harness IN LISTS harnesses)
        set(target ${harness}_${name})
        add_executable(${target} ${PROJECT_SOURCE_DIR}/apps/${harness}/main.cpp)
        target_include_directories(${target} PRIVATE ${gen_dir})
        target_link_libraries(${target} PRIVATE branes::racing branes::anatomy_glue)
        target_compile_definitions(${target} PRIVATE
            RACING_BASELINES_DIR="${PROJECT_SOURCE_DIR}/baselines")
        racing_set_warnings(${target})
        set_target_properties(${target} PROPERTIES
            FOLDER "Baselines/${name}"
            VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
    endforeach()
    if(arg_DEBUG_SCENARIO)
        if(NOT EXISTS ${PROJECT_SOURCE_DIR}/${arg_DEBUG_SCENARIO})
            message(FATAL_ERROR "racing_add_baseline(${name}): ${arg_DEBUG_SCENARIO} not found")
        endif()
        set(debug_log "\"${CMAKE_BINARY_DIR}/${name}.brlog\"")
        set_target_properties(sitl_${name} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS
            "--scenario ${arg_DEBUG_SCENARIO} --out ${debug_log}")
        set_target_properties(replay_${name} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS
            "--log ${debug_log} --out \"${CMAKE_BINARY_DIR}/${name}-replay.brlog\" --verify")
        set_target_properties(sitl_batch_${name} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS
            "--scenario ${arg_DEBUG_SCENARIO} --seeds 1 --jobs 1")
        set_target_properties(flight_stack_${name} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS
            "--course courses/a2rl-2025.yaml --vehicle vehicles/rp1.yaml --cycles 10")
        if(arg_PRECISION_SWEEP)
            set_target_properties(precision_sweep_${name} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS
                "--scenario ${arg_DEBUG_SCENARIO} --seeds 1 --jobs 1 --out-dir \"${CMAKE_BINARY_DIR}/${name}-sweep\"")
        endif()
    endif()
    # SITL closes the loop on the reflex dynamics (sim/); the batch runs seeds
    # on worker threads.
    target_link_libraries(sitl_${name} PRIVATE branes::racing_sim)
    target_link_libraries(sitl_batch_${name} PRIVATE branes::racing_sim Threads::Threads)
    if(arg_PRECISION_SWEEP)
        target_link_libraries(precision_sweep_${name} PRIVATE branes::racing_sim Threads::Threads universal::universal)
    endif()
endfunction()
