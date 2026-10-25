# cmake/baselines.cmake — turn baselines/<name>.yaml into build targets.
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
#   5. adds flight_stack_<name>, replay_<name>, and sitl_<name>.
include_guard(GLOBAL)

function(racing_add_baseline name)
    set(file ${PROJECT_SOURCE_DIR}/baselines/${name}.yaml)
    if(NOT EXISTS ${file})
        message(FATAL_ERROR "racing_add_baseline(${name}): ${file} not found")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${file})

    # ── 1. parse the composition block ──────────────────────────────────
    file(STRINGS ${file} lines)
    set(in_block FALSE)
    foreach(line IN LISTS lines)
        if(line MATCHES "^composition:[ ]*$")
            set(in_block TRUE)
            continue()
        endif()
        if(in_block)
            if(line MATCHES "^[ ]*(#.*)?$")
                continue()  # blank or comment line inside the block
            elseif(line MATCHES "^  ([a-z_]+):[ ]*([A-Za-z0-9_]+)[ ]*(#.*)?$")
                set(slot ${CMAKE_MATCH_1})
                set(impl ${CMAKE_MATCH_2})
                if(DEFINED _impl_${slot})
                    message(FATAL_ERROR "${file}: slot '${slot}' named twice")
                endif()
                set(_impl_${slot} ${impl})
            elseif(line MATCHES "^[^ ]")
                set(in_block FALSE)  # next top-level key
            else()
                message(FATAL_ERROR "${file}: cannot parse composition line '${line}'")
            endif()
        endif()
    endforeach()

    # ── 2. validate against the registry ────────────────────────────────
    set(includes "")
    set(types "")
    foreach(slot IN LISTS RACING_SLOTS)
        if(NOT DEFINED _impl_${slot})
            message(FATAL_ERROR "${file}: composition is missing slot '${slot}'")
        endif()
        set(impl ${_impl_${slot}})
        get_property(impl_slot GLOBAL PROPERTY RACING_IMPL_${impl}_SLOT)
        if(NOT impl_slot)
            message(FATAL_ERROR "${file}: '${impl}' is not a registered implementation")
        endif()
        if(NOT impl_slot STREQUAL slot)
            message(FATAL_ERROR "${file}: '${impl}' fills slot '${impl_slot}', not '${slot}'")
        endif()
        get_property(header GLOBAL PROPERTY RACING_IMPL_${impl}_HEADER)
        get_property(type   GLOBAL PROPERTY RACING_IMPL_${impl}_TYPE)
        list(APPEND includes "#include <${header}>")
        list(APPEND types "${type}<T>")
    endforeach()
    list(REMOVE_DUPLICATES includes)

    # ── 3. canonical hash ───────────────────────────────────────────────
    set(sorted_slots ${RACING_SLOTS})
    list(SORT sorted_slots)
    set(canonical "")
    foreach(slot IN LISTS sorted_slots)
        string(APPEND canonical "${slot}=${_impl_${slot}}\n")
    endforeach()
    string(SHA256 hash "${canonical}")

    # ── 4. generated header ─────────────────────────────────────────────
    list(JOIN includes "\n" RACING_BASELINE_INCLUDES)
    list(JOIN types ",\n                                 " RACING_BASELINE_TYPES)
    set(RACING_BASELINE_NAME ${name})
    set(RACING_BASELINE_HASH ${hash})
    set(gen_dir ${CMAKE_BINARY_DIR}/generated/${name})
    configure_file(${PROJECT_SOURCE_DIR}/cmake/racing_baseline.hpp.in ${gen_dir}/racing_baseline.hpp @ONLY)

    # ── 5. harness targets ──────────────────────────────────────────────
    foreach(harness flight_stack replay sitl)
        set(target ${harness}_${name})
        add_executable(${target} ${PROJECT_SOURCE_DIR}/apps/${harness}/main.cpp)
        target_include_directories(${target} PRIVATE ${gen_dir})
        target_link_libraries(${target} PRIVATE branes::racing)
        target_compile_definitions(${target} PRIVATE
            RACING_BASELINES_DIR="${PROJECT_SOURCE_DIR}/baselines")
        racing_set_warnings(${target})
        set_target_properties(${target} PROPERTIES FOLDER "Baselines/${name}")
    endforeach()
endfunction()
