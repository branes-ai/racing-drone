# Phase gate P0 (docs/plan/test-plan.md §6): the null baseline builds every
# harness, and replaying a log twice gives bit-identical output.
#
# Inputs: SITL, REPLAY, FLIGHT (executables), SOURCE_DIR, WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

function(run)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "${out}${err}")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "command failed (${rc}): ${ARGN}")
    endif()
endfunction()

# 1. SITL run.
run(${SITL} --scenario ${SOURCE_DIR}/scenarios/null-smoke.yaml --out ${WORK_DIR}/sitl.brlog)

# 2. Two replays, each verifying the outputs against the SITL recording.
run(${REPLAY} --log ${WORK_DIR}/sitl.brlog --out ${WORK_DIR}/replay1.brlog --verify)
run(${REPLAY} --log ${WORK_DIR}/sitl.brlog --out ${WORK_DIR}/replay2.brlog --verify)

# 3. The two replay logs are byte-identical.
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${WORK_DIR}/replay1.brlog ${WORK_DIR}/replay2.brlog
    RESULT_VARIABLE same)
if(NOT same EQUAL 0)
    message(FATAL_ERROR "replay is not deterministic: replay1.brlog and replay2.brlog differ")
endif()

# 3b. Replay rewrites the inputs and its own outputs in the same order under the
#     same header, so a replay of a SITL log by the same baseline is that log.
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${WORK_DIR}/sitl.brlog ${WORK_DIR}/replay1.brlog
    RESULT_VARIABLE same_sitl)
if(NOT same_sitl EQUAL 0)
    message(FATAL_ERROR "replay of the SITL log differs from the SITL log")
endif()

# 4. A replay of a replay reproduces it too (its outputs are a recording of this baseline).
run(${REPLAY} --log ${WORK_DIR}/replay1.brlog --out ${WORK_DIR}/replay3.brlog --verify)
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${WORK_DIR}/replay1.brlog ${WORK_DIR}/replay3.brlog
    RESULT_VARIABLE same3)
if(NOT same3 EQUAL 0)
    message(FATAL_ERROR "replaying a replay changed the log")
endif()

# 5. The flight stack starts with the same course and vehicle, flight and HIL forms.
run(${FLIGHT} --course ${SOURCE_DIR}/courses/a2rl-2025.yaml --vehicle ${SOURCE_DIR}/vehicles/rp1.yaml --cycles 10)
run(${FLIGHT} --course ${SOURCE_DIR}/courses/a2rl-2025.yaml --vehicle ${SOURCE_DIR}/vehicles/rp1.yaml
    --scenario ${SOURCE_DIR}/scenarios/null-smoke.yaml)

message(STATUS "gate P0: passed")
