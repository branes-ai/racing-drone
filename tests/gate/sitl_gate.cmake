# SITL determinism gate (docs/plan/sitl/): with the reflex dynamics, the
# simulated FC, and every random stream in the loop, a run is reproducible.
#
#  1. Two SITL runs of the same scenario and seed write byte-identical logs.
#  2. Two replays each verify the recorded outputs and are byte-identical to
#     each other and to the SITL log.
#  3. score_log reads the log.
#
# Inputs: SITL, REPLAY, SCORE (executables), SOURCE_DIR, WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

function(run)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "${out}${err}")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "command failed (${rc}): ${ARGN}")
    endif()
endfunction()

function(same a b what)
    execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${a} ${b} RESULT_VARIABLE rc)
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "${what}: ${a} and ${b} differ")
    endif()
endfunction()

set(SCENARIO ${SOURCE_DIR}/scenarios/scripted-hop.yaml)
run(${SITL} --scenario ${SCENARIO} --out ${WORK_DIR}/sitl1.brlog)
run(${SITL} --scenario ${SCENARIO} --out ${WORK_DIR}/sitl2.brlog)
same(${WORK_DIR}/sitl1.brlog ${WORK_DIR}/sitl2.brlog "a rerun from the same seed is not identical")

run(${REPLAY} --log ${WORK_DIR}/sitl1.brlog --out ${WORK_DIR}/replay1.brlog --verify)
run(${REPLAY} --log ${WORK_DIR}/sitl1.brlog --out ${WORK_DIR}/replay2.brlog --verify)
same(${WORK_DIR}/replay1.brlog ${WORK_DIR}/replay2.brlog "replay is not deterministic")
same(${WORK_DIR}/sitl1.brlog ${WORK_DIR}/replay1.brlog "replay of the SITL log differs from the SITL log")

run(${SCORE} --log ${WORK_DIR}/sitl1.brlog)

message(STATUS "gate sitl: passed")
