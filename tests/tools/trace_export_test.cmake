# trace_export on a B0 SITL log (apps/trace_export/main.cpp, sim/replay_trace.hpp).
#
#  1. The replay of b0-determinism is reflex-trace/1, at 20 Hz, with the
#     run's scenario, seed, and baseline, the true channels, the reference as
#     position_setpoint, and takeoff and the first gate passage as events.
#  2. A second export is byte-identical to the first.
#  3. A scenario the log was not recorded from is refused (scenario_hash).
#  4. A rate whose period is not a multiple of the physics step is refused.
#  5. A missing log exits with code 1 and an error naming it.
#
# Inputs: SITL, EXPORT (executables), SOURCE_DIR, WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

function(run out_var)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "${out}${err}")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "command failed (${rc}): ${ARGN}")
    endif()
    set(${out_var} "${out}" PARENT_SCOPE)
endfunction()

# Runs a command that must fail with code 1 and an error matching `pattern`.
function(refused pattern)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT rc EQUAL 1)
        message(FATAL_ERROR "expected exit code 1, got ${rc}: ${ARGN}")
    endif()
    if(NOT err MATCHES "${pattern}")
        message(FATAL_ERROR "expected an error matching '${pattern}', got: ${err}")
    endif()
endfunction()

set(scenario ${SOURCE_DIR}/scenarios/b0-determinism.yaml)
run(sitl_out ${SITL} --scenario ${scenario} --out ${WORK_DIR}/run.brlog)

# 1. The replay.
run(out ${EXPORT} --log ${WORK_DIR}/run.brlog --scenario ${scenario} --out ${WORK_DIR}/a.json --commit abc1234)
if(NOT out MATCHES "trace_export: ([0-9]+) samples, ([0-9]+) events ->")
    message(FATAL_ERROR "cannot read the sample and event counts from: ${out}")
endif()
# b0-determinism runs 8 s; it does not complete the course, so the replay
# keeps the whole log: samples at 0, 0.05, ..., up to its last truth sample.
if(CMAKE_MATCH_1 LESS 150 OR CMAKE_MATCH_1 GREATER 161)
    message(FATAL_ERROR "expected about 8 s at 20 Hz, got ${CMAKE_MATCH_1} samples")
endif()
if(CMAKE_MATCH_2 LESS 2)
    message(FATAL_ERROR "expected takeoff and at least one gate passage, got ${CMAKE_MATCH_2} events")
endif()
file(READ ${WORK_DIR}/a.json json)
foreach(needle
        "\"schema\":\"reflex-trace/1\""
        "\"commit\":\"abc1234\""
        "\"block\":\"b0-gt-pid\""
        "\"seed\":1700"
        "\"dt_sim\":0.001"
        "\"dt_trace\":0.05"
        "\"world\":\"z-up\""
        "\"vehicle\":{\"arm_length\":0.11}"
        "\"position\":["
        "\"attitude\":["
        "\"rotor_thrusts\":["
        "\"position_setpoint\":["
        "\"label\":\"takeoff\""
        "\"label\":\"gate 1, lap 1\"")
    string(FIND "${json}" "${needle}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "the replay has no ${needle}")
    endif()
endforeach()

# 2. Deterministic.
run(out ${EXPORT} --log ${WORK_DIR}/run.brlog --scenario ${scenario} --out ${WORK_DIR}/b.json --commit abc1234)
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${WORK_DIR}/a.json ${WORK_DIR}/b.json RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "two exports of one log differ")
endif()

# 3. The wrong scenario.
refused("was not recorded from" ${EXPORT} --log ${WORK_DIR}/run.brlog
        --scenario ${SOURCE_DIR}/scenarios/b0-nominal.yaml --out ${WORK_DIR}/c.json)

# 4. A rate off the physics grid (1 kHz): a period of 1.5 ms.
refused("multiple of the physics step" ${EXPORT} --log ${WORK_DIR}/run.brlog --scenario ${scenario}
        --out ${WORK_DIR}/d.json --rate 666.6666666666666)

# 5. A missing log.
refused("missing.brlog" ${EXPORT} --log ${WORK_DIR}/missing.brlog --scenario ${scenario} --out ${WORK_DIR}/e.json)
