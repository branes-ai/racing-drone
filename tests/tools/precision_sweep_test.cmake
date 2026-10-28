# precision_sweep_b0-gt-pid on one b0-determinism seed (docs/plan/precision-sweep/).
#
#  1. Every sweep type has one summary line, with status "constructed" or
#     "construction_failed" and a non-empty reason (acceptance criterion 4),
#     and one <type>.jsonl.
#  2. A rerun with a different --jobs and --out-dir gives byte-identical files
#     (criterion 3).
#  3. double.jsonl is sitl_batch_b0-gt-pid's --jsonl for the same seed (criterion 1).
#  4. gate_error.jsonl is consistent with the runs (below).
#
# Inputs: SWEEP, BATCH (executables), SOURCE_DIR, WORK_DIR.

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

set(scenario ${SOURCE_DIR}/scenarios/b0-determinism.yaml)
run(${SWEEP} --scenario ${scenario} --seeds 1 --jobs 1 --out-dir ${WORK_DIR}/a)
run(${SWEEP} --scenario ${scenario} --seeds 1 --jobs 2 --out-dir ${WORK_DIR}/b)

set(slugs double float cfloat32_8 posit32_2 bfloat16 binary16 posit16_1)
file(STRINGS ${WORK_DIR}/a/summary.jsonl lines)
list(LENGTH lines n)
list(LENGTH slugs types)
if(NOT n EQUAL types)
    message(FATAL_ERROR "summary.jsonl has ${n} lines, expected one per type (${types})")
endif()
foreach(slug IN LISTS slugs)
    list(FILTER lines EXCLUDE REGEX "^$")
    set(found "")
    foreach(line IN LISTS lines)
        if(line MATCHES "\"slug\":\"${slug}\"")
            set(found "${line}")
        endif()
    endforeach()
    if(NOT found)
        message(FATAL_ERROR "summary.jsonl has no line for ${slug}")
    endif()
    if(NOT found MATCHES "\"status\":\"constructed\"" AND NOT found MATCHES "\"status\":\"construction_failed\",\"reason\":\"[^\"]")
        message(FATAL_ERROR "${slug}: neither constructed nor construction_failed with a reason: ${found}")
    endif()
    if(NOT EXISTS ${WORK_DIR}/a/${slug}.jsonl)
        message(FATAL_ERROR "no ${slug}.jsonl")
    endif()
endforeach()

# 4. gate_error.jsonl (docs/plan/gate-error-by-gate/): 11 lines per built type;
#    each type's per-gate passages add up to its run's passages; double's
#    paired excess is exactly 0 wherever it has pairs.
file(STRINGS ${WORK_DIR}/a/gate_error.jsonl gate_lines)
foreach(slug IN LISTS slugs)
    file(READ ${WORK_DIR}/a/${slug}.jsonl run_line)
    if(run_line STREQUAL "")
        continue()  # the stack didn't build: no gate lines
    endif()
    string(JSON passages ERROR_VARIABLE no_score LENGTH "${run_line}" score gate_error_true_m)
    if(no_score)
        set(passages 0)  # a run error: no passages
    endif()
    set(lines_for 0)
    set(sum 0)
    foreach(line IN LISTS gate_lines)
        string(JSON type GET "${line}" type)
        if(NOT type STREQUAL slug)
            continue()
        endif()
        math(EXPR lines_for "${lines_for} + 1")
        string(JSON n GET "${line}" passages)
        math(EXPR sum "${sum} + ${n}")
        string(JSON pairs GET "${line}" pairs)
        if(slug STREQUAL "double" AND pairs GREATER 0)
            string(JSON excess GET "${line}" paired_excess_m p50)
            if(NOT excess EQUAL 0)
                message(FATAL_ERROR "double's paired excess is ${excess}, not 0: ${line}")
            endif()
        endif()
    endforeach()
    if(NOT lines_for EQUAL 11)
        message(FATAL_ERROR "gate_error.jsonl has ${lines_for} lines for ${slug}, expected 11")
    endif()
    if(NOT sum EQUAL passages)
        message(FATAL_ERROR "${slug}: per-gate passages add up to ${sum}, the run has ${passages}")
    endif()
endforeach()

foreach(name summary.jsonl lap_delta.jsonl gate_error.jsonl summary.md)
    same(${WORK_DIR}/a/${name} ${WORK_DIR}/b/${name} "rerun")
endforeach()
foreach(slug IN LISTS slugs)
    same(${WORK_DIR}/a/${slug}.jsonl ${WORK_DIR}/b/${slug}.jsonl "rerun")
endforeach()

# The scenario is shorter than two laps, so no run succeeds: --min-success 0.
run(${BATCH} --scenario ${scenario} --seeds 1 --min-success 0 --jsonl ${WORK_DIR}/batch.jsonl)
same(${WORK_DIR}/a/double.jsonl ${WORK_DIR}/batch.jsonl "double vs sitl_batch")
