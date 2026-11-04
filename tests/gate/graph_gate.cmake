# A baseline's graph build is its slot build (docs/plan/flow-graph/,
# acceptance 2 and 3): on every scenario, and in the batch, the two builds write
# the same runs.
#
#  1. On each scenario, the SITL logs of the slot and graph builds have
#     byte-identical records, and headers equal in every field except
#     composition_hash (log_compare --except composition_hash), which is the
#     slot hash in one and the graph's in the other: compared whole, the logs
#     differ there.
#  2. On each batch scenario, the two batches' JSON lines are byte-identical,
#     and so are their reports.
#
# Inputs: SITL, SITL_GRAPH, BATCH, BATCH_GRAPH, LOG_COMPARE (executables);
# SCENARIOS (the scenario files), BATCH_SCENARIOS, SEEDS; WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

# run(<output variable> <command>...): run the command, which must succeed,
# and set the variable to its standard output.
function(run var)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "${out}${err}")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "command failed (${rc}): ${ARGN}")
    endif()
    set(${var} "${out}" PARENT_SCOPE)
endfunction()

foreach(scenario IN LISTS SCENARIOS)
    get_filename_component(stem ${scenario} NAME_WE)
    set(slots ${WORK_DIR}/${stem}.slots.brlog)
    set(graph ${WORK_DIR}/${stem}.graph.brlog)
    run(ignored ${SITL} --scenario ${scenario} --out ${slots})
    run(ignored ${SITL_GRAPH} --scenario ${scenario} --out ${graph})
    run(ignored ${LOG_COMPARE} --a ${slots} --b ${graph} --except composition_hash)

    execute_process(COMMAND ${LOG_COMPARE} --a ${slots} --b ${graph} RESULT_VARIABLE rc OUTPUT_VARIABLE out)
    if(NOT rc EQUAL 1 OR NOT out MATCHES "the header's composition_hash differs")
        message(FATAL_ERROR "${stem}: the graph build's log doesn't record the graph's hash (${rc}): ${out}")
    endif()
endforeach()

foreach(scenario IN LISTS BATCH_SCENARIOS)
    get_filename_component(stem ${scenario} NAME_WE)
    set(args --scenario ${scenario} --seeds ${SEEDS} --min-success 0)
    run(slots_report ${BATCH} ${args} --jsonl ${WORK_DIR}/${stem}.slots.jsonl)
    run(graph_report ${BATCH_GRAPH} ${args} --jsonl ${WORK_DIR}/${stem}.graph.jsonl)
    execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files
        ${WORK_DIR}/${stem}.slots.jsonl ${WORK_DIR}/${stem}.graph.jsonl RESULT_VARIABLE rc)
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "${stem}: the batches' runs differ")
    endif()
    if(NOT slots_report STREQUAL graph_report)
        message(FATAL_ERROR "${stem}: the batches' reports differ:\n${slots_report}\n${graph_report}")
    endif()
endforeach()

message(STATUS "gate graph: passed")
