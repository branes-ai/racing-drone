# The edge trace on racing's graphs, and attribution (anatomy
# docs/plan/flow-testing, T5, P7 and P8).
#
#  1. A graph build's --trace is deterministic: two runs of the scenario write
#     equal traces (trace_compare exits 0), beside byte-identical logs.
#  2. Attribution (P8): b0-gt-pid and b0-shaped-pid differ in the controller's
#     body only, so their traces are equal up to the controller's first firing,
#     and first differ on an edge it writes: CascadedPid's command in
#     b0-gt-pid's, the shaper's reference in b0-shaped-pid's. The records
#     before it, the frames' and the tick's first three firings', are equal.
#  3. A body against a research graph (P7, on a mission's graph):
#     b0-shaped-pid's and b0-shaped-research's logs are equal but for the
#     baseline's name and the hash.
#  4. A slot build refuses --trace, and writes no log.
#
# Inputs: GT, SHAPED, RESEARCH (the three graph builds' SITL binaries), SLOT
# (a slot build's), TRACE_COMPARE, LOG_COMPARE; SCENARIO; WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

# run(<exit variable> <output variable> <command>...): run the command and set
# the variables to its exit code and its standard output and error.
function(run rc_var out_var)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "${out}${err}")
    set(${rc_var} "${rc}" PARENT_SCOPE)
    set(${out_var} "${out}${err}" PARENT_SCOPE)
endfunction()

# fly(<exe> <stem>): SITL on the scenario, writing <stem>.brlog and <stem>.brtrace.
function(fly exe stem)
    run(rc out ${exe} --scenario ${SCENARIO} --out ${WORK_DIR}/${stem}.brlog --trace ${WORK_DIR}/${stem}.brtrace)
    if(NOT rc EQUAL 0 OR NOT out MATCHES "edges -> ")
        message(FATAL_ERROR "${stem}: the traced run failed (${rc})")
    endif()
endfunction()

fly(${GT} gt)
fly(${GT} gt2)
fly(${SHAPED} shaped)
run(rc out ${RESEARCH} --scenario ${SCENARIO} --out ${WORK_DIR}/research.brlog)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "research: the run failed (${rc})")
endif()

# ── 1. determinism ──────────────────────────────────────────────────
run(rc out ${TRACE_COMPARE} --a ${WORK_DIR}/gt.brtrace --b ${WORK_DIR}/gt2.brtrace)
if(NOT rc EQUAL 0 OR NOT out MATCHES "are equal, [1-9][0-9]* records")
    message(FATAL_ERROR "a rerun's trace differs (${rc}): ${out}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${WORK_DIR}/gt.brlog ${WORK_DIR}/gt2.brlog
                RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "a traced rerun's log differs")
endif()

# ── 2. attribution (P8) ─────────────────────────────────────────────
run(rc out ${TRACE_COMPARE} --a ${WORK_DIR}/gt.brtrace --b ${WORK_DIR}/shaped.brtrace)
# The edge is the tick's fourth firing's: CascadedPid's step in one, which
# fills the controller alone, the shaper of the controller's body in the other.
set(want "a's is tick\\[3\\]\\.command \\(controller\\.step, in controller\\.step\\), "
         "b's tick\\[3\\]\\.shaped \\(controller\\.shaper\\.shape, in controller\\.step\\)")
string(CONCAT want ${want})
if(NOT rc EQUAL 1 OR NOT out MATCHES "differ: record [0-9]+: ${want}")
    message(FATAL_ERROR "b0-gt-pid's and b0-shaped-pid's traces don't first differ in the controller's body (${rc}): ${out}")
endif()

# ── 3. the body against the research graph (P7) ─────────────────────
run(rc out ${LOG_COMPARE} --a ${WORK_DIR}/shaped.brlog --b ${WORK_DIR}/research.brlog
    --except baseline_name,composition_hash)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "b0-shaped-pid's and b0-shaped-research's logs differ (${rc}): ${out}")
endif()

# ── 4. a slot build has no trace ────────────────────────────────────
run(rc out ${SLOT} --scenario ${SCENARIO} --out ${WORK_DIR}/slot.brlog --trace ${WORK_DIR}/slot.brtrace)
if(NOT rc EQUAL 1 OR NOT out MATCHES "a slot build has no graph: --trace is ")
    message(FATAL_ERROR "a slot build should refuse --trace (${rc}): ${out}")
endif()
if(EXISTS ${WORK_DIR}/slot.brlog OR EXISTS ${WORK_DIR}/slot.brtrace)
    message(FATAL_ERROR "a slot build's refused --trace wrote a file")
endif()

message(STATUS "gate trace: passed")
