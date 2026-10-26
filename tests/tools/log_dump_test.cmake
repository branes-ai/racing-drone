# log_dump on a B0 SITL log (apps/log_dump/main.cpp).
#
#  1. log_dump reads every record: its total equals the count SITL wrote.
#  2. tick.csv, state.csv, and command.csv each hold one row per tick (the
#     stack emits one state and one command per tick).
#  3. course.yaml and vehicle.yaml are the scenario's files, byte for byte.
#  4. In every CSV, the first and last rows have as many fields as the header.
#  5. A second dump is byte-identical to the first.
#  6. A missing log exits with code 1 and an error naming it.
#
# Inputs: SITL, DUMP (executables), SOURCE_DIR, WORK_DIR.

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

function(same a b what)
    execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${a} ${b} RESULT_VARIABLE rc)
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "${what}: ${a} and ${b} differ")
    endif()
endfunction()

run(sitl_out ${SITL} --scenario ${SOURCE_DIR}/scenarios/b0-determinism.yaml --out ${WORK_DIR}/run.brlog)
if(NOT sitl_out MATCHES ", ([0-9]+) ticks, ([0-9]+) records ->")
    message(FATAL_ERROR "cannot read the tick and record counts from: ${sitl_out}")
endif()
set(ticks ${CMAKE_MATCH_1})
set(records ${CMAKE_MATCH_2})

run(dump_out ${DUMP} --log ${WORK_DIR}/run.brlog --out-dir ${WORK_DIR}/a)
if(NOT dump_out MATCHES "log_dump: ([0-9]+) records ->")
    message(FATAL_ERROR "cannot read the record count from: ${dump_out}")
endif()
if(NOT CMAKE_MATCH_1 EQUAL records)
    message(FATAL_ERROR "log_dump read ${CMAKE_MATCH_1} records, SITL wrote ${records}")
endif()

foreach(name tick state command)
    file(STRINGS ${WORK_DIR}/a/${name}.csv lines)
    list(LENGTH lines n)
    math(EXPR rows "${n} - 1")
    if(NOT rows EQUAL ticks)
        message(FATAL_ERROR "${name}.csv has ${rows} rows, expected one per tick (${ticks})")
    endif()
endforeach()

same(${WORK_DIR}/a/course.yaml ${SOURCE_DIR}/courses/a2rl-2025.yaml "the embedded course")
same(${WORK_DIR}/a/vehicle.yaml ${SOURCE_DIR}/vehicles/rp1.yaml "the embedded vehicle")

function(fields line out_var)
    string(REGEX REPLACE "[^,]" "" commas "${line}")
    string(LENGTH "${commas}" n)
    set(${out_var} ${n} PARENT_SCOPE)
endfunction()
file(GLOB csvs ${WORK_DIR}/a/*.csv)
foreach(csv IN LISTS csvs)
    file(STRINGS ${csv} lines)
    list(LENGTH lines n)
    if(n LESS 2)
        continue()  # no rows (e.g. no gate in view)
    endif()
    list(GET lines 0 header)
    list(GET lines 1 first)
    list(GET lines -1 last)
    fields("${header}" nh)
    fields("${first}" nf)
    fields("${last}" nl)
    if(NOT nf EQUAL nh OR NOT nl EQUAL nh)
        message(FATAL_ERROR "${csv}: rows do not match the header's ${nh} separators (${nf}, ${nl})")
    endif()
endforeach()

run(dump2_out ${DUMP} --log ${WORK_DIR}/run.brlog --out-dir ${WORK_DIR}/b)
file(GLOB dumped RELATIVE ${WORK_DIR}/a ${WORK_DIR}/a/*)
foreach(f IN LISTS dumped)
    same(${WORK_DIR}/a/${f} ${WORK_DIR}/b/${f} "a second dump")
endforeach()

execute_process(COMMAND ${DUMP} --log ${WORK_DIR}/missing.brlog --out-dir ${WORK_DIR}/c
    RESULT_VARIABLE rc OUTPUT_QUIET ERROR_VARIABLE missing_err)
# A crash or failed launch leaves a non-numeric rc: require fail()'s exit code.
if(NOT "${rc}" STREQUAL "1")
    message(FATAL_ERROR "log_dump on a missing log: expected exit code 1, got '${rc}'")
endif()
if(NOT "${missing_err}" MATCHES "missing\\.brlog")
    message(FATAL_ERROR "log_dump's error does not name the missing log: ${missing_err}")
endif()

message(STATUS "log_dump: passed")
