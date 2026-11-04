# A run of a harness's command line (tests/CMakeLists.txt, tools.print_graph.*):
# COMMAND exits with EXPECT_EXIT, and what it writes, stdout then stderr,
# matches EXPECT_OUTPUT. A regex's `.` matches a line break too.
#
#   cmake -DCOMMAND=<exe;arg;...> -DEXPECT_EXIT=<n> -DEXPECT_OUTPUT=<regex> -P print_graph_test.cmake

foreach(var COMMAND EXPECT_EXIT EXPECT_OUTPUT)
    if(NOT DEFINED ${var})
        message(FATAL_ERROR "print_graph_test: -D${var} is required")
    endif()
endforeach()

execute_process(COMMAND ${COMMAND} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc STREQUAL "${EXPECT_EXIT}")
    message(FATAL_ERROR "print_graph_test: exit ${rc}, not ${EXPECT_EXIT}\nstdout:\n${out}\nstderr:\n${err}")
endif()
if(NOT "${out}${err}" MATCHES "${EXPECT_OUTPUT}")
    message(FATAL_ERROR "print_graph_test: the output does not match '${EXPECT_OUTPUT}'\nstdout:\n${out}\nstderr:\n${err}")
endif()
