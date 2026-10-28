# boundary.neutral_includes (docs/plan/framework-split): every file listed in
# cmake/framework_boundary.cmake exists, and includes no framework header that
# is not itself listed. A neutral file may include standard and third-party
# headers, and local headers ("...") beside it that are listed.
#
# Inputs: SOURCE_DIR; MANIFEST (default: cmake/framework_boundary.cmake).
if(NOT MANIFEST)
    set(MANIFEST ${SOURCE_DIR}/cmake/framework_boundary.cmake)
endif()
include(${MANIFEST})

# The include roots of the framework's headers, relative to the source root.
set(roots sdk/include sim/include baselines/glue/include)

set(failures "")
foreach(file IN LISTS RACING_NEUTRAL_FILES)
    if(NOT EXISTS ${SOURCE_DIR}/${file})
        list(APPEND failures "${file}: listed but missing")
        continue()
    endif()
    file(STRINGS ${SOURCE_DIR}/${file} lines REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]")
    get_filename_component(dir ${file} DIRECTORY)
    foreach(line IN LISTS lines)
        # The include path, normalized, so `<./branes/racing/x.hpp>` and the
        # like are checked as `<branes/racing/x.hpp>`.
        if(NOT line MATCHES "#[ \t]*include[ \t]*([<\"])([^>\"]+)[>\"]")
            continue()
        endif()
        set(quoted OFF)
        if(CMAKE_MATCH_1 STREQUAL "\"")
            set(quoted ON)
        endif()
        cmake_path(SET path NORMALIZE "${CMAKE_MATCH_2}")
        # A quoted include found beside the file is local; otherwise it is
        # searched on the include path, like an angle include.
        cmake_path(SET local NORMALIZE "${dir}/${path}")
        if(quoted AND EXISTS ${SOURCE_DIR}/${local})
            if(NOT local IN_LIST RACING_NEUTRAL_FILES)
                list(APPEND failures "${file}: includes \"${path}\", which is not mission-neutral")
            endif()
        elseif(path MATCHES "^branes/racing/")
            set(resolved "")
            foreach(root IN LISTS roots)
                if(EXISTS ${SOURCE_DIR}/${root}/${path})
                    set(resolved ${root}/${path})
                endif()
            endforeach()
            if(resolved STREQUAL "")
                list(APPEND failures "${file}: includes <${path}>, which is not a framework header")
            elseif(NOT resolved IN_LIST RACING_NEUTRAL_FILES)
                list(APPEND failures "${file}: includes <${path}>, which is not mission-neutral")
            endif()
        endif()
    endforeach()
endforeach()

list(LENGTH RACING_NEUTRAL_FILES count)
if(failures)
    list(JOIN failures "\n  " text)
    message(FATAL_ERROR "framework boundary violated:\n  ${text}")
endif()
message(STATUS "framework boundary: ${count} neutral files, none includes a racing-only header")
