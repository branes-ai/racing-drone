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
        if(line MATCHES "#[ \t]*include[ \t]*<(branes/racing/[^>]+)>")
            set(header ${CMAKE_MATCH_1})
            set(resolved "")
            foreach(root IN LISTS roots)
                if(EXISTS ${SOURCE_DIR}/${root}/${header})
                    set(resolved ${root}/${header})
                endif()
            endforeach()
            if(resolved STREQUAL "")
                list(APPEND failures "${file}: includes <${header}>, which is not a framework header")
            elseif(NOT resolved IN_LIST RACING_NEUTRAL_FILES)
                list(APPEND failures "${file}: includes <${header}>, which is not mission-neutral")
            endif()
        elseif(line MATCHES "#[ \t]*include[ \t]*\"([^\"]+)\"")
            set(local ${dir}/${CMAKE_MATCH_1})
            if(NOT local IN_LIST RACING_NEUTRAL_FILES)
                list(APPEND failures "${file}: includes \"${CMAKE_MATCH_1}\", which is not mission-neutral")
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
