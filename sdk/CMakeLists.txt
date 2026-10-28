# sdk/ — the racing framework library.
#
# `branes::racing` carries the data contracts (msg/), slot concepts and null
# implementations (slots/), the Stack composition (compose/), and the harness
# pieces (harness/) as headers, and compiles the config loaders (yaml-cpp),
# the replay log, and SHA-256. Everything is templated on the scalar type
# except the load-time configs and the log, which are double.

add_library(branes_racing STATIC
    src/config/baseline.cpp
    src/config/course.cpp
    src/config/scenario.cpp
    src/config/vehicle.cpp
    src/log/log.cpp
    src/util/sha256.cpp
)
add_library(branes::racing ALIAS branes_racing)

target_include_directories(branes_racing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_racing PUBLIC cxx_std_20)
# yaml-cpp stays an implementation detail of the loaders.
target_link_libraries(branes_racing PRIVATE yaml-cpp::yaml-cpp)
racing_set_warnings(branes_racing)
set_target_properties(branes_racing PROPERTIES FOLDER "SDK")
