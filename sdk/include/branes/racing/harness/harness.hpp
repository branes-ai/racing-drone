#pragma once
/// @file harness.hpp
/// @brief Racing's harness pieces, for the harness executables (flight_stack, replay, sitl).
///
/// Racing-only (docs/assessments/framework-boundary.md). The mission-neutral
/// pieces are included from args.hpp, files.hpp, runner.hpp and synthetic.hpp;
/// this header adds:
///  - check_baseline() over racing's slot list (config::kSlotNames);
///  - make_stack(): a racing baseline's Stack, its slots built from the configs.
///
/// The log is double. A stack in another scalar type T (the precision sweep,
/// docs/plan/precision-sweep/) gets its course and inputs cast to T, and its
/// outputs cast back; for T = double every cast is a copy.

#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/compose/stack.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/msg/course.hpp>

#include <filesystem>
#include <string>
#include <string_view>

namespace branes::racing::harness {

/// check_baseline() over racing's seven slots.
[[nodiscard]] inline result<config::BaselineConfig, std::string>
check_baseline(const std::filesystem::path& dir, std::string_view name, std::string_view compiled_hash) {
    return check_baseline(dir, name, compiled_hash, config::kSlotNames);
}

// ── stack construction ──────────────────────────────────────────────

/// Stack::create() for a harness: the course, vehicle, and baseline files
/// make the slot context. The course is cast to the stack's scalar type, and
/// a course that isn't valid() there is an error. Pre: valid(map).
template <class StackT>
[[nodiscard]] result<StackT, std::string>
make_stack(const CourseMap<double>& map, const config::VehicleConfig& vehicle, const config::BaselineConfig& baseline) {
    using T = typename StackT::scalar_type;
    const CourseMap<T> m = cast<T>(map);
    if (!valid(m)) {
        return std::string("the course is not valid in the stack's scalar type");
    }
    return StackT::create(m, SlotContext{vehicle, baseline, map});
}

}  // namespace branes::racing::harness
