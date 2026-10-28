#pragma once
/// @file course.hpp
/// @brief The course file: type, loader, emitter.
///
/// Racing-only (docs/assessments/framework-boundary.md). Every loader validates the
/// whole file before it returns a value, so a config that loads satisfies all
/// the invariants listed on its type. Loading never throws; it returns
/// result<X, ConfigError>. Every emitter writes a file that loads back to an
/// equal value (round trip), with doubles written to 17 significant digits.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/msg/course.hpp>

#include <filesystem>
#include <string>
#include <string_view>

namespace branes::racing::config {

// ── Course ──────────────────────────────────────────────────────────

/// courses/<name>.yaml. Invariants: valid(map) (see CourseMap).
///
/// A gate's orientation is written either as `yaw_rad` (a vertical gate) or as
/// `orientation_wxyz`; the emitter always writes `orientation_wxyz`.
struct CourseConfig {
    std::string name;
    CourseMap<double> map;

    friend bool operator==(const CourseConfig&, const CourseConfig&) = default;
};

[[nodiscard]] result<CourseConfig, ConfigError> load_course(const std::filesystem::path& path);
[[nodiscard]] result<CourseConfig, ConfigError> parse_course(std::string_view yaml, std::string_view origin);
[[nodiscard]] std::string emit_course(const CourseConfig& c);

}  // namespace branes::racing::config
