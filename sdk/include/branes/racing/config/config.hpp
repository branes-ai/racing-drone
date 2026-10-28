#pragma once
/// @file config.hpp
/// @brief Racing's configs: the vehicle, course, scenario, and baseline files.
///
/// Racing-only (docs/assessments/framework-boundary.md). The vehicle, scenario
/// and baseline loaders are mission-neutral (vehicle.hpp, scenario.hpp,
/// baseline.hpp); this header adds what racing supplies to them (its slot list
/// and its scenario schema) and the racing loaders built on them, so racing
/// code loads its files with one-argument calls.
///
/// Configs are load-time data and are stored in double. Components that run in
/// another scalar type convert what they need (e.g. cast<T>(CourseMap)).

#include <branes/racing/anatomy.hpp>
#include <branes/racing/config/course.hpp>

#include <array>
#include <cassert>
#include <filesystem>
#include <string>
#include <string_view>

namespace branes::racing::config {

// ── Scenario ────────────────────────────────────────────────────────

/// Racing's own `sim:` parameters, in the order they are stored and emitted.
inline constexpr std::array<WorldSimParam, 1> kRacingWorldSim = {
    WorldSimParam{"oracle_corner_sigma_px", 1.0, 0.0},  ///< oracle gate corners: 1σ per coordinate [px]
};

/// Racing's scenario schema: the world file is the course (`course:`).
inline constexpr ScenarioSchema kRacingScenario{"course", kRacingWorldSim};

[[nodiscard]] inline result<ScenarioConfig, ConfigError> load_scenario(const std::filesystem::path& path) {
    return load_scenario(path, kRacingScenario);
}

/// `base_dir` resolves the course and vehicle references.
[[nodiscard]] inline result<ScenarioConfig, ConfigError>
parse_scenario(std::string_view yaml, std::string_view origin, const std::filesystem::path& base_dir) {
    return parse_scenario(yaml, origin, base_dir, kRacingScenario);
}

/// Pre: follows_racing_schema(s).
[[nodiscard]] inline std::string emit_scenario(const ScenarioConfig& s) {
    return emit_scenario(s, kRacingScenario);
}

/// True if `s` carries racing's `sim:` parameters (one value per parameter of
/// kRacingWorldSim). Every scenario loaded with kRacingScenario does; a
/// scenario built in code must set `world_sim = default_world_sim(kRacingScenario)`.
[[nodiscard]] inline bool follows_racing_schema(const ScenarioConfig& s) noexcept {
    return s.world_sim.size() == kRacingWorldSim.size();
}

/// The oracle gate corners' noise, 1σ per coordinate [px]
/// (`sim.oracle_corner_sigma_px`). Pre: follows_racing_schema(s).
[[nodiscard]] inline double oracle_corner_sigma_px(const ScenarioConfig& s) noexcept {
    assert(follows_racing_schema(s));  // Pre
    return s.world_sim[0];
}

// ── Baseline ────────────────────────────────────────────────────────

/// Racing's seven slots, sorted (the canonical order of composition_hash()).
inline constexpr std::array<std::string_view, 7> kSlotNames = {
    "controller",
    "daa_tracker",
    "estimator",
    "fc_bridge",
    "gate_detector",
    "planner",
    "safety_filter",
};

[[nodiscard]] inline result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path) {
    return load_baseline(path, kSlotNames);
}

[[nodiscard]] inline result<BaselineConfig, ConfigError> parse_baseline(std::string_view yaml,
                                                                        std::string_view origin) {
    return parse_baseline(yaml, origin, kSlotNames);
}

/// SHA-256 (hex) of racing's canonical composition — identical to the hash
/// cmake/baselines.cmake compiles into each racing binary.
[[nodiscard]] inline std::string composition_hash(const BaselineConfig& b) {
    return composition_hash(b, kSlotNames);
}

}  // namespace branes::racing::config
