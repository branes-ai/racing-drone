// The composition hash that cmake/baselines.cmake compiles into a baseline's
// binaries must equal the hash config::composition_hash() computes from the
// baseline file at startup — otherwise every harness would refuse to start.

#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>

#include "racing_baseline.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

using namespace branes::racing;

TEST_CASE("null baseline: generated hash matches the startup hash", "[baseline]") {
    const auto b = config::load_baseline(std::filesystem::path(RACING_SOURCE_DIR) / "baselines/null.yaml");
    REQUIRE(b);
    REQUIRE(baseline::name == "null");
    REQUIRE(config::composition_hash(*b) == baseline::composition_hash);
}

TEST_CASE("null baseline: check_baseline accepts the repo file and rejects a stale hash", "[baseline]") {
    const auto dir = std::filesystem::path(RACING_SOURCE_DIR) / "baselines";
    REQUIRE(harness::check_baseline(dir, baseline::name, baseline::composition_hash));
    const auto stale = harness::check_baseline(dir, baseline::name, "0000");
    REQUIRE_FALSE(stale);
}

TEST_CASE("null baseline: the generated composition instantiates", "[baseline]") {
    const auto course = config::load_course(std::filesystem::path(RACING_SOURCE_DIR) / "courses/a2rl-2025.yaml");
    REQUIRE(course);
    baseline::Composition<double> stack(course->map);
    REQUIRE(valid(stack.on_tick(Timestamp{0}).state));
}
