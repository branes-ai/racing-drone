// Unit tests for racing's configs (config/config.hpp, config/course.hpp): the
// course loads, emits, and reports each invariant violation with its key; the
// repo's scenario and baseline files load through racing's schema and slot
// list. The mission-neutral loaders are tested in anatomy.

#include <branes/racing/config/config.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>

using namespace branes::racing;
using namespace branes::racing::config;

namespace {

const std::filesystem::path src = RACING_SOURCE_DIR;

/// Replace the first occurrence of `from` in `s` with `to`. Pre: `from` occurs in `s`.
std::string replace(std::string s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    REQUIRE(at != std::string::npos);
    return s.replace(at, from.size(), to);
}

std::string file_text(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    REQUIRE(in);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::string course_text() {
    return file_text(src / "courses/a2rl-2025.yaml");
}

}  // namespace

TEST_CASE("course: a2rl-2025 loads and satisfies CourseMap's invariants", "[config]") {
    const auto c = load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(c);
    CHECK(c->map.gate_count == 11u);
    CHECK(c->map.sequence_length == 11u);
    CHECK(c->map.laps == 2u);
    CHECK(c->map.gates[0].inner_width == 1.5);
    REQUIRE(valid(c->map));
}

TEST_CASE("course: emit then parse round-trips exactly", "[config]") {
    const auto c = load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(c);
    const auto back = parse_course(emit_course(*c), "emitted");
    REQUIRE(back);
    REQUIRE(*back == *c);
}

TEST_CASE("course: invariant violations name the key", "[config]") {
    const std::string good = course_text();
    CHECK(parse_course(replace(good, "laps: 2", "laps: 0"), "case").error().key == "laps");
    CHECK(parse_course(replace(good, "- {id: 2,", "- {id: 1,"), "case").error().key == "gates[1].id");
    CHECK(parse_course(replace(good, "sequence: [1,", "sequence: [42,"), "case").error().key == "sequence[0]");
    CHECK(parse_course(replace(good, "sequence: [1,", "sequence: [1.5,"), "case").error().key == "sequence[0]");
    CHECK(parse_course(replace(good, "max_m: [76.0, 18.0, 5.4]", "max_m: [7.0, 18.0, 5.4]"), "case").error().key ==
          "gates[0].center_m");  // gate 1 at x = 10 is now outside the fence
}

TEST_CASE("scenario: null-smoke resolves its files relative to itself", "[config]") {
    const auto s = load_scenario(src / "scenarios/null-smoke.yaml");
    REQUIRE(s);
    CHECK(s->seed == 20261024u);
    CHECK(s->duration_s == 0.2);
    CHECK(std::filesystem::equivalent(s->world_path, src / "courses/a2rl-2025.yaml"));
    CHECK(std::filesystem::equivalent(s->vehicle_path, src / "vehicles/rp1.yaml"));

    const auto back = parse_scenario(emit_scenario(*s), "emitted", src / "scenarios");
    REQUIRE(back);
    REQUIRE(*back == *s);
}

TEST_CASE("baseline: null loads; composition hash is canonical", "[config]") {
    const auto b = load_baseline(src / "baselines/null.yaml");
    REQUIRE(b);
    CHECK(b->name == "null");
    CHECK(b->composition.size() == kSlotNames.size());
    CHECK(numeric_parameter(*b, "control_rate_hz") == 100.0);
    CHECK_FALSE(numeric_parameter(*b, "absent"));

    const auto back = parse_baseline(emit_baseline(*b), "emitted");
    REQUIRE(back);
    REQUIRE(*back == *b);
    CHECK(composition_hash(*back) == composition_hash(*b));

    auto other = *b;
    other.composition["controller"] = "Mpcc";
    CHECK(composition_hash(other) != composition_hash(*b));  // the hash covers the composition
    other = *b;
    other.parameters["control_rate_hz"] = "200";
    CHECK(composition_hash(other) == composition_hash(*b));  // ... and not the parameters
}

TEST_CASE("scenario: racing's oracle noise is read, checked, and round-trips", "[config]") {
    const std::string head =
        "name: t\ncourse: ../courses/a2rl-2025.yaml\nvehicle: ../vehicles/rp1.yaml\nduration_s: 1\nseed: 1\n";
    const auto plain = parse_scenario(head, "case", src / "scenarios");
    REQUIRE(plain);
    REQUIRE(follows_racing_schema(*plain));
    CHECK(oracle_corner_sigma_px(*plain) == 1.0);  // the fallback

    const auto set = parse_scenario(head + "sim: {oracle_corner_sigma_px: 2.5}\n", "case", src / "scenarios");
    REQUIRE(set);
    CHECK(oracle_corner_sigma_px(*set) == 2.5);
    const auto back = parse_scenario(emit_scenario(*set), "emitted", src / "scenarios");
    REQUIRE(back);
    CHECK(*back == *set);

    const auto negative = parse_scenario(head + "sim: {oracle_corner_sigma_px: -1}\n", "case", src / "scenarios");
    REQUIRE_FALSE(negative);
    CHECK(negative.error().key == "sim.oracle_corner_sigma_px");
    CHECK(negative.error().message == "must be ≥ 0");
}
