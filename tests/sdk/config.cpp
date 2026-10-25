// Unit tests for the config loaders (config/config.hpp): the repo's files load,
// emit → parse round-trips to an equal value, and each invariant violation is
// reported with the offending key.

#include <branes/racing/config/config.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

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

// The cases edit the repo's own files (text we control), not emitter output.
std::string rp1_text() {
    return file_text(src / "vehicles/rp1.yaml");
}

std::string course_text() {
    return file_text(src / "courses/a2rl-2025.yaml");
}

}  // namespace

TEST_CASE("vehicle: rp1 loads with the documented values", "[config]") {
    const auto v = load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(v);
    CHECK(v->name == "rp1");
    CHECK(v->mass == 0.966);
    CHECK(v->rotor_count == 4u);
    CHECK(v->camera.width == 820u);
    CHECK(v->camera.height == 616u);
    CHECK(v->camera.model == CameraModel::equidistant);
    CHECK(v->camera.shutter == Shutter::rolling);
    CHECK(is_unit(v->camera.rotation_body));
}

TEST_CASE("vehicle: emit then parse round-trips exactly", "[config]") {
    const auto v = load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(v);
    const auto back = parse_vehicle(emit_vehicle(*v), "emitted");
    REQUIRE(back);
    REQUIRE(*back == *v);
}

TEST_CASE("vehicle: invariant violations name the key", "[config]") {
    const std::string good = rp1_text();
    struct Case {
        std::string from, to, key;
    };
    const Case cases[] = {
        {"mass_kg: 0.966", "mass_kg: 0", "mass_kg"},
        {"rotor_count: 4", "rotor_count: 6", "rotor_count"},
        {"time_constant_s: 0.02", "time_constant_s: -0.02", "rotor.time_constant_s"},
        {"rate_hz: 90", "rate_hz: 0", "camera.rate_hz"},
        {"rotation_body_wxyz: [0.5, -0.5, 0.5, -0.5]", "rotation_body_wxyz: [1, 1, 0, 0]", "camera.rotation_body_wxyz"},
        {"shutter: rolling", "shutter: diagonal", "camera.shutter"},
    };
    for (const auto& c : cases) {
        CAPTURE(c.to);
        const auto r = parse_vehicle(replace(good, c.from, c.to), "case");
        REQUIRE_FALSE(r);
        CHECK(r.error().key == c.key);
    }
}

TEST_CASE("vehicle: unknown and missing keys are errors", "[config]") {
    const std::string good = rp1_text();
    const auto typo = parse_vehicle(replace(good, "arm_length_m", "arm_lenght_m"), "case");
    REQUIRE_FALSE(typo);
    CHECK(typo.error().message == "unknown key");
    const auto r = parse_vehicle("name: x\n", "case");
    REQUIRE_FALSE(r);
    CHECK(r.error().message == "missing");
    CHECK_FALSE(parse_vehicle("[not, a, mapping]", "case"));
    CHECK_FALSE(parse_vehicle("name: [unclosed", "case"));
}

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
    CHECK(std::filesystem::equivalent(s->course_path, src / "courses/a2rl-2025.yaml"));
    CHECK(std::filesystem::equivalent(s->vehicle_path, src / "vehicles/rp1.yaml"));

    const auto back = parse_scenario(emit_scenario(*s), "emitted", src / "scenarios");
    REQUIRE(back);
    REQUIRE(*back == *s);
}

TEST_CASE("scenario: missing files, bad duration, missing seed", "[config]") {
    const auto base = src / "scenarios";
    const std::string head = "name: t\ncourse: ../courses/a2rl-2025.yaml\nvehicle: ../vehicles/rp1.yaml\n";
    CHECK(parse_scenario(head + "duration_s: 1\nseed: 1\n", "case", base));
    CHECK(parse_scenario(head + "duration_s: 0\nseed: 1\n", "case", base).error().key == "duration_s");
    CHECK(parse_scenario(head + "duration_s: 1\n", "case", base).error().key == "seed");
    CHECK(parse_scenario(head + "duration_s: 1\nseed: -3\n", "case", base).error().key == "seed");
    CHECK(parse_scenario(
              "name: t\ncourse: nope.yaml\nvehicle: ../vehicles/rp1.yaml\nduration_s: 1\nseed: 1\n", "case", base)
              .error()
              .key == "course");
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

TEST_CASE("baseline: name rule, missing slot, unknown slot", "[config]") {
    const std::string slots =
        "composition:\n  estimator: A\n  gate_detector: A\n  planner: A\n  controller: A\n  safety_filter: A\n"
        "  daa_tracker: A\n  fc_bridge: A\n";
    CHECK(parse_baseline("name: ok-1\n" + slots, "case"));
    CHECK(parse_baseline("name: Bad_Name\n" + slots, "case").error().key == "name");
    CHECK(parse_baseline("name: x\n" + replace(slots, "  fc_bridge: A\n", ""), "case").error().key ==
          "composition.fc_bridge");
    CHECK(parse_baseline("name: x\n" + slots + "  pilot_arbiter: A\n", "case").error().key ==
          "composition.pilot_arbiter");
}
