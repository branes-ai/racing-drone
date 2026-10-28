// Unit tests for the config loaders (config/config.hpp): the repo's files load,
// emit → parse round-trips to an equal value, and each invariant violation is
// reported with the offending key.

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
        {"rate_hz: 1000 ", "rate_hz: 1e12 ", "imu.rate_hz"},
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
    CHECK(std::filesystem::equivalent(s->world_path, src / "courses/a2rl-2025.yaml"));
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

TEST_CASE("vehicle: optional IMU fields default to 0 and are range-checked", "[config]") {
    const auto v = parse_vehicle(rp1_text(), "rp1");
    REQUIRE(v);
    CHECK(v->imu.latency_s == 0.001);
    CHECK(v->imu.initial_gyro_bias_sigma == 5.0e-3);
    // Without the optional keys the vehicle still loads, with zeros.
    std::string bare = rp1_text();
    for (const std::string key : {"  gyro_bias_random_walk:",
                                  "  accel_bias_random_walk:",
                                  "  initial_gyro_bias_sigma:",
                                  "  initial_accel_bias_sigma:",
                                  "  latency_s:"}) {
        const auto at = bare.find(key);
        REQUIRE(at != std::string::npos);
        bare.erase(at, bare.find('\n', at) - at + 1);
    }
    const auto b = parse_vehicle(bare, "bare");
    REQUIRE(b);
    CHECK(b->imu.latency_s == 0);
    CHECK(b->imu.gyro_bias_random_walk == 0);
    CHECK(parse_vehicle(replace(rp1_text(), "latency_s: 0.001", "latency_s: 0.2"), "case").error().key ==
          "imu.latency_s");
    CHECK(parse_vehicle(replace(rp1_text(), "initial_gyro_bias_sigma: 5.0e-3", "initial_gyro_bias_sigma: -1"), "case")
              .error()
              .key == "imu.initial_gyro_bias_sigma");
}

TEST_CASE("scenario: sim, randomization, and faults parse and round-trip", "[config]") {
    const auto s = load_scenario(src / "scenarios/scripted-hop.yaml");
    REQUIRE(s);
    CHECK(s->randomization.mass == 0.05);
    CHECK(s->randomization.start_yaw_rad == 0.1);
    REQUIRE(s->faults.size() == 2);
    CHECK(s->faults[0].type == FaultType::camera_corrupt);
    CHECK(s->faults[0].probability == 0.25);
    CHECK(s->faults[0].until_s == kMaxDurationS);
    CHECK(s->faults[1].type == FaultType::imu_bias_step);
    CHECK(s->faults[1].accel[2] == 0.1);
    CHECK(s->sim == SimParams{});  // no sim block: the defaults
    const auto back = parse_scenario(emit_scenario(*s), "emitted", src / "scenarios");
    REQUIRE(back);
    REQUIRE(*back == *s);
}

TEST_CASE("scenario: sim, randomization, and fault errors name the key", "[config]") {
    const auto base = src / "scenarios";
    const std::string head =
        "name: t\ncourse: ../courses/a2rl-2025.yaml\nvehicle: ../vehicles/rp1.yaml\nduration_s: 1\nseed: 1\n";
    const auto key = [&](const std::string& tail) {
        const auto r = parse_scenario(head + tail, "case", base);
        return r ? std::string("(parsed)") : r.error().key;
    };
    CHECK(key("sim: {physics_rate_hz: 50}\n") == "sim.physics_rate_hz");
    CHECK(key("sim: {command_timeout_s: 0}\n") == "sim.command_timeout_s");
    CHECK(key("sim: {fc_gyro_lowpass_hz: 600}\n") == "sim.fc_gyro_lowpass_hz");
    CHECK(key("sim: {test_pattern_frames: maybe}\n") == "sim.test_pattern_frames");
    CHECK(key("sim: {bogus: 1}\n") == "sim.bogus");
    CHECK(key("randomization: {mass: 1.0}\n") == "randomization.mass");
    CHECK(key("randomization: {start_xy_m: -1}\n") == "randomization.start_xy_m");
    CHECK(key("faults: [{type: gremlin, t_s: 0}]\n") == "faults[0].type");
    CHECK(key("faults: [{type: link_loss, t_s: 2, until_s: 1}]\n") == "faults[0]");
    CHECK(key("faults: [{type: link_loss, t_s: 0, until_s: 1e30}]\n") == "faults[0]");
    CHECK(key("faults: [{type: motor_scale, t_s: 0, rotor: 4, scale: 0.5}]\n") == "faults[0].rotor");
    CHECK(key("faults: [{type: motor_scale, t_s: 0, rotor: 1, scale: 1.5}]\n") == "faults[0].scale");
    CHECK(key("faults: [{type: camera_corrupt, t_s: 0, probability: 2}]\n") == "faults[0].probability");
    CHECK(key("faults: [{type: command_delay, t_s: 0, delay_s: 3}]\n") == "faults[0].delay_s");
    CHECK(key("faults: [{type: link_loss, t_s: 0, probability: 1}]\n") == "faults[0].probability");
    CHECK(key("sim: {physics_rate_hz: 2000, test_pattern_frames: true}\nfaults: [{type: link_loss, t_s: 1}]\n") ==
          "(parsed)");
}

TEST_CASE("scenario: the optional speed limit parses, round-trips, and must be positive", "[config]") {
    const auto s = load_scenario(src / "scenarios/b0-nominal.yaml");
    REQUIRE(s);
    REQUIRE(s->speed_limit_m_s);
    CHECK(*s->speed_limit_m_s == 5.0);
    const auto back = parse_scenario(emit_scenario(*s), "emitted", src / "scenarios");
    REQUIRE(back);
    CHECK(*back == *s);
    const auto none = load_scenario(src / "scenarios/scripted-hop.yaml");
    REQUIRE(none);
    CHECK_FALSE(none->speed_limit_m_s);
    const std::string head =
        "name: t\ncourse: ../courses/a2rl-2025.yaml\nvehicle: ../vehicles/rp1.yaml\nduration_s: 1\nseed: 1\n";
    for (const char* bad : {"speed_limit_m_s: 0\n", "speed_limit_m_s: -1\n", "speed_limit_m_s: fast\n"}) {
        const auto r = parse_scenario(head + bad, "case", src / "scenarios");
        REQUIRE_FALSE(r);
        CHECK(r.error().key == "speed_limit_m_s");
    }
}

TEST_CASE("fault_event_valid checks every invariant", "[config]") {
    FaultEvent f;
    CHECK(fault_event_valid(f));  // the defaults: link_loss over the whole run
    const auto bad = [](auto&& edit) {
        FaultEvent e;
        edit(e);
        return !fault_event_valid(e);
    };
    CHECK(bad([](FaultEvent& e) { e.t_s = -1; }));
    CHECK(bad([](FaultEvent& e) { e.until_s = 0; }));
    CHECK(bad([](FaultEvent& e) { e.until_s = 1e30; }));
    CHECK(bad([](FaultEvent& e) { e.t_s = std::numeric_limits<double>::quiet_NaN(); }));
    CHECK(bad([](FaultEvent& e) { e.probability = 1.5; }));
    CHECK(bad([](FaultEvent& e) { e.delay_s = std::numeric_limits<double>::quiet_NaN(); }));
    CHECK(bad([](FaultEvent& e) { e.scale = -0.1; }));
    CHECK(bad([](FaultEvent& e) { e.rotor = 4; }));
    CHECK(bad([](FaultEvent& e) { e.gyro[1] = std::numeric_limits<double>::infinity(); }));
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

TEST_CASE("scenario: the loader takes another mission's schema", "[config]") {
    // A mission whose world file is a `site:` and that has its own `sim:`
    // parameter: the loader names neither racing key.
    const std::array<WorldSimParam, 1> extra = {WorldSimParam{"depth_sigma_m", 0.05, 0.0}};
    const ScenarioSchema schema{"site", extra};
    const std::string head =
        "name: t\nsite: ../courses/a2rl-2025.yaml\nvehicle: ../vehicles/rp1.yaml\nduration_s: 1\nseed: 1\n";

    const auto s = parse_scenario(head + "sim: {depth_sigma_m: 0.2}\n", "case", src / "scenarios", schema);
    REQUIRE(s);
    CHECK(s->world_ref == "../courses/a2rl-2025.yaml");
    REQUIRE(s->world_sim.size() == 1u);
    CHECK(s->world_sim[0] == 0.2);
    const auto back = parse_scenario(emit_scenario(*s, schema), "emitted", src / "scenarios", schema);
    REQUIRE(back);
    CHECK(*back == *s);

    // Racing's keys are unknown under this schema, and the other way round.
    CHECK(parse_scenario(head + "sim: {oracle_corner_sigma_px: 1}\n", "case", src / "scenarios", schema).error().key ==
          "sim.oracle_corner_sigma_px");
    CHECK(parse_scenario(head, "case", src / "scenarios").error().key == "site");
    const auto missing = parse_scenario(
        "name: t\nvehicle: ../vehicles/rp1.yaml\nduration_s: 1\nseed: 1\n", "case", src / "scenarios", schema);
    REQUIRE_FALSE(missing);
    CHECK(missing.error().key == "site");
}

TEST_CASE("baseline: the loader takes another mission's slot list", "[config]") {
    const std::array<std::string_view, 2> slots = {"controller", "planner"};
    const std::string yaml = "name: two\ncomposition:\n  controller: A\n  planner: B\n";
    const auto b = parse_baseline(yaml, "case", slots);
    REQUIRE(b);
    CHECK(b->composition.size() == 2u);
    CHECK(composition_hash(*b, slots) != composition_hash(*b));  // racing's seven slots hash differently
    CHECK(parse_baseline(yaml, "case").error().key == "composition.daa_tracker");  // racing needs all seven
    CHECK(parse_baseline(yaml + "  estimator: C\n", "case", slots).error().key == "composition.estimator");
}
