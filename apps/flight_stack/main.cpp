// flight_stack_<baseline>: the companion flight-stack executable.
//
//   flight_stack_<baseline> --course <course.yaml> --vehicle <vehicle.yaml>
//                           [--scenario <scenario.yaml>] [--cycles <n>]
//
// Flight takes the course and vehicle explicitly; HIL adds --scenario, whose
// course and vehicle must be the same files (ADR 0001). Phase 0 has no sensor
// drivers or FC link yet: the stack runs <n> control cycles (default 0) at the
// baseline's control rate against its FcBridge and exits. Drivers, the real-
// time loop, and the FC bridge replace that in Phases 1-4.

#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/msg/time.hpp>

#include "racing_baseline.hpp"

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "flight_stack_%s: %s\n", std::string(racing::baseline::name).c_str(), message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"course", "vehicle", "scenario", "cycles"})) {
        return fail(*bad);
    }
    const auto course_path = args->value("course");
    const auto vehicle_path = args->value("vehicle");
    if (!course_path || !vehicle_path) {
        return fail("usage: --course <course.yaml> --vehicle <vehicle.yaml> [--scenario <yaml>] [--cycles <n>]");
    }
    long long cycles = 0;
    if (const auto c = args->value("cycles")) {
        // The whole argument must be the number: "10x" and "1e3" are rejected.
        const auto [end, ec] = std::from_chars(c->data(), c->data() + c->size(), cycles);
        if (ec != std::errc{} || end != c->data() + c->size() || cycles < 0) {
            return fail("--cycles must be a non-negative integer");
        }
    }

    auto baseline = racing::harness::check_baseline(
        RACING_BASELINES_DIR, racing::baseline::name, racing::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }
    const double control_hz = racing::config::numeric_parameter(*baseline, "control_rate_hz").value_or(100.0);
    const auto period = racing::checked_period_ns(control_hz);
    if (!period) {
        return fail("parameter control_rate_hz must be in [" + std::to_string(racing::kMinRateHz) + ", " +
                    std::to_string(racing::kMaxRateHz) + "] Hz");
    }

    auto course = racing::config::load_course(*course_path);
    if (!course) {
        return fail(course.error().to_string());
    }
    auto vehicle = racing::config::load_vehicle(*vehicle_path);
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }

    // HIL: the scenario must name the same course and vehicle files.
    if (const auto s = args->value("scenario")) {
        auto scenario = racing::config::load_scenario(*s);
        if (!scenario) {
            return fail(scenario.error().to_string());
        }
        const auto same_file = [](const std::string& a, const std::filesystem::path& b) {
            const auto ha = racing::sha256_file_hex(a);
            const auto hb = racing::sha256_file_hex(b);
            return ha && hb && *ha == *hb;
        };
        if (!same_file(*course_path, scenario->course_path) || !same_file(*vehicle_path, scenario->vehicle_path)) {
            return fail("the scenario's course / vehicle differ from --course / --vehicle");
        }
    }

    // Tick times are i · period; keep the last one inside int64 (no signed overflow).
    if (cycles > 0 && cycles - 1 > std::numeric_limits<std::int64_t>::max() / *period) {
        return fail("--cycles too large for the control rate (tick time would overflow)");
    }

    auto built = racing::harness::make_stack<racing::baseline::Composition<double>>(course->map, *vehicle, *baseline);
    if (!built) {
        return fail(built.error());
    }
    auto& stack = *built;
    for (long long i = 0; i < cycles; ++i) {
        (void)stack.on_tick(racing::Timestamp{i * *period});
    }
    std::printf("flight_stack_%s: course %s, vehicle %s, %lld cycles at %.0f Hz\n",
                std::string(racing::baseline::name).c_str(),
                course->name.c_str(),
                vehicle->name.c_str(),
                cycles,
                control_hz);
    return 0;
}
