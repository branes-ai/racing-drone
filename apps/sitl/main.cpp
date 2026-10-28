// sitl_<baseline>: run a baseline against a simulated vehicle and write the log.
//
//   sitl_<baseline> --scenario <scenario.yaml> --out <run.brlog>
//
// The scenario names the course and vehicle (ADR 0001); --course and --vehicle
// are rejected, so there is one source of truth. The vehicle flies on the
// reflex dynamics model (sim::Simulator, docs/plan/sitl/): each tick's command
// goes to the simulated flight controller, which drives the plant. The log
// holds every input event, the sim channels (truth, oracle gates, the drawn
// randomization), and every output, so replay_<baseline> can reproduce the run.

#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/sim/simulator.hpp>
#include <branes/racing/sim/sitl_loop.hpp>

#include "racing_baseline.hpp"

#include <cstdio>
#include <string>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "sitl_%s: %s\n", std::string(racing::baseline::name).c_str(), message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (args->has("course") || args->has("vehicle")) {
        return fail("--course / --vehicle are not accepted: the scenario names them");
    }
    if (auto bad = args->only({"scenario", "out"})) {
        return fail(*bad);
    }
    const auto scenario_path = args->value("scenario");
    const auto out_path = args->value("out");
    if (!scenario_path || !out_path) {
        return fail("usage: --scenario <scenario.yaml> --out <run.brlog>");
    }

    auto baseline = racing::harness::check_baseline(
        RACING_BASELINES_DIR, racing::baseline::name, racing::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }
    const double control_hz =
        racing::config::numeric_parameter(*baseline, "control_rate_hz").value_or(racing::config::kDefaultControlRateHz);

    auto scenario = racing::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    auto course = racing::config::load_course(scenario->world_path);
    if (!course) {
        return fail(course.error().to_string());
    }
    auto vehicle = racing::config::load_vehicle(scenario->vehicle_path);
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }
    const auto course_file = racing::harness::read_with_hash(scenario->world_path);
    const auto vehicle_file = racing::harness::read_with_hash(scenario->vehicle_path);
    const auto scenario_hash = racing::sha256_file_hex(*scenario_path);
    if (!course_file || !vehicle_file || !scenario_hash) {
        return fail("cannot read the scenario's input files");
    }

    // Validate everything before LogWriter::open truncates --out.
    auto sim = racing::sim::Simulator::create(*vehicle, course->map, *scenario, control_hz);
    if (!sim) {
        return fail(sim.error());
    }

    auto built = racing::harness::make_stack<racing::baseline::Composition<double>>(course->map, *vehicle, *baseline);
    if (!built) {
        return fail(built.error());
    }
    auto& stack = *built;

    racing::log::LogHeader header;
    header.baseline_name = std::string(racing::baseline::name);
    header.composition_hash = std::string(racing::baseline::composition_hash);
    header.world_hash = course_file->hash;
    header.vehicle_hash = vehicle_file->hash;
    header.scenario_hash = *scenario_hash;
    header.world_yaml = course_file->text;
    header.vehicle_yaml = vehicle_file->text;
    auto writer = racing::log::LogWriter::open(*out_path, header);
    if (!writer) {
        return fail(writer.error().path + ": " + writer.error().message);
    }

    const auto ticks = racing::sim::run_sitl(*sim, stack, racing::log::kRacingChannels, *writer);
    if (!ticks) {
        return fail(ticks.error());
    }
    const auto records = writer->record_count();
    if (auto closed = writer->close(); !closed) {
        return fail(closed.error().path + ": " + closed.error().message);
    }
    std::printf("sitl_%s: %s, %llu ticks, %llu records -> %s\n",
                std::string(racing::baseline::name).c_str(),
                scenario->name.c_str(),
                static_cast<unsigned long long>(*ticks),
                static_cast<unsigned long long>(records),
                out_path->c_str());
    return 0;
}
