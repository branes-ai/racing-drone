// score_log: score a SITL log against its course (test plan §3).
//
//   score_log --log <run.brlog>
//
// Reads the course from the log header, the true state from the `truth`
// channel, and the stack's estimates from the `state` channel, and prints one
// JSON line: success, laps, lap times, gate errors (true and estimated),
// collision, geofence breach, and the first failure. Any baseline's log scores
// the same way, so the scoreboard compares like with like.

#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/sim/score.hpp>

#include <cstdio>
#include <optional>
#include <string>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "score_log: %s\n", message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    if (!in_path) {
        return fail("usage: --log <run.brlog>");
    }
    auto reader = racing::log::LogReader::open(*in_path);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    auto course = racing::config::parse_course(reader->header().course_yaml, "course (from " + *in_path + ")");
    if (!course) {
        return fail(course.error().to_string());
    }
    racing::sim::Scorer scorer(course->map);
    std::uint64_t truths = 0;
    std::optional<racing::Timestamp> last_truth;
    for (;;) {
        auto rec = reader->next();
        if (!rec) {
            return fail(rec.error().path + ": " + rec.error().message);
        }
        if (!rec->has_value()) {
            break;
        }
        const racing::log::LogRecord& r = **rec;
        if (r.channel == racing::log::Channel::truth) {
            const auto x = racing::codec::decode_truth(r.payload);
            if (!x) {
                return fail("undecodable truth record at t=" + std::to_string(r.t.ns));
            }
            // The log is untrusted input: check the scorer's I1 precondition
            // here and report it, as Runner does for ticks.
            if (last_truth && x->t < *last_truth) {
                return fail("truth record at t=" + std::to_string(x->t.ns) +
                            " is earlier than the previous one (t=" + std::to_string(last_truth->ns) + ")");
            }
            last_truth = x->t;
            scorer.on_truth(*x);
            ++truths;
        } else if (r.channel == racing::log::Channel::state) {
            const auto x = racing::codec::decode_state(r.payload);
            if (!x) {
                return fail("undecodable state record at t=" + std::to_string(r.t.ns));
            }
            scorer.on_estimate(*x);
        }
    }
    if (truths == 0) {
        return fail(*in_path + " has no truth channel: score SITL logs only");
    }
    std::printf("%s\n", racing::sim::to_json(scorer.score()).c_str());
    return 0;
}
