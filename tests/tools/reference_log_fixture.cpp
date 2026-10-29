// reference_log_fixture <out.brlog>: writes a log whose reference records hold
// 0, 1, and 3 samples, for tools.log_dump (one CSV row per sample, not per
// record). B0's planner always emits one sample, so a SITL log cannot show the
// difference.

#include <branes/racing/anatomy_mobility.hpp>

#include <cstdint>
#include <cstdio>

namespace racing = branes::racing;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: reference_log_fixture <out.brlog>\n");
        return 1;
    }
    racing::log::LogHeader header;
    header.baseline_name = "fixture";
    header.composition_hash = "none";
    header.world_hash = "none";
    header.vehicle_hash = "none";
    auto writer = racing::log::LogWriter::open(argv[1], header);
    if (!writer) {
        std::fprintf(
            stderr, "reference_log_fixture: %s: %s\n", writer.error().path.c_str(), writer.error().message.c_str());
        return 1;
    }
    std::int64_t t = 0;
    for (const std::uint32_t n : {0u, 1u, 3u}) {
        racing::Reference<double> ref{};
        ref.count = n;
        for (std::uint32_t i = 0; i < n; ++i) {
            ref.samples[i].t = racing::Timestamp{t + 1000 * static_cast<std::int64_t>(i)};
            ref.samples[i].progress = static_cast<double>(i);
        }
        writer->write(racing::log::Channel::reference, racing::Timestamp{t}, racing::codec::encode(ref));
        t += 4'000'000;
    }
    if (auto closed = writer->close(); !closed) {
        std::fprintf(stderr, "reference_log_fixture: %s\n", closed.error().message.c_str());
        return 1;
    }
    return 0;
}
