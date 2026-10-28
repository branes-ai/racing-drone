// Racing's log channel (log/channels.hpp): oracle_gates reads only with
// kRacingChannels. The log itself is tested in anatomy.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/log/channels.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

using namespace branes::racing;
using namespace branes::racing::log;

namespace {

std::filesystem::path temp_file(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("racing_log_test_" + name + ".brlog");
}

LogHeader sample_header() {
    LogHeader h;
    h.baseline_name = "null";
    h.composition_hash = "abc";
    h.world_hash = "c";
    h.vehicle_hash = "v";
    h.world_yaml = "name: course\n";
    h.vehicle_yaml = "name: vehicle\n";
    return h;
}

/// Remove a test file without throwing (on Windows a still-open file cannot be removed).
void cleanup(const std::filesystem::path& p) {
    std::error_code ec;
    std::filesystem::remove(p, ec);
}

}  // namespace

TEST_CASE("log: racing's oracle_gates channel reads only with kRacingChannels", "[log]") {
    const auto p = temp_file("mission_channel");
    {
        auto w = LogWriter::open(p, sample_header());
        REQUIRE(w);
        w->write(oracle_gates, Timestamp{10}, {});
        REQUIRE(w->close());
    }
    {
        auto r = LogReader::open(p, kRacingChannels);  // scoped: closed before cleanup()
        REQUIRE(r);
        const auto rec = r->next();
        REQUIRE(rec);
        REQUIRE(rec->has_value());
        CHECK((*rec)->channel == oracle_gates);
    }
    {
        auto r = LogReader::open(p, {});  // no mission channels: 5 is unknown
        REQUIRE(r);
        const auto rec = r->next();
        REQUIRE_FALSE(rec);
        CHECK(rec.error().message == "unknown channel 5");
    }
    cleanup(p);
}
