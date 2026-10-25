// Unit tests for the replay log (log/log.hpp): records round-trip, identical
// writes give identical files, and damaged files are reported, not misread.

#include <branes/racing/log/log.hpp>

#include <catch2/catch_test_macros.hpp>

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
    h.course_hash = "c";
    h.vehicle_hash = "v";
    h.course_yaml = "name: course\n";
    h.vehicle_yaml = "name: vehicle\n";
    return h;
}

void write_sample(const std::filesystem::path& p) {
    auto w = LogWriter::open(p, sample_header());
    REQUIRE(w);
    const std::vector<std::byte> payload = {std::byte{1}, std::byte{2}, std::byte{3}};
    w->write(Channel::imu, Timestamp{10}, payload);
    w->write(Channel::tick, Timestamp{10}, {});
    w->write(Channel::imu, Timestamp{20}, payload);
    REQUIRE(w->record_count() == 3u);
    REQUIRE(w->close());
    REQUIRE_FALSE(w->is_open());
}

/// Remove a test file without throwing (on Windows a still-open file cannot be removed).
void cleanup(const std::filesystem::path& p) {
    std::error_code ec;
    std::filesystem::remove(p, ec);
}

std::vector<char> read_all(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST_CASE("log: header and records round-trip", "[log]") {
    const auto p = temp_file("roundtrip");
    write_sample(p);

    std::vector<LogRecord> recs;
    {
        // Scoped: the reader closes its file before cleanup() (Windows cannot
        // remove an open file).
        auto r = LogReader::open(p);
        REQUIRE(r);
        REQUIRE(r->header() == sample_header());
        for (;;) {
            auto rec = r->next();
            REQUIRE(rec);
            if (!rec->has_value()) {
                break;
            }
            recs.push_back(**rec);
        }
    }
    REQUIRE(recs.size() == 3u);
    CHECK(recs[0].channel == Channel::imu);
    CHECK(recs[0].t == Timestamp{10});
    CHECK(recs[0].payload.size() == 3u);
    CHECK(recs[1].channel == Channel::tick);
    CHECK(recs[1].payload.empty());
    CHECK(recs[2].t == Timestamp{20});
    cleanup(p);
}

TEST_CASE("log: identical writes give byte-identical files", "[log]") {
    const auto a = temp_file("same_a");
    const auto b = temp_file("same_b");
    write_sample(a);
    write_sample(b);
    REQUIRE(read_all(a) == read_all(b));
    cleanup(a);
    cleanup(b);
}

TEST_CASE("log: a moved-from writer no longer owns the file", "[log]") {
    const auto p = temp_file("moved");
    auto w = LogWriter::open(p, sample_header());
    REQUIRE(w);
    LogWriter moved = std::move(*w);
    REQUIRE(moved.is_open());
    REQUIRE_FALSE(w->is_open());
    REQUIRE(moved.close());
    cleanup(p);
}

TEST_CASE("log: bad magic, truncation, and missing files are errors", "[log]") {
    CHECK_FALSE(LogReader::open(temp_file("does_not_exist")));

    const auto bad = temp_file("bad_magic");
    {
        std::ofstream out(bad, std::ios::binary);
        out << "NOTALOG.................";
    }
    const auto r = LogReader::open(bad);
    REQUIRE_FALSE(r);
    CHECK(r.error().message == "not a racing log (bad magic)");
    cleanup(bad);

    const auto cut = temp_file("truncated");
    write_sample(cut);
    const auto size = std::filesystem::file_size(cut);
    std::filesystem::resize_file(cut, size - 2);  // cut into the last payload
    {
        auto reader = LogReader::open(cut);  // scoped: closed before cleanup()
        REQUIRE(reader);
        REQUIRE(*reader->next());  // records 1 and 2 are intact
        REQUIRE(*reader->next());
        const auto last = reader->next();
        REQUIRE_FALSE(last);
        CHECK(last.error().message == "truncated record payload");
    }
    cleanup(cut);
}
