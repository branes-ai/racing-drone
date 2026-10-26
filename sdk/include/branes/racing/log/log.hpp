#pragma once
/// @file log.hpp
/// @brief The replay log: LogWriter and LogReader (SYS-08, SW-03).
///
/// File format (all integers little-endian):
///
///   magic        8 bytes  "BRLOG\0\0\0"
///   version      u32      kLogVersion
///   header_size  u32      size of the encoded LogHeader that follows
///   header       LogHeader fields, strings as u32 length + bytes
///   records      repeated until end of file:
///                  channel u16, payload_size u32, t i64 [ns], payload
///
/// The header embeds the course and vehicle files, so a log replays on its own
/// (docs/adr/0001-baseline-composition-framework.md, harness inputs).
/// Phase 0 writes synchronously, off the hot path; the per-channel ring
/// buffers and recorder thread of ADR 0006 replace the write path in Phase 1+.

#include <branes/racing/msg/time.hpp>
#include <branes/racing/result.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace branes::racing::log {

inline constexpr std::uint32_t kLogVersion = 1;

/// Record channels. Inputs (< 16) are what the harness feeds the stack;
/// outputs (≥ 16) are what the stack produced. Sim channels (4–6) are inputs
/// the SITL harness records for scoring and for SITL-only slots; the stack
/// does not consume them yet, and replay copies them through.
enum class Channel : std::uint16_t {
    imu = 1,
    camera = 2,
    tick = 3,
    truth = 4,         ///< TruthState (SITL)
    oracle_gates = 5,  ///< GateObservations projected from the truth (SITL)
    sim_config = 6,    ///< the run's drawn randomization, as YAML text (SITL)
    state = 16,
    command = 17,
};

[[nodiscard]] constexpr bool is_output(Channel c) noexcept {
    return static_cast<std::uint16_t>(c) >= 16;
}

[[nodiscard]] constexpr bool is_sim(Channel c) noexcept {
    const auto v = static_cast<std::uint16_t>(c);
    return v >= 4 && v <= 6;
}

[[nodiscard]] constexpr bool is_known(std::uint16_t c) noexcept {
    return (c >= 1 && c <= 6) || c == 16 || c == 17;
}

/// Run provenance (ADR 0001): what produced the log, and from which inputs.
struct LogHeader {
    std::string baseline_name;
    std::string composition_hash;
    std::string course_hash;
    std::string vehicle_hash;
    std::string scenario_hash{"none"};  ///< "none" when no scenario file was used (flight)
    bool reference{true};               ///< false for what-if replays with overridden inputs
    std::string course_yaml;            ///< the course file, verbatim
    std::string vehicle_yaml;           ///< the vehicle file, verbatim

    friend bool operator==(const LogHeader&, const LogHeader&) = default;
};

struct LogRecord {
    Channel channel{Channel::tick};
    Timestamp t;
    std::vector<std::byte> payload;

    friend bool operator==(const LogRecord&, const LogRecord&) = default;
};

struct LogError {
    std::string path;
    std::string message;
};

/// Appends records to a new log file.
///
/// Model
/// -----
///   out_         : std::ofstream [owned] — the file
///   path_        : the file's path (for errors)
///   open_        : bool — records may be appended
///   last_t_      : per-channel time of the last record
///   records_     : number of records written
///
/// Invariants
/// ----------
///   I1: the header was written exactly once, by open()       — constrains out_
///   I2: records are appended only while open_                — constrains open_
///   I3: within a channel, record times never decrease        — constrains last_t_
class LogWriter {
public:
    /// Create `path` (truncating it) and write the header.
    /// Post (on success): is_open(), record_count() == 0, I1.
    [[nodiscard]] static result<LogWriter, LogError> open(const std::filesystem::path& path, const LogHeader& header);

    /// Post: `other` is closed (it no longer owns the file).
    LogWriter(LogWriter&& other) noexcept
        : out_(std::move(other.out_)), path_(std::move(other.path_)), open_(std::exchange(other.open_, false)),
          last_t_(std::move(other.last_t_)), records_(other.records_) {}
    LogWriter& operator=(LogWriter&& other) noexcept {
        if (this != &other) {
            if (open_) {
                (void)close();
            }
            out_ = std::move(other.out_);
            path_ = std::move(other.path_);
            open_ = std::exchange(other.open_, false);
            last_t_ = std::move(other.last_t_);
            records_ = other.records_;
        }
        return *this;
    }
    LogWriter(const LogWriter&) = delete;
    LogWriter& operator=(const LogWriter&) = delete;
    ~LogWriter();

    /// Append one record. Pre: is_open(); t ≥ the channel's previous record time (I3).
    /// I/O errors are latched and reported by close().
    void write(Channel channel, Timestamp t, std::span<const std::byte> payload);

    /// Flush and close. Post: !is_open(). Returns the first I/O error, if any.
    result<ok_t, LogError> close();

    [[nodiscard]] bool is_open() const noexcept {
        return open_;
    }
    [[nodiscard]] std::uint64_t record_count() const noexcept {
        return records_;
    }

private:
    LogWriter(std::ofstream out, std::filesystem::path path) noexcept
        : out_(std::move(out)), path_(std::move(path)), open_(true) {}

    std::ofstream out_;
    std::filesystem::path path_;
    bool open_{false};
    std::vector<std::pair<Channel, std::int64_t>> last_t_;
    std::uint64_t records_{0};
};

/// Reads a log sequentially.
///
/// Model: in_ [owned] — the file; header_ — the decoded header.
/// Invariants: I1 between calls, the read position is on a record boundary (or at the end).
class LogReader {
public:
    /// Open `path` and validate the magic, version, and header.
    [[nodiscard]] static result<LogReader, LogError> open(const std::filesystem::path& path);

    [[nodiscard]] const LogHeader& header() const noexcept {
        return header_;
    }

    /// The next record; nullopt at a clean end of file; an error for a truncated
    /// record or an unknown channel. Preserves I1.
    [[nodiscard]] result<std::optional<LogRecord>, LogError> next();

private:
    LogReader(std::ifstream in, std::filesystem::path path, LogHeader header) noexcept
        : in_(std::move(in)), path_(std::move(path)), header_(std::move(header)) {}

    std::ifstream in_;
    std::filesystem::path path_;
    LogHeader header_;
};

}  // namespace branes::racing::log
