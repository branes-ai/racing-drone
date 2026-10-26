#pragma once
/// @file config.hpp
/// @brief Vehicle, course, scenario, and baseline files: types, loaders, emitters.
///
/// Every loader validates the whole file before it returns a value, so a
/// config that loads satisfies all the invariants listed on its type. Loading
/// never throws; it returns result<X, ConfigError>. Every emitter writes a file
/// that loads back to an equal value (round trip), with doubles written to 17
/// significant digits.
///
/// Configs are load-time data and are stored in double. Components that run in
/// another scalar type convert what they need (e.g. cast<T>(CourseMap)).

#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/geometry.hpp>
#include <branes/racing/msg/sensors.hpp>
#include <branes/racing/msg/time.hpp>
#include <branes/racing/result.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace branes::racing::config {

/// What went wrong, and where.
struct ConfigError {
    std::string file;  ///< file or origin name
    std::string key;   ///< YAML key path, e.g. "camera.fx" (empty if not key-specific)
    std::string message;

    [[nodiscard]] std::string to_string() const {
        return file + (key.empty() ? "" : ": " + key) + ": " + message;
    }
};

// ── Vehicle ─────────────────────────────────────────────────────────

enum class CameraModel : std::uint8_t {
    pinhole_radtan = 0,
    equidistant = 1,  ///< Kannala-Brandt
};

enum class Shutter : std::uint8_t {
    global = 0,
    rolling = 1,
};

struct RotorParams {
    double thrust_coefficient{0};  ///< thrust = k_f·ω² [N / (rad/s)²]
    double torque_coefficient{0};  ///< torque = k_m·thrust [m]
    double max_thrust{0};          ///< per rotor [N]
    double time_constant{0};       ///< first-order motor time constant [s]

    friend bool operator==(const RotorParams&, const RotorParams&) = default;
};

struct DragParams {
    Vec3<double> linear{};     ///< body-frame linear drag [N / (m/s)]
    Vec3<double> quadratic{};  ///< body-frame quadratic drag, the v|v| term [N / (m/s)²]

    friend bool operator==(const DragParams&, const DragParams&) = default;
};

/// Largest IMU latency a vehicle file may give [s].
inline constexpr double kMaxImuLatencyS = 0.1;

struct ImuParams {
    double rate_hz{0};
    double gyro_range{0};           ///< per axis [rad/s]
    double accel_range{0};          ///< per axis [m/s²]
    double gyro_noise_density{0};   ///< [rad/s/√Hz]
    double accel_noise_density{0};  ///< [m/s²/√Hz]
    // Optional (default 0), used by the SITL IMU model (sim/imu.hpp).
    double gyro_bias_random_walk{0};     ///< [rad/s²/√Hz]
    double accel_bias_random_walk{0};    ///< [m/s³/√Hz]
    double initial_gyro_bias_sigma{0};   ///< turn-on bias, 1σ per axis [rad/s]
    double initial_accel_bias_sigma{0};  ///< turn-on bias, 1σ per axis [m/s²]
    double latency_s{0};                 ///< measurement → delivery [s], ≤ kMaxImuLatencyS

    friend bool operator==(const ImuParams&, const ImuParams&) = default;
};

/// Camera frame convention: z forward, x right, y down.
struct CameraParams {
    CameraModel model{CameraModel::equidistant};
    std::uint32_t width{0};
    std::uint32_t height{0};
    PixelFormat format{PixelFormat::mono8};
    double fx{0};
    double fy{0};
    double cx{0};
    double cy{0};
    std::array<double, 4> distortion{};  ///< k1..k4 (equidistant) or k1, k2, p1, p2 (radtan)
    double rate_hz{0};
    Shutter shutter{Shutter::rolling};
    std::int64_t row_readout_ns{0};
    Vec3<double> translation_body{};  ///< camera origin in the body frame [m]
    Quat<double> rotation_body{};     ///< camera → body

    friend bool operator==(const CameraParams&, const CameraParams&) = default;
};

/// vehicles/<name>.yaml.
///
/// Invariants: mass > 0; inertia diagonal > 0; arm_length > 0; rotor_count == 4;
/// rotor parameters > 0; drag ≥ 0; IMU rate, ranges > 0 and noise ≥ 0; camera
/// width, height, fx, fy, rate > 0; row_readout_ns ≥ 0; rotation_body is unit.
struct VehicleConfig {
    std::string name;
    double mass{0};          ///< [kg]
    Vec3<double> inertia{};  ///< diagonal [kg·m²]
    double arm_length{0};    ///< center to rotor axis [m]
    std::uint32_t rotor_count{0};
    RotorParams rotor;
    DragParams drag;
    ImuParams imu;
    CameraParams camera;

    friend bool operator==(const VehicleConfig&, const VehicleConfig&) = default;
};

[[nodiscard]] result<VehicleConfig, ConfigError> load_vehicle(const std::filesystem::path& path);
[[nodiscard]] result<VehicleConfig, ConfigError> parse_vehicle(std::string_view yaml, std::string_view origin);
[[nodiscard]] std::string emit_vehicle(const VehicleConfig& v);

// ── Course ──────────────────────────────────────────────────────────

/// courses/<name>.yaml. Invariants: valid(map) (see CourseMap).
///
/// A gate's orientation is written either as `yaw_rad` (a vertical gate) or as
/// `orientation_wxyz`; the emitter always writes `orientation_wxyz`.
struct CourseConfig {
    std::string name;
    CourseMap<double> map;

    friend bool operator==(const CourseConfig&, const CourseConfig&) = default;
};

[[nodiscard]] result<CourseConfig, ConfigError> load_course(const std::filesystem::path& path);
[[nodiscard]] result<CourseConfig, ConfigError> parse_course(std::string_view yaml, std::string_view origin);
[[nodiscard]] std::string emit_course(const CourseConfig& c);

// ── Scenario ────────────────────────────────────────────────────────

/// SITL settings of a scenario (`sim:`), all optional.
struct SimParams {
    double physics_rate_hz{1000};      ///< plant and simulated-FC step rate
    double command_timeout_s{0.1};     ///< simulated FC: no command for longer → rotors off
    double fc_gyro_lowpass_hz{100};    ///< simulated FC's gyro filter cutoff
    double oracle_corner_sigma_px{1};  ///< oracle gate corners: 1σ per coordinate [px]
    bool test_pattern_frames{false};   ///< camera records carry a test-pattern image (else header only)

    friend bool operator==(const SimParams&, const SimParams&) = default;
};

/// Per-run randomization (`randomization:`), drawn from the seed. Each field
/// is a half-width: a parameter p becomes p·(1 + u), u uniform in [−f, f].
struct Randomization {
    double mass{0};
    double inertia{0};
    double drag{0};
    double motor_time_constant{0};
    double start_xy_m{0};     ///< start position jitter per axis [m]
    double start_yaw_rad{0};  ///< start yaw jitter [rad]

    friend bool operator==(const Randomization&, const Randomization&) = default;
};

enum class FaultType : std::uint8_t {
    imu_bias_step,    ///< add `gyro` / `accel` to the IMU biases at t_s
    camera_corrupt,   ///< mark each frame corrupted with `probability`
    camera_blackout,  ///< deliver no frames
    command_delay,    ///< delay commands to the simulated FC by `delay_s`
    link_loss,        ///< drop commands to the simulated FC
    motor_scale,      ///< rotor `rotor` delivers `scale` of its commanded thrust
};

/// One timed fault (`faults:` entry). Active on [t_s, until_s); until_s
/// defaults to the end of the run. imu_bias_step acts once, at t_s.
///
/// Invariants (fault_event_valid): every field finite;
/// 0 ≤ t_s < until_s ≤ kMaxDurationS; probability, delay_s, scale ∈ [0, 1];
/// rotor < 4.
struct FaultEvent {
    FaultType type{FaultType::link_loss};
    double t_s{0};
    double until_s{kMaxDurationS};
    Vec3<double> gyro{};     ///< imu_bias_step [rad/s]
    Vec3<double> accel{};    ///< imu_bias_step [m/s²]
    double probability{0};   ///< camera_corrupt, ∈ [0, 1]
    double delay_s{0};       ///< command_delay, ∈ [0, 1] s
    std::uint32_t rotor{0};  ///< motor_scale, < 4
    double scale{1};         ///< motor_scale, ∈ [0, 1]

    friend bool operator==(const FaultEvent&, const FaultEvent&) = default;
};

[[nodiscard]] std::string_view name_of(FaultType t) noexcept;

/// True if `f` satisfies FaultEvent's invariants. The scenario loader enforces
/// them field by field; a Simulator built from a programmatic scenario checks
/// them with this.
[[nodiscard]] bool fault_event_valid(const FaultEvent& f) noexcept;

/// scenarios/<name>.yaml. Course and vehicle paths are relative to the scenario
/// file's directory.
///
/// Invariants: the course and vehicle files exist; duration_s > 0; seed is given;
/// sim rates and limits are in range; randomization fractions ∈ [0, 1); every
/// fault has 0 ≤ t_s < until_s and its own fields in range.
struct ScenarioConfig {
    std::string name;
    std::string course_ref;              ///< as written in the file
    std::string vehicle_ref;             ///< as written in the file
    std::filesystem::path course_path;   ///< resolved
    std::filesystem::path vehicle_path;  ///< resolved
    double duration_s{0};
    std::uint64_t seed{0};
    SimParams sim;                   ///< optional `sim:` block
    Randomization randomization;     ///< optional `randomization:` block
    std::vector<FaultEvent> faults;  ///< optional `faults:` list, in file order

    friend bool operator==(const ScenarioConfig&, const ScenarioConfig&) = default;
};

[[nodiscard]] result<ScenarioConfig, ConfigError> load_scenario(const std::filesystem::path& path);
/// `base_dir` resolves the course and vehicle references.
[[nodiscard]] result<ScenarioConfig, ConfigError>
parse_scenario(std::string_view yaml, std::string_view origin, const std::filesystem::path& base_dir);
[[nodiscard]] std::string emit_scenario(const ScenarioConfig& s);

// ── Baseline ────────────────────────────────────────────────────────

/// The seven slots, sorted (the canonical order of composition_hash()).
inline constexpr std::array<std::string_view, 7> kSlotNames = {
    "controller",
    "daa_tracker",
    "estimator",
    "fc_bridge",
    "gate_detector",
    "planner",
    "safety_filter",
};

/// baselines/<name>.yaml.
///
/// Invariants: name matches [a-z0-9-]+; composition names exactly the seven
/// slots of kSlotNames, each with an implementation name [A-Za-z0-9_]+;
/// parameters are scalar values.
struct BaselineConfig {
    std::string name;
    std::map<std::string, std::string> composition;  ///< slot → implementation
    std::map<std::string, std::string> parameters;   ///< key → scalar value

    friend bool operator==(const BaselineConfig&, const BaselineConfig&) = default;
};

[[nodiscard]] result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path);
[[nodiscard]] result<BaselineConfig, ConfigError> parse_baseline(std::string_view yaml, std::string_view origin);
[[nodiscard]] std::string emit_baseline(const BaselineConfig& b);

/// SHA-256 (hex) of the canonical composition "slot=Impl\n", slots sorted —
/// identical to the hash cmake/baselines.cmake compiles into each binary.
[[nodiscard]] std::string composition_hash(const BaselineConfig& b);

/// A numeric parameter, or nullopt if absent or not a number.
[[nodiscard]] std::optional<double> numeric_parameter(const BaselineConfig& b, std::string_view key);

}  // namespace branes::racing::config
