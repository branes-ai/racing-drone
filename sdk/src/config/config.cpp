#include <branes/racing/config/config.hpp>
#include <branes/racing/util/sha256.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <sstream>

namespace branes::racing::config {

namespace {

// ── parsing helpers ─────────────────────────────────────────────────
// Parsing uses exceptions internally (yaml-cpp throws); every public entry
// point catches them and returns a ConfigError, so the API never throws.

struct Fail {
    std::string key;
    std::string message;
};

std::string join(std::string_view prefix, std::string_view key) {
    return prefix.empty() ? std::string(key) : std::string(prefix) + "." + std::string(key);
}

YAML::Node child(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = n[key];
    if (!c) {
        throw Fail{join(prefix, key), "missing"};
    }
    return c;
}

/// Reject keys outside `allowed`, so a typo is an error rather than a silently
/// ignored setting.
void only_keys(const YAML::Node& n, std::string_view prefix, std::initializer_list<std::string_view> allowed) {
    if (!n.IsMap()) {
        throw Fail{std::string(prefix), "expected a mapping"};
    }
    for (const auto& kv : n) {
        const auto key = kv.first.as<std::string>();
        bool known = false;
        for (const auto a : allowed) {
            known = known || key == a;
        }
        if (!known) {
            throw Fail{join(prefix, key), "unknown key"};
        }
    }
}

double as_double(const YAML::Node& n, const std::string& where) {
    if (!n.IsScalar()) {
        throw Fail{where, "expected a number"};
    }
    try {
        const double v = n.as<double>();
        if (!std::isfinite(v)) {
            throw Fail{where, "must be finite"};
        }
        return v;
    } catch (const YAML::Exception&) {
        throw Fail{where, "expected a number"};
    }
}

double get_double(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    return as_double(child(n, prefix, key), join(prefix, key));
}

std::uint64_t get_uint(const YAML::Node& n, std::string_view prefix, const std::string& key, std::uint64_t max) {
    const YAML::Node c = child(n, prefix, key);
    const std::string where = join(prefix, key);
    if (!c.IsScalar()) {
        throw Fail{where, "expected a non-negative integer"};
    }
    const auto text = c.as<std::string>();
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
        throw Fail{where, "expected a non-negative integer"};
    }
    try {
        const std::uint64_t v = c.as<std::uint64_t>();
        if (v > max) {
            throw Fail{where, "out of range"};
        }
        return v;
    } catch (const YAML::Exception&) {
        throw Fail{where, "out of range"};
    }
}

/// A gate id written as a non-negative integer that fits an int32.
std::int32_t as_gate_id(const YAML::Node& n, const std::string& where) {
    if (!n.IsScalar()) {
        throw Fail{where, "expected a gate id"};
    }
    const auto text = n.as<std::string>();
    if (text.empty() || text.size() > 9 || text.find_first_not_of("0123456789") != std::string::npos) {
        throw Fail{where, "expected a gate id (a non-negative integer)"};
    }
    return static_cast<std::int32_t>(std::stol(text));
}

std::int64_t get_int64(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = child(n, prefix, key);
    const std::string where = join(prefix, key);
    try {
        return c.as<std::int64_t>();
    } catch (const YAML::Exception&) {
        throw Fail{where, "expected an integer"};
    }
}

std::string get_string(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = child(n, prefix, key);
    if (!c.IsScalar()) {
        throw Fail{join(prefix, key), "expected a string"};
    }
    return c.as<std::string>();
}

template <std::size_t N>
std::array<double, N> get_array(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = child(n, prefix, key);
    const std::string where = join(prefix, key);
    if (!c.IsSequence() || c.size() != N) {
        throw Fail{where, "expected a list of " + std::to_string(N) + " numbers"};
    }
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        out[i] = as_double(c[i], where + "[" + std::to_string(i) + "]");
    }
    return out;
}

Quat<double> get_quat(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const auto a = get_array<4>(n, prefix, key);
    const Quat<double> q{a[0], a[1], a[2], a[3]};
    if (!is_unit(q)) {
        throw Fail{join(prefix, key), "must be a unit quaternion (w, x, y, z)"};
    }
    return q;
}

void require(bool condition, std::string key, std::string message) {
    if (!condition) {
        throw Fail{std::move(key), std::move(message)};
    }
}

YAML::Node parse_root(std::string_view yaml) {
    try {
        YAML::Node root = YAML::Load(std::string(yaml));
        if (!root.IsMap()) {
            throw Fail{"", "the file must be a YAML mapping"};
        }
        return root;
    } catch (const YAML::ParserException& e) {
        throw Fail{"", std::string("YAML syntax error: ") + e.what()};
    }
}

/// Run `body`, translating every failure into a ConfigError for `origin`.
template <typename T, typename F>
result<T, ConfigError> guarded(std::string_view origin, F&& body) {
    try {
        return body();
    } catch (const Fail& f) {
        return ConfigError{std::string(origin), f.key, f.message};
    } catch (const YAML::Exception& e) {
        return ConfigError{std::string(origin), "", e.what()};
    }
}

std::optional<std::string> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

template <typename T, typename Parse>
result<T, ConfigError> load_file(const std::filesystem::path& path, Parse&& parse) {
    const auto text = read_file(path);
    if (!text) {
        return ConfigError{path.string(), "", "cannot read the file"};
    }
    return parse(*text, path.string());
}

// ── emitting helpers ────────────────────────────────────────────────

YAML::Emitter& emit_seq(YAML::Emitter& out, std::span<const double> values) {
    out << YAML::Flow << YAML::BeginSeq;
    for (const double v : values) {
        out << v;
    }
    return out << YAML::EndSeq;
}

YAML::Emitter& emit_quat(YAML::Emitter& out, const Quat<double>& q) {
    const std::array<double, 4> a = {q.w, q.x, q.y, q.z};
    return emit_seq(out, a);
}

std::string finish(YAML::Emitter& out) {
    return std::string(out.c_str()) + "\n";
}

// ── enum names ──────────────────────────────────────────────────────

CameraModel camera_model(const std::string& s, const std::string& where) {
    if (s == "pinhole_radtan") {
        return CameraModel::pinhole_radtan;
    }
    if (s == "equidistant") {
        return CameraModel::equidistant;
    }
    throw Fail{where, "expected pinhole_radtan or equidistant"};
}
const char* name_of(CameraModel m) {
    return m == CameraModel::pinhole_radtan ? "pinhole_radtan" : "equidistant";
}

Shutter shutter(const std::string& s, const std::string& where) {
    if (s == "global") {
        return Shutter::global;
    }
    if (s == "rolling") {
        return Shutter::rolling;
    }
    throw Fail{where, "expected global or rolling"};
}
const char* name_of(Shutter s) {
    return s == Shutter::global ? "global" : "rolling";
}

PixelFormat pixel_format(const std::string& s, const std::string& where) {
    if (s == "mono8") {
        return PixelFormat::mono8;
    }
    if (s == "rgb8") {
        return PixelFormat::rgb8;
    }
    throw Fail{where, "expected mono8 or rgb8"};
}
const char* name_of(PixelFormat f) {
    return f == PixelFormat::rgb8 ? "rgb8" : "mono8";
}

bool all_positive(const Vec3<double>& v) {
    return v[0] > 0 && v[1] > 0 && v[2] > 0;
}
bool all_non_negative(const Vec3<double>& v) {
    return v[0] >= 0 && v[1] >= 0 && v[2] >= 0;
}

}  // namespace

// ── Vehicle ─────────────────────────────────────────────────────────

result<VehicleConfig, ConfigError> parse_vehicle(std::string_view yaml, std::string_view origin) {
    return guarded<VehicleConfig>(origin, [&]() -> VehicleConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root,
                  "",
                  {"name", "mass_kg", "inertia_kgm2", "arm_length_m", "rotor_count", "rotor", "drag", "imu", "camera"});
        VehicleConfig v;
        v.name = get_string(root, "", "name");
        v.mass = get_double(root, "", "mass_kg");
        require(v.mass > 0, "mass_kg", "must be > 0");
        v.inertia = get_array<3>(root, "", "inertia_kgm2");
        require(all_positive(v.inertia), "inertia_kgm2", "every diagonal entry must be > 0");
        v.arm_length = get_double(root, "", "arm_length_m");
        require(v.arm_length > 0, "arm_length_m", "must be > 0");
        v.rotor_count = static_cast<std::uint32_t>(get_uint(root, "", "rotor_count", 64));
        require(v.rotor_count == 4, "rotor_count", "only quadrotors (4) are supported");

        const YAML::Node rotor = child(root, "", "rotor");
        only_keys(rotor, "rotor", {"thrust_coefficient", "torque_coefficient", "max_thrust_n", "time_constant_s"});
        v.rotor.thrust_coefficient = get_double(rotor, "rotor", "thrust_coefficient");
        v.rotor.torque_coefficient = get_double(rotor, "rotor", "torque_coefficient");
        v.rotor.max_thrust = get_double(rotor, "rotor", "max_thrust_n");
        v.rotor.time_constant = get_double(rotor, "rotor", "time_constant_s");
        require(v.rotor.thrust_coefficient > 0, "rotor.thrust_coefficient", "must be > 0");
        require(v.rotor.torque_coefficient > 0, "rotor.torque_coefficient", "must be > 0");
        require(v.rotor.max_thrust > 0, "rotor.max_thrust_n", "must be > 0");
        require(v.rotor.time_constant > 0, "rotor.time_constant_s", "must be > 0");

        const YAML::Node drag = child(root, "", "drag");
        only_keys(drag, "drag", {"linear", "quadratic"});
        v.drag.linear = get_array<3>(drag, "drag", "linear");
        v.drag.quadratic = get_array<3>(drag, "drag", "quadratic");
        require(all_non_negative(v.drag.linear), "drag.linear", "must be ≥ 0");
        require(all_non_negative(v.drag.quadratic), "drag.quadratic", "must be ≥ 0");

        const YAML::Node imu = child(root, "", "imu");
        only_keys(imu,
                  "imu",
                  {"rate_hz", "gyro_range_rad_s", "accel_range_m_s2", "gyro_noise_density", "accel_noise_density"});
        v.imu.rate_hz = get_double(imu, "imu", "rate_hz");
        v.imu.gyro_range = get_double(imu, "imu", "gyro_range_rad_s");
        v.imu.accel_range = get_double(imu, "imu", "accel_range_m_s2");
        v.imu.gyro_noise_density = get_double(imu, "imu", "gyro_noise_density");
        v.imu.accel_noise_density = get_double(imu, "imu", "accel_noise_density");
        require(v.imu.rate_hz > 0, "imu.rate_hz", "must be > 0");
        require(v.imu.gyro_range > 0, "imu.gyro_range_rad_s", "must be > 0");
        require(v.imu.accel_range > 0, "imu.accel_range_m_s2", "must be > 0");
        require(v.imu.gyro_noise_density >= 0, "imu.gyro_noise_density", "must be ≥ 0");
        require(v.imu.accel_noise_density >= 0, "imu.accel_noise_density", "must be ≥ 0");

        const YAML::Node cam = child(root, "", "camera");
        only_keys(cam,
                  "camera",
                  {"model",
                   "width",
                   "height",
                   "format",
                   "fx",
                   "fy",
                   "cx",
                   "cy",
                   "distortion",
                   "rate_hz",
                   "shutter",
                   "row_readout_ns",
                   "translation_body_m",
                   "rotation_body_wxyz"});
        v.camera.model = camera_model(get_string(cam, "camera", "model"), "camera.model");
        v.camera.width = static_cast<std::uint32_t>(get_uint(cam, "camera", "width", 1u << 16));
        v.camera.height = static_cast<std::uint32_t>(get_uint(cam, "camera", "height", 1u << 16));
        v.camera.format = pixel_format(get_string(cam, "camera", "format"), "camera.format");
        v.camera.fx = get_double(cam, "camera", "fx");
        v.camera.fy = get_double(cam, "camera", "fy");
        v.camera.cx = get_double(cam, "camera", "cx");
        v.camera.cy = get_double(cam, "camera", "cy");
        v.camera.distortion = get_array<4>(cam, "camera", "distortion");
        v.camera.rate_hz = get_double(cam, "camera", "rate_hz");
        v.camera.shutter = shutter(get_string(cam, "camera", "shutter"), "camera.shutter");
        v.camera.row_readout_ns = get_int64(cam, "camera", "row_readout_ns");
        v.camera.translation_body = get_array<3>(cam, "camera", "translation_body_m");
        v.camera.rotation_body = get_quat(cam, "camera", "rotation_body_wxyz");
        require(v.camera.width > 0 && v.camera.height > 0, "camera", "width and height must be > 0");
        require(v.camera.fx > 0 && v.camera.fy > 0, "camera", "fx and fy must be > 0");
        require(v.camera.rate_hz > 0, "camera.rate_hz", "must be > 0");
        require(v.camera.row_readout_ns >= 0, "camera.row_readout_ns", "must be ≥ 0");
        require(v.camera.shutter == Shutter::rolling || v.camera.row_readout_ns == 0,
                "camera.row_readout_ns",
                "must be 0 for a global shutter");
        return v;
    });
}

result<VehicleConfig, ConfigError> load_vehicle(const std::filesystem::path& path) {
    return load_file<VehicleConfig>(path, parse_vehicle);
}

std::string emit_vehicle(const VehicleConfig& v) {
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << v.name;
    out << YAML::Key << "mass_kg" << YAML::Value << v.mass;
    out << YAML::Key << "inertia_kgm2" << YAML::Value;
    emit_seq(out, v.inertia);
    out << YAML::Key << "arm_length_m" << YAML::Value << v.arm_length;
    out << YAML::Key << "rotor_count" << YAML::Value << v.rotor_count;
    out << YAML::Key << "rotor" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "thrust_coefficient" << YAML::Value << v.rotor.thrust_coefficient;
    out << YAML::Key << "torque_coefficient" << YAML::Value << v.rotor.torque_coefficient;
    out << YAML::Key << "max_thrust_n" << YAML::Value << v.rotor.max_thrust;
    out << YAML::Key << "time_constant_s" << YAML::Value << v.rotor.time_constant;
    out << YAML::EndMap;
    out << YAML::Key << "drag" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "linear" << YAML::Value;
    emit_seq(out, v.drag.linear);
    out << YAML::Key << "quadratic" << YAML::Value;
    emit_seq(out, v.drag.quadratic);
    out << YAML::EndMap;
    out << YAML::Key << "imu" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rate_hz" << YAML::Value << v.imu.rate_hz;
    out << YAML::Key << "gyro_range_rad_s" << YAML::Value << v.imu.gyro_range;
    out << YAML::Key << "accel_range_m_s2" << YAML::Value << v.imu.accel_range;
    out << YAML::Key << "gyro_noise_density" << YAML::Value << v.imu.gyro_noise_density;
    out << YAML::Key << "accel_noise_density" << YAML::Value << v.imu.accel_noise_density;
    out << YAML::EndMap;
    out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "model" << YAML::Value << name_of(v.camera.model);
    out << YAML::Key << "width" << YAML::Value << v.camera.width;
    out << YAML::Key << "height" << YAML::Value << v.camera.height;
    out << YAML::Key << "format" << YAML::Value << name_of(v.camera.format);
    out << YAML::Key << "fx" << YAML::Value << v.camera.fx;
    out << YAML::Key << "fy" << YAML::Value << v.camera.fy;
    out << YAML::Key << "cx" << YAML::Value << v.camera.cx;
    out << YAML::Key << "cy" << YAML::Value << v.camera.cy;
    out << YAML::Key << "distortion" << YAML::Value;
    emit_seq(out, v.camera.distortion);
    out << YAML::Key << "rate_hz" << YAML::Value << v.camera.rate_hz;
    out << YAML::Key << "shutter" << YAML::Value << name_of(v.camera.shutter);
    out << YAML::Key << "row_readout_ns" << YAML::Value << v.camera.row_readout_ns;
    out << YAML::Key << "translation_body_m" << YAML::Value;
    emit_seq(out, v.camera.translation_body);
    out << YAML::Key << "rotation_body_wxyz" << YAML::Value;
    emit_quat(out, v.camera.rotation_body);
    out << YAML::EndMap;
    out << YAML::EndMap;
    return finish(out);
}

// ── Course ──────────────────────────────────────────────────────────

result<CourseConfig, ConfigError> parse_course(std::string_view yaml, std::string_view origin) {
    return guarded<CourseConfig>(origin, [&]() -> CourseConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"name", "laps", "geofence", "start", "gates", "sequence"});
        CourseConfig c;
        CourseMap<double>& m = c.map;
        c.name = get_string(root, "", "name");
        m.laps = static_cast<std::uint32_t>(get_uint(root, "", "laps", 1000));
        require(m.laps >= 1, "laps", "must be ≥ 1");

        const YAML::Node fence = child(root, "", "geofence");
        only_keys(fence, "geofence", {"min_m", "max_m"});
        m.geofence.min = get_array<3>(fence, "geofence", "min_m");
        m.geofence.max = get_array<3>(fence, "geofence", "max_m");
        for (std::size_t a = 0; a < 3; ++a) {
            require(m.geofence.min[a] < m.geofence.max[a], "geofence", "min_m must be < max_m on every axis");
        }

        const YAML::Node start = child(root, "", "start");
        only_keys(start, "start", {"position_m", "yaw_rad"});
        m.start.position = get_array<3>(start, "start", "position_m");
        m.start.yaw = get_double(start, "start", "yaw_rad");
        require(contains(m.geofence, m.start.position), "start.position_m", "must be inside the geofence");

        const YAML::Node gates = child(root, "", "gates");
        require(gates.IsSequence() && gates.size() >= 1 && gates.size() <= kMaxGates,
                "gates",
                "expected a list of 1 to " + std::to_string(kMaxGates) + " gates");
        m.gate_count = static_cast<std::uint32_t>(gates.size());
        for (std::uint32_t i = 0; i < m.gate_count; ++i) {
            const std::string p = "gates[" + std::to_string(i) + "]";
            const YAML::Node g = gates[i];
            only_keys(g, p, {"id", "center_m", "yaw_rad", "orientation_wxyz", "inner_width_m", "inner_height_m"});
            Gate<double>& gate = m.gates[i];
            gate.id = as_gate_id(child(g, p, "id"), p + ".id");
            for (std::uint32_t j = 0; j < i; ++j) {
                require(m.gates[j].id != gate.id, p + ".id", "duplicate gate id " + std::to_string(gate.id));
            }
            gate.center = get_array<3>(g, p, "center_m");
            require(contains(m.geofence, gate.center), p + ".center_m", "must be inside the geofence");
            const bool has_yaw = static_cast<bool>(g["yaw_rad"]);
            const bool has_q = static_cast<bool>(g["orientation_wxyz"]);
            require(has_yaw != has_q, p, "give exactly one of yaw_rad and orientation_wxyz");
            if (has_yaw) {
                const double half = 0.5 * get_double(g, p, "yaw_rad");
                gate.orientation = Quat<double>{std::cos(half), 0.0, 0.0, std::sin(half)};
            } else {
                gate.orientation = get_quat(g, p, "orientation_wxyz");
            }
            gate.inner_width = get_double(g, p, "inner_width_m");
            gate.inner_height = get_double(g, p, "inner_height_m");
            require(gate.inner_width > 0 && gate.inner_height > 0, p, "inner size must be > 0");
        }

        const YAML::Node seq = child(root, "", "sequence");
        require(seq.IsSequence() && seq.size() >= 1 && seq.size() <= kMaxGateSequence,
                "sequence",
                "expected a list of 1 to " + std::to_string(kMaxGateSequence) + " gate ids");
        m.sequence_length = static_cast<std::uint32_t>(seq.size());
        for (std::uint32_t s = 0; s < m.sequence_length; ++s) {
            const std::string p = "sequence[" + std::to_string(s) + "]";
            const std::int32_t id = as_gate_id(seq[s], p);
            bool found = false;
            for (std::uint32_t i = 0; i < m.gate_count; ++i) {
                found = found || m.gates[i].id == id;
            }
            require(found, p, "no gate with id " + std::to_string(id));
            m.sequence[s] = id;
        }
        return c;
    });
}

result<CourseConfig, ConfigError> load_course(const std::filesystem::path& path) {
    return load_file<CourseConfig>(path, parse_course);
}

std::string emit_course(const CourseConfig& c) {
    const CourseMap<double>& m = c.map;
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << c.name;
    out << YAML::Key << "laps" << YAML::Value << m.laps;
    out << YAML::Key << "geofence" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min_m" << YAML::Value;
    emit_seq(out, m.geofence.min);
    out << YAML::Key << "max_m" << YAML::Value;
    emit_seq(out, m.geofence.max);
    out << YAML::EndMap;
    out << YAML::Key << "start" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position_m" << YAML::Value;
    emit_seq(out, m.start.position);
    out << YAML::Key << "yaw_rad" << YAML::Value << m.start.yaw;
    out << YAML::EndMap;
    out << YAML::Key << "gates" << YAML::Value << YAML::BeginSeq;
    for (std::uint32_t i = 0; i < m.gate_count; ++i) {
        const Gate<double>& g = m.gates[i];
        out << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << g.id;
        out << YAML::Key << "center_m" << YAML::Value;
        emit_seq(out, g.center);
        out << YAML::Key << "orientation_wxyz" << YAML::Value;
        emit_quat(out, g.orientation);
        out << YAML::Key << "inner_width_m" << YAML::Value << g.inner_width;
        out << YAML::Key << "inner_height_m" << YAML::Value << g.inner_height;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "sequence" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (std::uint32_t s = 0; s < m.sequence_length; ++s) {
        out << m.sequence[s];
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    return finish(out);
}

// ── Scenario ────────────────────────────────────────────────────────

result<ScenarioConfig, ConfigError>
parse_scenario(std::string_view yaml, std::string_view origin, const std::filesystem::path& base_dir) {
    return guarded<ScenarioConfig>(origin, [&]() -> ScenarioConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"name", "course", "vehicle", "duration_s", "seed"});
        ScenarioConfig s;
        s.name = get_string(root, "", "name");
        s.course_ref = get_string(root, "", "course");
        s.vehicle_ref = get_string(root, "", "vehicle");
        s.course_path = (base_dir / s.course_ref).lexically_normal();
        s.vehicle_path = (base_dir / s.vehicle_ref).lexically_normal();
        std::error_code ec;
        require(std::filesystem::is_regular_file(s.course_path, ec), "course", "no file " + s.course_path.string());
        require(std::filesystem::is_regular_file(s.vehicle_path, ec), "vehicle", "no file " + s.vehicle_path.string());
        s.duration_s = get_double(root, "", "duration_s");
        require(s.duration_s > 0, "duration_s", "must be > 0");
        s.seed = get_uint(root, "", "seed", std::numeric_limits<std::uint64_t>::max());
        return s;
    });
}

result<ScenarioConfig, ConfigError> load_scenario(const std::filesystem::path& path) {
    const auto base = path.parent_path();
    return load_file<ScenarioConfig>(
        path, [&](std::string_view text, std::string_view origin) { return parse_scenario(text, origin, base); });
}

std::string emit_scenario(const ScenarioConfig& s) {
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << s.name;
    out << YAML::Key << "course" << YAML::Value << s.course_ref;
    out << YAML::Key << "vehicle" << YAML::Value << s.vehicle_ref;
    out << YAML::Key << "duration_s" << YAML::Value << s.duration_s;
    out << YAML::Key << "seed" << YAML::Value << s.seed;
    out << YAML::EndMap;
    return finish(out);
}

// ── Baseline ────────────────────────────────────────────────────────

result<BaselineConfig, ConfigError> parse_baseline(std::string_view yaml, std::string_view origin) {
    return guarded<BaselineConfig>(origin, [&]() -> BaselineConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"name", "composition", "parameters"});
        BaselineConfig b;
        b.name = get_string(root, "", "name");
        require(!b.name.empty() &&
                    b.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-") == std::string::npos,
                "name",
                "must match [a-z0-9-]+");

        const YAML::Node comp = child(root, "", "composition");
        only_keys(comp,
                  "composition",
                  {"controller", "daa_tracker", "estimator", "fc_bridge", "gate_detector", "planner", "safety_filter"});
        for (const auto slot : kSlotNames) {
            const std::string key(slot);
            const std::string impl = get_string(comp, "composition", key);
            require(!impl.empty() &&
                        impl.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_") ==
                            std::string::npos,
                    "composition." + key,
                    "must match [A-Za-z0-9_]+");
            b.composition[key] = impl;
        }

        if (const YAML::Node params = root["parameters"]) {
            require(params.IsMap() || params.IsNull(), "parameters", "expected a mapping");
            if (params.IsMap()) {
                for (const auto& kv : params) {
                    const auto key = kv.first.as<std::string>();
                    require(kv.second.IsScalar(), "parameters." + key, "must be a scalar");
                    b.parameters[key] = kv.second.as<std::string>();
                }
            }
        }
        return b;
    });
}

result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path) {
    return load_file<BaselineConfig>(path, parse_baseline);
}

std::string emit_baseline(const BaselineConfig& b) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << b.name;
    out << YAML::Key << "composition" << YAML::Value << YAML::BeginMap;
    for (const auto& [slot, impl] : b.composition) {
        out << YAML::Key << slot << YAML::Value << impl;
    }
    out << YAML::EndMap;
    out << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
    for (const auto& [key, value] : b.parameters) {
        out << YAML::Key << key << YAML::Value << value;
    }
    out << YAML::EndMap;
    out << YAML::EndMap;
    return finish(out);
}

std::string composition_hash(const BaselineConfig& b) {
    std::string canonical;
    for (const auto slot : kSlotNames) {
        const auto it = b.composition.find(std::string(slot));
        canonical += std::string(slot) + "=" + (it == b.composition.end() ? std::string() : it->second) + "\n";
    }
    return sha256_hex(canonical);
}

std::optional<double> numeric_parameter(const BaselineConfig& b, std::string_view key) {
    const auto it = b.parameters.find(std::string(key));
    if (it == b.parameters.end()) {
        return std::nullopt;
    }
    try {
        std::size_t used = 0;
        const double v = std::stod(it->second, &used);
        if (used != it->second.size() || !std::isfinite(v)) {
            return std::nullopt;
        }
        return v;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace branes::racing::config
