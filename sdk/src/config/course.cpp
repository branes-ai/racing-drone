#include <branes/racing/config/course.hpp>

#include "parse.hpp"

namespace branes::racing::config {

using namespace detail;

namespace {

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

}  // namespace

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

}  // namespace branes::racing::config
