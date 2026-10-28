// The cast of racing's gate observations to every type of the precision sweep
// is the scalar conversion of each field (racing-drone docs/plan/precision-sweep/).
// The framework's casts are tested in anatomy.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_sweep.hpp>
#include <branes/racing/msg/perception.hpp>

#include "precision_fixtures.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

using namespace branes::racing;
using precision_fixtures::neighbors;

namespace {

template <typename T>
inline constexpr bool is_posit = std::is_same_v<T, sim::posit32> || std::is_same_v<T, sim::posit16>;

/// A value in [-50, 50) from the stream, for message fields.
double draw(SplitMix64& rng) {
    return rng.symmetric(50.0);
}

// Each message's scalar fields, in a fixed order, for filling and comparing.
template <class A, class F>
void each_of(A& a, F& f) {
    for (auto& x : a) {
        f(x);
    }
}
template <class Q, class F>
void each_of_quat(Q& q, F& f) {
    f(q.w);
    f(q.x);
    f(q.y);
    f(q.z);
}
template <class M, class F>
void each_observations(M& g, F&& f) {
    for (auto& o : g.gates) {
        for (auto& c : o.corners) {
            each_of(c, f);
        }
        each_of(o.covariance, f);
        f(o.quality);
    }
}

/// Fill every scalar field of a double message with a distinct value.
template <class M, class Each>
M filled(Each each) {
    M m{};
    SplitMix64 rng(1700);
    each(m, [&](double& x) { x = draw(rng); });
    return m;
}

/// The scalar fields of a message as doubles.
template <class M, class Each>
std::vector<double> fields(const M& m, Each each) {
    std::vector<double> out;
    each(m, [&](const auto& x) { out.push_back(double(x)); });
    return out;
}

/// Pre: the message's scalar fields. Post: each field of cast<T>(m) is T(field).
template <typename T, class M, class Each>
void check_fields(const M& m, Each each) {
    const auto c = cast<T>(m);
    const std::vector<double> in = fields(m, each);
    const std::vector<double> out = fields(c, each);
    REQUIRE(in.size() == out.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        INFO("field " << i << " = " << in[i]);
        CHECK(out[i] == double(T(in[i])));
    }
    CHECK(cast<double>(cast<double>(m)) == m);  // identity for double
}

}  // namespace

TEMPLATE_LIST_TEST_CASE("cast: the gate observations cast is the scalar conversion of each field",
                        "[precision][cast]",
                        sim::SweepTypes) {
    using T = TestType;
    const auto observations = [](auto& m, auto&& f) { each_observations(m, f); };

    auto g = filled<GateObservations<double>>(observations);
    g.count = 2;
    for (std::uint32_t i = 0; i < kMaxVisibleGates; ++i) {
        g.gates[i].gate_id = std::int32_t(i) + 7;
    }
    check_fields<T>(g, observations);
    CHECK(cast<T>(g).count == g.count);
    for (std::uint32_t i = 0; i < kMaxVisibleGates; ++i) {
        CHECK(cast<T>(g).gates[i].gate_id == g.gates[i].gate_id);
    }
}
