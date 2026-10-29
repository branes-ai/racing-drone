#pragma once
/// @file ground_truth.hpp
/// @brief GroundTruthEstimator: racing's ground-truth estimator (SITL only).
///
/// Baseline B0's estimator (docs/plan/b0/). Racing-only
/// (docs/assessments/framework-boundary.md) only in its frame input: the
/// estimate itself is GroundTruthCore (ground_truth_core.hpp), and this adds
/// the gate observations racing's Estimator slot takes, which it ignores.

#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/msg/perception.hpp>
#include <branes/racing/slots/slots.hpp>

namespace branes::racing {

template <typename T>
class GroundTruthEstimator : public GroundTruthCore<T> {
public:
    void process_frame(const CameraFrame&, const GateObservations<T>&) noexcept {}
};

static_assert(Estimator<GroundTruthEstimator<double>, double>);

}  // namespace branes::racing
