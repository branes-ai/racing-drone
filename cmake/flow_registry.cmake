# cmake/flow_registry.cmake — the operators a racing graph baseline can name
# (anatomy docs/adr/0004, step 3; docs/plan/flow-graph/).
#
# Racing-only (docs/assessments/framework-boundary.md); the registry mechanism
# is anatomy's (cmake/flow.cmake). A baseline's `graph:` block maps each
# composite to an operator. racing_register_operator() records, for that name,
# the header that declares it, its C++ class template and its binding's
# (templated on the scalar type): racing's for the estimator, the gate detector
# and the planner (flow/bindings.hpp), anatomy's for the mission-neutral
# composites (mobility/flow/bindings.hpp). Every slot implementation is an
# operator of its composite, unchanged, so the names are the slot registry's
# (cmake/slot_registry.cmake). racing_register_body() records a body of several
# operators, and racing_register_graph() a research graph (anatomy's
# flow_register_body and flow_register_graph).
include_guard(GLOBAL)
include(${ANATOMY_CMAKE_DIR}/flow.cmake)

function(racing_register_operator name)
    flow_register_operator(racing ${name} ${ARGN})
endfunction()

function(racing_register_body name)
    flow_register_body(racing ${name} ${ARGN})
endfunction()

function(racing_register_graph name)
    flow_register_graph(racing ${name} ${ARGN})
endfunction()

set(_null branes/racing/slots/null.hpp)
set(_racing branes::racing::graph)
set(_neutral branes::racing::flow)
racing_register_operator(NullEstimator    HEADER ${_null} TYPE branes::racing::NullEstimator
                         BINDING ${_racing}::EstimatorBinding)
racing_register_operator(NullGateDetector HEADER ${_null} TYPE branes::racing::NullGateDetector
                         BINDING ${_racing}::GateDetectorBinding)
racing_register_operator(NullPlanner      HEADER ${_null} TYPE branes::racing::NullPlanner
                         BINDING ${_racing}::PlannerBinding)
racing_register_operator(NullController   HEADER ${_null} TYPE branes::racing::NullController
                         BINDING ${_neutral}::ControllerBinding)
racing_register_operator(NullSafetyFilter HEADER ${_null} TYPE branes::racing::NullSafetyFilter
                         BINDING ${_neutral}::SafetyFilterBinding)
racing_register_operator(NullDaaTracker   HEADER ${_null} TYPE branes::racing::NullDaaTracker
                         BINDING ${_neutral}::DaaTrackerBinding)
racing_register_operator(NullFcBridge     HEADER ${_null} TYPE branes::racing::NullFcBridge
                         BINDING ${_neutral}::FcBridgeBinding)

# Test fixtures: open-loop components for the SITL gates (not racing components).
racing_register_operator(ScriptedHopController HEADER branes/anatomy/mobility/slots/scripted.hpp
                         TYPE branes::anatomy::ScriptedHopController BINDING ${_neutral}::ControllerBinding)

# Baseline B0 (docs/plan/b0/).
racing_register_operator(GroundTruthEstimator HEADER branes/racing/slots/ground_truth.hpp
                         TYPE branes::racing::GroundTruthEstimator BINDING ${_racing}::EstimatorBinding)
racing_register_operator(WaypointPlanner HEADER branes/racing/slots/waypoint.hpp
                         TYPE branes::racing::WaypointPlanner BINDING ${_racing}::PlannerBinding)
racing_register_operator(CascadedPid HEADER branes/anatomy/glue/cascaded_pid.hpp
                         TYPE branes::anatomy::glue::CascadedPid BINDING ${_neutral}::ControllerBinding)

# Baselines b0-shaped-pid and b0-shaped-research (anatomy docs/plan/flow-testing,
# T5): a body of two operators for the controller, and a research graph of
# B0's operators and those two. ShapedPid is also the slot registry's name of
# the same operators wired by hand.
racing_register_body(ShapedPid HEADER branes/racing/flow/shaped_pid.hpp TYPE branes::racing::graph::ShapedPidBody)
racing_register_graph(ShapedPidResearch HEADER branes/racing/flow/shaped_pid.hpp
                      TYPE branes::racing::graph::ShapedPidResearch)
