# cmake/slot_registry.cmake — racing's slots, and the implementations a racing baseline can name.
#
# Racing-only (docs/assessments/framework-boundary.md); the registry mechanism
# is mission-neutral (anatomy cmake/composition.cmake). A baseline's `composition:`
# block maps each slot to an implementation name. racing_register_impl()
# records, for that name, the slot it fills, the header that declares it, and
# its C++ class template (templated on the scalar type). Platform components
# (cortex, reflex, synapse) register here as they land.
include_guard(GLOBAL)
include(${ANATOMY_CMAKE_DIR}/composition.cmake)

# The seven slots (docs/adr/0001-baseline-composition-framework.md), in the
# order the generated Stack<> takes them.
set(RACING_SLOTS estimator gate_detector planner controller safety_filter daa_tracker fc_bridge)
composition_define_slots(racing ${RACING_SLOTS})

function(racing_register_impl impl)
    composition_register_impl(racing ${impl} ${ARGN})
endfunction()

# Null implementations: stateless stubs for the `null` baseline (Phase 0).
set(_null branes/racing/slots/null.hpp)
racing_register_impl(NullEstimator    SLOT estimator     HEADER ${_null} TYPE branes::racing::NullEstimator)
racing_register_impl(NullGateDetector SLOT gate_detector HEADER ${_null} TYPE branes::racing::NullGateDetector)
racing_register_impl(NullPlanner      SLOT planner       HEADER ${_null} TYPE branes::racing::NullPlanner)
racing_register_impl(NullController   SLOT controller    HEADER ${_null} TYPE branes::racing::NullController)
racing_register_impl(NullSafetyFilter SLOT safety_filter HEADER ${_null} TYPE branes::racing::NullSafetyFilter)
racing_register_impl(NullDaaTracker   SLOT daa_tracker   HEADER ${_null} TYPE branes::racing::NullDaaTracker)
racing_register_impl(NullFcBridge     SLOT fc_bridge     HEADER ${_null} TYPE branes::racing::NullFcBridge)

# Test fixtures: open-loop components for the SITL gates (not racing components).
racing_register_impl(ScriptedHopController SLOT controller HEADER branes/anatomy/slots/scripted.hpp
                     TYPE branes::anatomy::ScriptedHopController)

# Baseline B0 (docs/plan/b0/): the SITL ground-truth estimator, the waypoint
# planner, and the reflex cascade adapter (baselines/glue/).
racing_register_impl(GroundTruthEstimator SLOT estimator HEADER branes/racing/slots/ground_truth.hpp
                     TYPE branes::racing::GroundTruthEstimator)
racing_register_impl(WaypointPlanner SLOT planner HEADER branes/racing/slots/waypoint.hpp
                     TYPE branes::racing::WaypointPlanner)
racing_register_impl(CascadedPid SLOT controller HEADER branes/anatomy/glue/cascaded_pid.hpp
                     TYPE branes::anatomy::glue::CascadedPid)
