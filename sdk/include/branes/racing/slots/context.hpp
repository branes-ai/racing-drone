#pragma once
/// @file context.hpp
/// @brief SlotContext: racing's slot context, over the course.
///
/// Racing-only (docs/assessments/framework-boundary.md). The machinery
/// (BasicSlotContext, make_slot, ContextBuilt, TruthConsumer, parameter_in) is
/// mission-neutral, in slot_context.hpp; racing's world is the course.

#include <branes/racing/msg/course.hpp>
#include <branes/racing/slots/slot_context.hpp>

namespace branes::racing {

static_assert(MissionWorld<CourseMap<double>>, "the course is racing's world: it carries the start pose");

/// Racing's slot context: the vehicle, the baseline, and the course (`world`).
using SlotContext = BasicSlotContext<CourseMap<double>>;

}  // namespace branes::racing
