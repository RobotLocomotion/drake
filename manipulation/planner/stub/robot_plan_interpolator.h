#pragma once

#warning The include path drake/manipulation/planner/robot_plan_interpolator.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/manipulation/util/robot_plan_interpolator.h.

#include "drake/manipulation/util/robot_plan_interpolator.h"

namespace drake {
namespace manipulation {
namespace planner {

using RobotPlanInterpolator DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use  drake::manipulation::util::RobotPlanInterpolator instead.") =
    drake::manipulation::util::RobotPlanInterpolator;

using InterpolatorType DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use  drake::manipulation::util::InterpolatorType instead.") =
    drake::manipulation::util::InterpolatorType;

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
