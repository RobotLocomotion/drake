#pragma once

#warning The include path drake/manipulation/planner/constraint_relaxing_ik.h is deprecated and will be removed on or after 2023-06-01; instead, use drake/multibody/inverse_kinematics/constraint_relaxing_ik.h.

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/inverse_kinematics/constraint_relaxing_ik.h"

namespace drake {
namespace manipulation {
namespace planner {

using ConstraintRelaxingIk DRAKE_DEPRECATED(
    "2023-06-01",
    "Please use drake::multibody::ConstraintRelaxingIk instead.") =
    drake::multibody::ConstraintRelaxingIk;

}
}  // namespace manipulation
}  // namespace drake
