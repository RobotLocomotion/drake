#pragma once

#warning The include path drake/systems/trajectory_optimization/direct_collocation.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/direct_collocation.h.

#include "drake/common/drake_deprecated.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
// Deprecated includes from original include file.
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using DirectCollocation DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use drake::planning::trajectory_optimization::DirectCollocation "
    "instead.") = drake::planning::trajectory_optimization::DirectCollocation;

using DirectCollocationConstraint DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use "
    "drake::planning::trajectory_optimization::DirectCollocationConstraint "
    "instead.") =
    drake::planning::trajectory_optimization::DirectCollocationConstraint;

// We can't apply the deprecated attribute to this function alias (it's not
// *truly* an alias). We'll rely on the #warning at the top as sufficient.
// The odds that they are importing the right header but exercising the function
// from the wrong namespace is small.
using planning::trajectory_optimization::AddDirectCollocationConstraint;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
