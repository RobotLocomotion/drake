#pragma once

#warning The include path drake/systems/trajectory_optimization/direct_collocation.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/direct_collocation.h.

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

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
