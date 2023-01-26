#pragma once

#warning The include path drake/systems/trajectory_optimization/multiple_shooting.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/multiple_shooting.h.

#include "drake/planning/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using MultipleShooting DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use drake::planning::trajectory_optimization::MultipleShooting "
    "instead.") = drake::planning::trajectory_optimization::MultipleShooting;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
