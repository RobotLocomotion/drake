#pragma once

#warning The include path drake/systems/trajectory_optimization/integration_constraint.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/integration_constraint.h.

#include "drake/common/drake_deprecated.h"
#include "drake/planning/trajectory_optimization/integration_constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using MidPointIntegrationConstraint DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use "
    "drake::planning::trajectory_optimization::MidPointIntegrationConstraint "
    "instead.") =
    drake::planning::trajectory_optimization::MidPointIntegrationConstraint;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
