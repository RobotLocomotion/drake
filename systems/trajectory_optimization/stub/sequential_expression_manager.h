#pragma once

#warning The include path drake/systems/trajectory_optimization/sequential_expression_manager.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/sequential_expression_manager.h.

#include "drake/common/drake_deprecated.h"
#include "drake/planning/trajectory_optimization/sequential_expression_manager.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace internal {

using SequentialExpressionManager DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use "
    "drake::planning::trajectory_optimization::SequentialExpressionManager "
    "instead.") = drake::planning::trajectory_optimization::internal::
    SequentialExpressionManager;

}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
