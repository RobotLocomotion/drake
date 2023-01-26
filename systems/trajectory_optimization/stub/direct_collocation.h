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

DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use "
    "drake::planning::trajectory_optimization::AddDirectCollocationConstraint "
    "instead.")
solvers::Binding<solvers::Constraint> AddDirectCollocationConstraint(
    std::shared_ptr<DirectCollocationConstraint> constraint,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& input,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_input,
    solvers::MathematicalProgram* prog) {
  return planning::trajectory_optimization::AddDirectCollocationConstraint(
      std::move(constraint), timestep, state, next_state, input, next_input,
      prog);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
