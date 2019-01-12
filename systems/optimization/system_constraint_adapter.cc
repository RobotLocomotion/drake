#include "drake/systems/optimization/system_constraint_adapter.h"

namespace drake {
namespace systems {
SystemConstraintAdapter::SystemConstraintAdapter(
    const System<double>* const system)
    : system_double_{system},
      system_autodiff_{system_double_->ToAutoDiffXd()},
      system_symbolic_{system_double_->ToSymbolicMaybe()} {
  DRAKE_DEMAND(system);
}

std::shared_ptr<SystemConstraintWrapper> SystemConstraintAdapter::Create(
    SystemConstraintIndex index, const Context<double>& context,
    UpdateContextFromDecisionVariables<double> selector_double,
    UpdateContextFromDecisionVariables<AutoDiffXd> selector_autodiff,
    int x_size) const {
  return std::make_shared<SystemConstraintWrapper>(
      system_double_, system_autodiff_.get(), index, context, selector_double,
      selector_autodiff, x_size);
}
}  // namespace systems
}  // namespace drake
