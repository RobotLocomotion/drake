#include "drake/systems/optimization/system_constraint_wrapper.h"

namespace drake {
namespace systems {
SystemConstraintWrapper::SystemConstraintWrapper(
    const System<double>* const system_double,
    const System<AutoDiffXd>* const system_autodiff,
    SystemConstraintIndex index, const Context<double>& context,
    UpdateContextFromX<double> selector_double,
    UpdateContextFromX<AutoDiffXd> selector_autodiff, int x_size)
    : solvers::Constraint(system_double->get_constraint(index).size(), x_size,
                          system_double->get_constraint(index).lower_bound(),
                          system_double->get_constraint(index).upper_bound(),
                          system_double->get_constraint(index).description()),
      system_double_{system_double},
      system_autodiff_{system_autodiff},
      index_{index},
      context_double_{context.Clone()},
      context_autodiff_{system_autodiff_->CreateDefaultContext()},
      selector_double_(selector_double),
      selector_autodiff_(selector_autodiff) {
  context_autodiff_->SetTimeStateAndParametersFrom(*context_double_);
}

void SystemConstraintWrapper::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                     Eigen::VectorXd* y) const {
  selector_double_(*system_double_, x, context_double_.get());
  system_double_->get_constraint(index_).Calc(*context_double_, y);
}

void SystemConstraintWrapper::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                     AutoDiffVecXd* y) const {
  selector_autodiff_(*system_autodiff_, x, context_autodiff_.get());
  system_autodiff_->get_constraint(index_).Calc(*context_autodiff_, y);
}

void SystemConstraintWrapper::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "SystemConstraintWrapper::DoEval do not support symbolic computation.");
}
}  // namespace systems
}  // namespace drake
