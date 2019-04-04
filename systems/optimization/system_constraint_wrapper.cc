#include "drake/systems/optimization/system_constraint_wrapper.h"

#include <utility>

namespace drake {
namespace systems {
SystemConstraintWrapper::SystemConstraintWrapper(
    const System<double>* const system_double,
    const System<AutoDiffXd>* const system_autodiff,
    SystemConstraintIndex index, const Context<double>& context,
    UpdateContextFromDecisionVariablesFunction<double> updater_double,
    UpdateContextFromDecisionVariablesFunction<AutoDiffXd> updater_autodiff,
    int x_size)
    : solvers::Constraint(system_double->get_constraint(index).size(), x_size,
                          system_double->get_constraint(index).lower_bound(),
                          system_double->get_constraint(index).upper_bound(),
                          system_double->get_constraint(index).description()),
      system_double_{system_double},
      owned_system_autodiff_{system_autodiff == nullptr
                                 ? system_double_->ToAutoDiffXd()
                                 : nullptr},
      system_autodiff_{system_autodiff == nullptr ? owned_system_autodiff_.get()
                                                  : system_autodiff},
      index_{index},
      context_double_{context.Clone()},
      context_autodiff_{system_autodiff_->CreateDefaultContext()},
      updater_double_(std::move(updater_double)),
      updater_autodiff_(std::move(updater_autodiff)) {
  context_autodiff_->SetTimeStateAndParametersFrom(*context_double_);
  for (int i = 0; i < context_double_->num_input_ports(); ++i) {
    if (context_double_->MaybeGetFixedInputPortValue(i) != nullptr) {
      // TODO(hongkai.dai): support system with fixed values in the input ports,
      // when we can copy the fixed input port value from Context<double> to
      // Context<AutoDiffXd>.
      throw std::runtime_error(
          "SystemConstraintWrapper doesn't support system with fixed input "
          "ports yet.");
    }
  }
}

const System<AutoDiffXd>& SystemConstraintWrapper::system_autodiff() const {
  return *system_autodiff_;
}

void SystemConstraintWrapper::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                     Eigen::VectorXd* y) const {
  updater_double_(*system_double_, x, context_double_.get());
  system_double_->get_constraint(index_).Calc(*context_double_, y);
}

void SystemConstraintWrapper::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                     AutoDiffVecXd* y) const {
  updater_autodiff_(system_autodiff(), x, context_autodiff_.get());
  system_autodiff().get_constraint(index_).Calc(*context_autodiff_, y);
}

void SystemConstraintWrapper::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "SystemConstraintWrapper::DoEval does not support symbolic computation.");
}
}  // namespace systems
}  // namespace drake
