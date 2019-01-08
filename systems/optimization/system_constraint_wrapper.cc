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
      context_double_{context.Clone()},
      context_autodiff_{system_autodiff_->CreateDefaultContext()},
      selector_double_(selector_double),
      selector_autodiff_(selector_autodiff) {}
}  // namespace systems
}  // namespace drake
