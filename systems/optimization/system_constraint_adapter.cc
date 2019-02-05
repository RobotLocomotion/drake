#include "drake/systems/optimization/system_constraint_adapter.h"

#include "drake/solvers/create_constraint.h"

namespace drake {
namespace systems {
SystemConstraintAdapter::SystemConstraintAdapter(
    const System<double>* const system)
    : system_double_{system},
      system_autodiff_{system_double_->ToAutoDiffXd()},
      // TODO(hongkai.dai): use system_symbolic_ to parse the constraint in the
      // symbolic form.
      system_symbolic_{system_double_->ToSymbolicMaybe()} {
  DRAKE_DEMAND(system);
}

std::vector<solvers::Binding<solvers::Constraint>>
SystemConstraintAdapter::CreateConstraintSymbolically(
    SystemConstraintIndex index,
    const Context<symbolic::Expression>& context) const {
  if (!system_symbolic_) {
    throw std::runtime_error(
        "SystemConstraintAdapter: cannot create the constraint symbolically, "
        "since the system is not instantiated with symbolic expression.");
  }
  const SystemConstraint<symbolic::Expression>& system_constraint =
      system_symbolic_->get_constraint(index);
  VectorX<symbolic::Expression> constraint_val(system_constraint.size());
  // Evaluate the constraint as symbolic expressions.
  system_constraint.Calc(context, &constraint_val);
  std::vector<solvers::Binding<solvers::Constraint>> symbolic_constraints;
  symbolic_constraints.reserve(constraint_val.size());
  // Parse the symbolic expression to constraint.
  for (int i = 0; i < constraint_val.size(); ++i) {
    solvers::internal::ParseConstraint(constraint_val(i),
                                       system_constraint.bounds().lower()(i),
                                       system_constraint.bounds().upper()(i));
  }
  return symbolic_constraints;
}

}  // namespace systems
}  // namespace drake
