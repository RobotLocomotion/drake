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

optional<std::vector<solvers::Binding<solvers::Constraint>>>
SystemConstraintAdapter::MaybeCreateConstraintSymbolically(
    SystemConstraintIndex index,
    const Context<symbolic::Expression>& context) const {
  if (!system_symbolic_) {
    return {};
  }
  const SystemConstraint<symbolic::Expression>& system_constraint =
      system_symbolic_->get_constraint(index);
  VectorX<symbolic::Expression> constraint_val(system_constraint.size());
  // Evaluate the constraint as symbolic expressions.
  system_constraint.Calc(context, &constraint_val);
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  constraints.reserve(constraint_val.size());
  // Parse the symbolic expression to constraint.
  // If the symbolic expression have some special structures (for example, being
  // linear), then we can create a
  // LinearConstraint/LinearEqualityConstraint/BoundingBoxConstraint.

  bool is_success = true;
  for (int i = 0; i < constraint_val.size(); ++i) {
    std::unique_ptr<solvers::Binding<solvers::Constraint>> linear_constraint =
        solvers::internal::MaybeParseLinearConstraint(
            constraint_val(i), system_constraint.bounds().lower()(i),
            system_constraint.bounds().upper()(i));
    if (linear_constraint) {
      constraints.push_back(*linear_constraint);
    } else {
      // TODO(hongkai.dai): parse second order cone constraint.
      is_success = false;
      break;
    }
  }
  if (!is_success) {
    return {};
  }
  return constraints;
}

const System<symbolic::Expression>& SystemConstraintAdapter::system_symbolic()
    const {
  if (system_symbolic_) {
    return *system_symbolic_;
  } else {
    throw std::runtime_error(
        "SystemConstraintAdapter: the system cannot be instantiated with"
        "symbolic::Expression.");
  }
}

}  // namespace systems
}  // namespace drake
