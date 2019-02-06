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

struct UpdateSymbolicContextFromDecisionVariablesFunction {
  
};
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
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  constraints.reserve(constraint_val.size());
  // Parse the symbolic expression to constraint.
  // If the symbolic expression have some special structures (for example, being
  // linear), then we can create a
  // LinearConstraint/LinearEqualityConstraint/BoundingBoxConstraint. If the
  // symbolic expression exhibits no structure that can be parsed to a special
  // derived class of solvers::Constraint, then we will return a
  // SystemConstraintWrapper as a generic nonlinear constraint, which contains
  // *all* the rows in @p constraint_val, that cannot be parsed to special
  // derived class of solvers::Constraint.

  // generic_constraint_rows stores the rows in @p constraint_val, whose
  // symbolic expression will be regarded as generic constraint.
  std::vector<int> generic_constraint_rows;
  generic_constraint_rows.reserve(constraint_val.size());
  for (int i = 0; i < constraint_val.size(); ++i) {
    std::unique_ptr<solvers::Binding<solvers::Constraint>> linear_constraint =
        solvers::internal::MaybeParseLinearConstraint(
            constraint_val(i), system_constraint.bounds().lower()(i),
            system_constraint.bounds().upper()(i));
    if (linear_constraint) {
      constraints.push_back(*linear_constraint);
    } else {
      // TODO(hongkai.dai): parse second order cone constraint.
      generic_constraint_rows.push_back(i);
    }
  }
  if (!generic_constraint_rows.empty()) {
  }
  return constraints;
}

}  // namespace systems
}  // namespace drake
