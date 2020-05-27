#include "drake/solvers/get_infeasible_constraints.h"

#include <memory>

namespace drake {
namespace solvers {

std::vector<std::string> GetInfeasibleConstraints(
    const MathematicalProgram& prog, const MathematicalProgramResult& result,
    std::optional<double> tolerance) {
  std::vector<std::string> descriptions;

  if (!tolerance) {
    // TODO(russt): Extract the constraint tolerance from the solver.  This
    // value was used successfully for some time in MATLAB Drake, so I've
    // ported it as the default here.
    tolerance = 1e-4;
  }

  for (const auto& binding : prog.GetAllConstraints()) {
    const Eigen::VectorXd val = result.EvalBinding(binding);
    const std::shared_ptr<Constraint>& constraint = binding.evaluator();
    std::string d = constraint->get_description();
    if (d.empty()) {
      d = NiceTypeName::Get(*constraint);
    }
    for (int i = 0; i < val.rows(); i++) {
      if (val[i] < constraint->lower_bound()[i] - *tolerance ||
          val[i] > constraint->upper_bound()[i] + *tolerance) {
        descriptions.push_back(d + "[" + std::to_string(i) + "]: " +
                               std::to_string(constraint->lower_bound()[i]) +
                               " <= " + std::to_string(val[i]) + " <= " +
                               std::to_string(constraint->upper_bound()[i]));
      }
    }
  }
  return descriptions;
}

std::vector<Binding<Constraint>> GetInfeasibleConstraintBindings(
    const MathematicalProgram& prog, const MathematicalProgramResult& result,
    std::optional<double> tolerance) {
  std::vector<Binding<Constraint>> infeasible_bindings;

  if (!tolerance) {
    // TODO(russt): Extract the constraint tolerance from the solver.  This
    // value was used successfully for some time in MATLAB Drake, so I've
    // ported it as the default here.
    tolerance = 1e-4;
  }

  for (const auto& binding : prog.GetAllConstraints()) {
    const Eigen::VectorXd val = result.EvalBinding(binding);
    for (int i = 0; i < binding.evaluator()->num_constraints(); ++i) {
      if (val(i) > binding.evaluator()->upper_bound()(i) + *tolerance ||
          val(i) < binding.evaluator()->lower_bound()(i) - *tolerance) {
        infeasible_bindings.push_back(binding);
        continue;
      }
    }
  }
  return infeasible_bindings;
}
}  // namespace solvers
}  // namespace drake
