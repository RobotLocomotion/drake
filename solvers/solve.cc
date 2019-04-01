#include "drake/solvers/solve.h"

#include <memory>

#include "drake/common/nice_type_name.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
MathematicalProgramResult Solve(const MathematicalProgram& prog,
                                const optional<Eigen::VectorXd>& initial_guess,
                                const optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  std::unique_ptr<SolverInterface> solver = MakeSolver(solver_id);
  MathematicalProgramResult result{};
  solver->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
  const Eigen::VectorXd initial_guess_xd = initial_guess;
  return Solve(prog, initial_guess_xd, {});
}

MathematicalProgramResult Solve(const MathematicalProgram& prog) {
  return Solve(prog, {}, {});
}

std::vector<std::string> GetInfeasibleConstraints(
    const MathematicalProgram& prog, const MathematicalProgramResult&
    result, optional<double> tolerance) {
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
        descriptions.push_back(
            d + "[" + std::to_string(i) +
            "]: " + std::to_string(constraint->lower_bound()[i]) +
            " <= " + std::to_string(val[i]) +
            " <= " + std::to_string(constraint->upper_bound()[i]));
      }
    }
  }
  return descriptions;
}
}  // namespace solvers
}  // namespace drake
