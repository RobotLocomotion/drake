#include "drake/planning/graph_algorithms/max_stable_set.h"
#include "drake/solvers/choose_best_solver.h"


namespace drake {
namespace planning {

MaxStableSetSolverViaMIP::MaxStableSetSolverViaMIP(
    const solvers::SolverId& solver_id, const solvers::SolverOptions& options)
    : solver_id_(solver_id), options_(options){};

MaxStableSetOptions::MaxStableSetOptions(const MaxStableSetSolverBase* m_solver)
    : solver{const_cast<MaxStableSetSolverBase*>(std::move(m_solver))} {};

VectorX<bool> MaxStableSetSolverViaMIP::SolveMaxStableSet(
    SparseMatrix<bool> adjacency_matrix) {
  DRAKE_DEMAND(adjacency_matrix.isApprox(adjacency_matrix.transpose()));
  DRAKE_DEMAND(adjacency_matrix.rows() == adjacency_matrix.cols());
  // Ensure that there are no self-loops in the adjacency matrix as otherwise
  // there is no stable set.
  DRAKE_DEMAND(!(adjacency_matrix.diagonal().all() == 0));

  solvers::MathematicalProgram prog;
  auto x = prog.NewBinaryVariables(adjacency_matrix.cols(), "x");
  // Maximize ∑ᵢ xᵢ
  prog.AddLinearCost(-Eigen::VectorXd::Ones(x.rows()), 0, x);
  // Constraint xᵢ + xⱼ ≤ 1
  // TODO(AlexandreAmice) Remove toDense()
  prog.AddLinearConstraint(adjacency_matrix.cast<double>().toDense(),
                           Eigen::VectorXd::Zero(x.rows()),
                           Eigen::VectorXd::Ones(x.rows()), x);
  solvers::MathematicalProgramResult result;
  solvers::MakeSolver(solver_id_)
      ->Solve(prog, std::nullopt, options_, &result);

  return result.GetSolution(x).cast<bool>();
}

}  // namespace planning
}  // namespace drake