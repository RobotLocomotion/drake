#include "drake/planning/graph_algorithms/max_clique.h"

#include "drake/planning/graph_algorithms/max_stable_set.h"
#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

MaxCliqueSolverViaMIP::MaxCliqueSolverViaMIP(
    const solvers::SolverId& solver_id, const solvers::SolverOptions& options)
    : solver_id_(solver_id), options_(options){};

MaxCliqueOptions::MaxCliqueOptions(const MaxCliqueSolverBase* m_solver)
    : solver{const_cast<MaxCliqueSolverBase*>(std::move(m_solver))} {};

VectorX<bool> MaxCliqueSolverViaMIP::SolveMaxClique(
    SparseMatrix<bool> adjacency_matrix) {
  SparseMatrix<bool> complement_adjacency_matrix = -adjacency_matrix;
  complement_adjacency_matrix.diagonal() =
      Eigen::SparseMatrix::Zero(adjacency_matrix.cols());

  //
  //  DRAKE_DEMAND(adjacency_matrix.isApprox(adjacency_matrix.transpose()));
  //  DRAKE_DEMAND(adjacency_matrix.rows() == adjacency_matrix.cols());
  //  // Ensure that there are no self-loops in the adjacency matrix as
  //  otherwise
  //  // there is no stable set.
  //  DRAKE_DEMAND(!(adjacency_matrix.diagonal().all() == 0));
  //
  //  solvers::MathematicalProgram prog;
  //  auto x = prog.NewBinaryVariables(adjacency_matrix.cols(), "x");
  //  // Maximize ∑ᵢ xᵢ
  //  prog.AddLinearCost(-Eigen::VectorXd::Ones(x.rows()), 0, x);
  //  // Constraint xᵢ + xⱼ ≤ 1
  //  // TODO(AlexandreAmice) Remove toDense()
  //  prog.AddLinearConstraint(adjacency_matrix.cast<double>().toDense(),
  //                           Eigen::VectorXd::Zero(x.rows()),
  //                           Eigen::VectorXd::Ones(x.rows()), x);
  //  solvers::MathematicalProgramResult result;
  //  solvers::MakeSolver(solver_id_)
  //      ->Solve(prog, std::nullopt, options_, &result);
  //
  //  return result.GetSolution(x).cast<bool>();
}
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake