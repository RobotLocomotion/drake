#include "drake/planning/graph_algorithms/max_stable_set.h"

#include <utility>
#include <vector>

#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

using Eigen::SparseMatrix;

MaxStableSetSolverViaMIP::MaxStableSetSolverViaMIP(
    const solvers::SolverId& solver_id, const solvers::SolverOptions& options)
    : solver_id_(solver_id), options_(options) {}

MaxStableSetOptions::MaxStableSetOptions(const MaxStableSetSolverBase* m_solver)
    : solver{const_cast<MaxStableSetSolverBase*>(std::move(m_solver))} {};

VectorX<bool> MaxStableSetSolverViaMIP::SolveMaxStableSet(
    const SparseMatrix<bool>& adjacency_matrix) {
  DRAKE_DEMAND(adjacency_matrix.isApprox(adjacency_matrix.transpose()));
  DRAKE_DEMAND(adjacency_matrix.rows() == adjacency_matrix.cols());

  solvers::MathematicalProgram prog;
  auto x = prog.NewBinaryVariables(adjacency_matrix.cols(), "x");

  // Maximize ∑ᵢ xᵢ
  prog.AddLinearCost(-Eigen::VectorXd::Ones(x.rows()), 0, x);
  // Constraint xᵢ + xⱼ ≤ 1
  std::vector<Eigen::Triplet<double>> A_constraint_triplets;
  A_constraint_triplets.reserve(adjacency_matrix.nonZeros());
  int count = 0;
  for (int i = 0; i < adjacency_matrix.outerSize(); ++i) {
    for (Eigen::SparseMatrix<bool>::InnerIterator it(adjacency_matrix, i); it;
         ++it) {
      A_constraint_triplets.emplace_back(count, it.row(), 1);
      A_constraint_triplets.emplace_back(count, it.col(), 1);
      ++count;
    }
  }
  SparseMatrix<double> A(count, x.rows());
  A.setFromTriplets(A_constraint_triplets.begin(), A_constraint_triplets.end());
  prog.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()),
                           Eigen::VectorXd::Ones(A.rows()), x);
  solvers::MathematicalProgramResult result;
  solvers::MakeSolver(solver_id_)->Solve(prog, std::nullopt, options_, &result);
  return result.GetSolution(x).cast<bool>();
}

VectorX<bool> MaxStableSet(const Eigen::SparseMatrix<bool>& adjacency_matrix,
                           const MaxStableSetOptions& options) {
  return options.solver->SolveMaxStableSet(adjacency_matrix);
}
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
