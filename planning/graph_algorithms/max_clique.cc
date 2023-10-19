#include "drake/planning/graph_algorithms/max_clique.h"

#include <optional>
#include <utility>
#include <vector>

#include "drake/common/ssize.h"
#include "drake/planning/graph_algorithms/graph_algorithms_internal.h"
#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

MaxCliqueSolverViaMIP::MaxCliqueSolverViaMIP(
    const std::optional<solvers::SolverId> solver_id,
    const solvers::SolverOptions& options)
    : solver_id_(solver_id), options_(options) {}

MaxCliqueOptions::MaxCliqueOptions(const MaxCliqueSolverBase* m_solver)
    : solver{const_cast<MaxCliqueSolverBase*>(std::move(m_solver))} {};

VectorX<bool> MaxCliqueSolverViaMIP::SolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) {
  DRAKE_DEMAND(adjacency_matrix.isApprox(adjacency_matrix.transpose()));
  DRAKE_DEMAND(adjacency_matrix.rows() == adjacency_matrix.cols());
  const int n = adjacency_matrix.rows();

  solvers::MathematicalProgram prog;
  auto x = prog.NewBinaryVariables(adjacency_matrix.cols(), "x");

  // Maximize ∑ᵢ xᵢ
  prog.AddLinearCost(Eigen::VectorXd::Constant(x.rows(), -1), 0, x);

  // Constraint xᵢ + xⱼ ≤ 1 if (i,j) is not in the edge.
  std::vector<Eigen::Triplet<double>> A_constraint_triplets;
  // Reserve the number of non-zero elements in the complement adjacency matrix
  const int num_zero_in_adjacency =
      adjacency_matrix.rows() * adjacency_matrix.rows() -
      adjacency_matrix.rows() - adjacency_matrix.nonZeros();
  // Each zero element of the adjacency matrix corresponds to two
  // non-zero entries in the constraint matrix. However, the constraint from
  // adjacency(i,j) = 0 is the same constraint required by adjacency(j,i) = 0,
  // and so we only need to add constraints from the upper triangular part.
  // Therefore, we need exactly num_zero_in_adjacency triplets.
  A_constraint_triplets.reserve(num_zero_in_adjacency);
  int count = 0;
  // TODO(Alexandre.Amice) if performance becomes an issue for large graphs,
  // consider doing step in parallel.
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (!adjacency_matrix.coeff(i, j)) {
        A_constraint_triplets.emplace_back(count, i, 1);
        A_constraint_triplets.emplace_back(count, j, 1);
        ++count;
      }
    }
  }
  DRAKE_DEMAND(ssize(A_constraint_triplets) == num_zero_in_adjacency);
  SparseMatrix<double> A(count, x.rows());
  A.setFromTriplets(A_constraint_triplets.begin(), A_constraint_triplets.end());
  prog.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()),
                           Eigen::VectorXd::Ones(A.rows()), x);

  solvers::MathematicalProgramResult result;
  solvers::MakeSolver(solver_id_.value_or(solvers::ChooseBestSolver(prog)))
      ->Solve(prog, std::nullopt, options_, &result);

  return result.GetSolution(x).cast<bool>();
}

VectorX<bool> MaxClique(const Eigen::SparseMatrix<bool>& adjacency_matrix,
                        const MaxCliqueOptions& options) {
  return options.solver->SolveMaxClique(adjacency_matrix);
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
