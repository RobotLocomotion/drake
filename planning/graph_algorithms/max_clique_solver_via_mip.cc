#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"

#include <memory>
#include <vector>

#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

MaxCliqueSolverViaMip::MaxCliqueSolverViaMip(
    const std::optional<Eigen::VectorXd>& initial_guess,
    const solvers::SolverOptions& solver_options)
    : initial_guess_{initial_guess}, solver_options_{solver_options} {}

VectorX<bool> MaxCliqueSolverViaMip::DoSolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) const {
  const int n = adjacency_matrix.rows();
  DRAKE_THROW_UNLESS(!initial_guess_.has_value() ||
                     initial_guess_.value().rows() == n);

  solvers::MathematicalProgram prog;
  auto x = prog.NewBinaryVariables(adjacency_matrix.cols(), "x");

  // Maximize ∑ᵢ xᵢ
  prog.AddLinearCost(Eigen::VectorXd::Constant(x.rows(), -1), 0, x);

  // Constraint xᵢ + xⱼ ≤ 1 if (i,j) is not in the edge.
  std::vector<Eigen::Triplet<double>> A_constraint_triplets;
  // Reserve the number of non-zero elements in the complement adjacency matrix
  const int num_zero_in_adjacency =
      adjacency_matrix.rows() * adjacency_matrix.rows() -
      adjacency_matrix.nonZeros();
  const int num_zero_on_diagonal =
      adjacency_matrix.rows() -
      adjacency_matrix.diagonal().template cast<int>().sum();
  const int num_expected_constraints =
      num_zero_in_adjacency - num_zero_on_diagonal;

  // Each zero element of the adjacency matrix corresponds to two
  // non-zero entries in the constraint matrix. However, the constraint from
  // adjacency(i,j) = 0 is the same constraint required by adjacency(j,i) = 0,
  // and so we only need to add constraints from the upper triangular part.
  // Therefore, we need exactly num_zero_in_adjacency triplets.
  A_constraint_triplets.reserve(num_expected_constraints);
  int count = 0;
  // TODO(Alexandre.Amice) if performance becomes an issue for large graphs,
  // consider doing this step in parallel.
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (!adjacency_matrix.coeff(i, j)) {
        A_constraint_triplets.emplace_back(count, i, 1);
        A_constraint_triplets.emplace_back(count, j, 1);
        ++count;
      }
    }
  }
  DRAKE_DEMAND(ssize(A_constraint_triplets) == num_expected_constraints);
  SparseMatrix<double> A(count, x.rows());
  A.setFromTriplets(A_constraint_triplets.begin(), A_constraint_triplets.end());
  prog.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()),
                           Eigen::VectorXd::Ones(A.rows()), x);

  solvers::MathematicalProgramResult result;
  std::unique_ptr<solvers::SolverInterface> solver;
  std::optional<solvers::SolverId> solver_id;
  try {
    solver_id = solvers::ChooseBestSolver(prog);
  } catch (const std::exception&) {
    // TODO(Alexandre.Amice) update the error message if other max clique
    // solvers based become available.
    throw std::runtime_error(
        "SolveMaxClique: There is no solver available that can solve the "
        "mixed-integer version of maximum clique. Please check "
        "https://drake.mit.edu/doxygen_cxx/group__solvers.html for more "
        "details about supported mixed integer solvers and how to enable "
        "them.");
  }
  solver = solvers::MakeSolver(solver_id.value());
  solver->Solve(prog, initial_guess_, solver_options_, &result);
  DRAKE_DEMAND(result.is_success());

  // Manually cast the return to a boolean to avoid round off errors from the
  // MIP solver.
  return result.GetSolution(x).unaryExpr([](double elt) {
    return elt >= 0.5;
  });
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
