#include "drake/planning/graph_algorithms/max_clique.h"

#include <utility>

#include "drake/planning/graph_algorithms/graph_algorithms_internal.h"
#include "drake/planning/graph_algorithms/max_stable_set.h"
#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

MaxCliqueSolverViaMIP::MaxCliqueSolverViaMIP(
    const solvers::SolverId& solver_id, const solvers::SolverOptions& options)
    : solver_id_(solver_id), options_(options) {}

MaxCliqueOptions::MaxCliqueOptions(const MaxCliqueSolverBase* m_solver)
    : solver{const_cast<MaxCliqueSolverBase*>(std::move(m_solver))} {};

VectorX<bool> MaxCliqueSolverViaMIP::SolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) {
  const SparseMatrix<bool> complement_adjacency_matrix =
      internal::ComplementaryAdjacencyMatrix(adjacency_matrix);
  MaxStableSetSolverViaMIP stable_set_solver(solver_id_, options_);
  MaxStableSetOptions stable_set_options(&stable_set_solver);
  return MaxStableSet(complement_adjacency_matrix, stable_set_options);
}

VectorX<bool> MaxClique(const Eigen::SparseMatrix<bool>& adjacency_matrix,
                        const MaxCliqueOptions& options) {
  return options.solver->SolveMaxClique(adjacency_matrix);
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
