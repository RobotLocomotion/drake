#include "drake/planning/graph_algorithms/min_clique_cover_solver_base.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

MinCliqueCoverSolverBase::~MinCliqueCoverSolverBase() = default;

std::vector<std::set<int>> MinCliqueCoverSolverBase::SolveMinCliqueCover(
    const Eigen::SparseMatrix<bool>& adjacency_matrix, bool partition) {
  DRAKE_THROW_UNLESS(adjacency_matrix.rows() == adjacency_matrix.cols());
  DRAKE_THROW_UNLESS(adjacency_matrix.isApprox(adjacency_matrix.transpose()));
  return DoSolveMinCliqueCover(adjacency_matrix, partition);
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
