#include "drake/planning/graph_algorithms/max_clique_solver_base.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

MaxCliqueSolverBase::~MaxCliqueSolverBase() = default;

VectorX<bool> MaxCliqueSolverBase::SolveMaxClique(
    const Eigen::SparseMatrix<bool>& adjacency_matrix) const {
  DRAKE_THROW_UNLESS(adjacency_matrix.rows() == adjacency_matrix.cols());
  DRAKE_THROW_UNLESS(adjacency_matrix.isApprox(adjacency_matrix.transpose()));
  return DoSolveMaxClique(adjacency_matrix);
}

// Deprecated 2025-05-01.
std::unique_ptr<MaxCliqueSolverBase> MaxCliqueSolverBase::Clone() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return DoClone();
#pragma GCC diagnostic pop
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
