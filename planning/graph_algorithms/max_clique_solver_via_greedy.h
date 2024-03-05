#pragma once

#include <optional>

#include <Eigen/Sparse>

#include "drake/planning/graph_algorithms/max_clique_solver_base.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

/**
 * Produces approximate solution to the maximum clique problem via a greedy heuristic.
 * TODO(wernerpe): complete docs
 * @throws std::exception if the initial guess has the wrong size for the
 * provided adjacency matrix.
 */
class MaxCliqueSolverViaGreedy final : public MaxCliqueSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverViaGreedy);
  MaxCliqueSolverViaGreedy();

 private:
  VectorX<bool> DoSolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const final;

};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
