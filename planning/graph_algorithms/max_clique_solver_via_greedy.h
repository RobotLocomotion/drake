#pragma once
#include <memory>

#include <Eigen/Sparse>

#include "drake/planning/graph_algorithms/max_clique_solver_base.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

/**
 * Approximately solves the maximum clique problem via a greedy heuristic.
 * Vertices are greedily added to the clique based on their degree of
 * connectivity. The algorithm initializes the clique with an empty set and
 * makes every vertex a candidate, then the degree of every vertex is computed
 * and the candidate vertex with the highest degree is added to the clique.
 * Afterwards, new candidate list is updated and the previous two steps are
 * repeated until no candidates are left.
 */
class MaxCliqueSolverViaGreedy final : public MaxCliqueSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverViaGreedy);
  MaxCliqueSolverViaGreedy() = default;

 private:
  VectorX<bool> DoSolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const final;
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
