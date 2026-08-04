#pragma once

#include <memory>
#include <set>
#include <vector>

#include <Eigen/Sparse>

#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/graph_algorithms/min_clique_cover_solver_base.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

/**
 * Approximately solves the min clique cover problem via a greedy heuristic.
 *
 * At each step, the largest clique found by the provided max clique solver is
 * added to the clique cover. These vertices are then removed from graph and max
 * clique is solved again. This loop continues until a clique is found which is
 * smaller than the min clique size.
 *
 * @note if min clique size > 1, then this class will not strictly compute a
 * clique cover since not all vertices will be covered.
 */
class MinCliqueCoverSolverViaGreedy final : public MinCliqueCoverSolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinCliqueCoverSolverViaGreedy);

  /** Constructs using the given max clique solver. */
  explicit MinCliqueCoverSolverViaGreedy(
      std::shared_ptr<MaxCliqueSolverBase> max_clique_solver,
      int min_clique_size = 1);

  /**
   * Set the minimum clique size. Throws if this is less than 1.
   * @param min_clique_size
   */
  void set_min_clique_size(const int min_clique_size) {
    DRAKE_THROW_UNLESS(min_clique_size > 0);
    min_clique_size_ = min_clique_size;
  }

  int get_min_clique_size() const { return min_clique_size_; }

 private:
  std::vector<std::set<int>> DoSolveMinCliqueCover(
      const Eigen::SparseMatrix<bool>& adjacency_matrix,
      bool partition = false) final;

  std::shared_ptr<MaxCliqueSolverBase> max_clique_solver_;

  int min_clique_size_{1};
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
