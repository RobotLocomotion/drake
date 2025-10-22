#pragma once
#include <set>
#include <vector>

#include <Eigen/Sparse>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

class MinCliqueCoverSolverBase {
 public:
  virtual ~MinCliqueCoverSolverBase();

  /**
   * Given the adjacency matrix of an undirected graph, finds a (potentially
   * approximate) minimum clique cover of the graph. A clique is a collection of
   * vertices in a graph such that each pair of vertices is connected by an edge
   * (i.e. a fully connected subgraph). A clique cover is a collection of
   * cliques where each vertex in the graph is in at least one of the cliques.
   * This problem is known to be NP-complete, and so the concrete implementation
   * of the solver determines whether the return of this function is the true
   * minimum clique cover of the graph (which may take very long to compute), or
   * only an approximate solution found via heuristics.
   *
   * This method throws if the adjacency matrix is not symmetric and may throw
   * depending on the concrete implementation of the solver.
   *
   * @param adjacency_matrix a symmetric binary matrix encoding the edge
   * relationship.
   * @param partition If true, then every vertex is allowed to be covered
   * exactly once. Otherwise, the same vertex may be covered multiple times.
   * @return A binary vector with the same indexing as the adjacency matrix,
   * with true indicating membership in the clique.
   * @return  The ith entry of the returned vector is a vector containing the
   * indices of the ith clique.
   *
   * Note that this method is intentionally not-const because some solvers may
   * use randomized algorithms.
   */
  [[nodiscard]] std::vector<std::set<int>> SolveMinCliqueCover(
      const Eigen::SparseMatrix<bool>& adjacency_matrix,
      bool partition = false);

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MinCliqueCoverSolverBase);
  MinCliqueCoverSolverBase() = default;

 private:
  virtual std::vector<std::set<int>> DoSolveMinCliqueCover(
      const Eigen::SparseMatrix<bool>& adjacency_matrix,
      bool partition = false) = 0;
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
