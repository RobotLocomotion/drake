#pragma once
#include <memory>

#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
/**
 * The problem of finding the maximum clique in a graph is known to be
 * NP-complete. This base class provides a unified interface for various
 * implementations of a solver for this problem which may be solved rigorously
 * or via heuristics.
 */
class MaxCliqueSolverBase {
 public:
  virtual ~MaxCliqueSolverBase();

  /**
   * Given the adjacency matrix of an undirected graph, find the maximum clique
   * within the graph. A clique is a collection of vertices in a graph such that
   * each pair of vertices is connected by an edge (i.e. a fully connected
   * subgraph). This problem is known to be NP-complete, and so the concrete
   * implementation of the solver determines whether the return of this function
   * is the true maximum clique in the graph (which may take very long to
   * compute), or only an approximate solution found via heuristics.
   *
   * This method throws if the adjacency matrix is not symmetric and may throw
   * depending on the concrete implementation of the solver.
   *
   * @param adjacency_matrix a symmetric binary matrix encoding the edge
   * relationship.
   * @return A binary vector with the same indexing as the adjacency matrix,
   * with true indicating membership in the clique.
   */
  [[nodiscard]] VectorX<bool> SolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const;

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverBase);
  MaxCliqueSolverBase() = default;

 private:
  virtual VectorX<bool> DoSolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const = 0;
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
