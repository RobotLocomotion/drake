#pragma once
#include <Eigen/Sparse>

#include "drake/common/drake_throw.h"
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
  MaxCliqueSolverBase() = default;
  virtual ~MaxCliqueSolverBase() {}

  /**
   * Given the adjacency matrix of an undirected graph, find the maximum clique
   * within the graph. A clique is a collection of vertices in a graph such that
   * each pair of vertices is connected by an edge (i.e. a fully connected
   * subgraph). This problem is known to be NP-complete, and so the choice of
   * solvers in @param options determines whether the return of this function is
   * the true maximum clique in the subgraph (which may take very long to
   * compute), or only an approximate solution found via heuristics.
   * @param adjacency_matrix a symmetric (0,1)-matrix encoding the edge
   * relationship.
   * @param options options for solving the max-clique problem.
   * @return A binary vector with the same indexing as the adjacency matrix,
   * with 1 indicating membership in the clique.
   * @throws if the adjacency matrix is not symmetric.
   * @throws based on the preconditions of the solver contained in @param
   * options.
   */
  VectorX<bool> SolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const;

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverBase);

 private:
  virtual VectorX<bool> DoSolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const = 0;
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
