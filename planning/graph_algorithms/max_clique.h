#pragma once

#include <Eigen/Sparse>

#include "drake/common/ssize.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using Eigen::SparseMatrix;

/**
 * The problem of finding the maximum clique in a graph is known to a
 * NP-complete. This base class provides a unified interface for various
 * implementations of a solver for this problem which may be solved rigorously,
 * or via heuristics.
 */
class MaxCliqueSolverBase {
 public:
  MaxCliqueSolverBase() {}

  virtual VectorX<bool> SolveMaxClique(SparseMatrix<bool> adjacency_matrix) = 0;

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverBase);
};

/**
 * Solves the maximum clique problem to global optimality by solving a
 * mixed-integer program.
 *
 * Note: This solver leverage convex optimization solvers (e.g. Gurobi and/or
 * Mosek). We recommend enabling those solvers if possible
 * (https://drake.mit.edu/bazel.html#proprietary_solvers).
 */
class MaxCliqueSolverViaMIP final : public MaxCliqueSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverViaMIP);
  MaxCliqueSolverViaMIP(
      const solvers::SolverId& solver_id = solvers::GurobiSolver::id());

  VectorX<bool> SolveMaxClique(SparseMatrix<bool> adjacency_matrix);

 private:
  solvers::SolverId solver_id_;
};

struct MaxCliqueOptions {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueOptions)
  MaxCliqueOptions() = default;
  ~MaxCliqueOptions() = default;
  MaxCliqueSolverBase solver = MaxCliqueSolverViaMIP();
};

/**
 * Given the adjacency matrix of an undirected graph, find the maximum clique
 * within the graph. A clique is a collection of vertices in a graph such that
 * each pair of vertices is connected by an edge (i.e. a fully connected
 * subgraph). This problem is known to be NP-complete, and so the choice of
 * solvers in options determines whether the return of this function is the true
 * maximum clique in the subgraph (which may take very long to compute), or just
 * some large subgraph.
 * @param adjacency_matrix a symmetric (0,1)-matrix encoding the edge
 * relationship
 * @param options options for solving the max-clique problem.
 * @return
 */
VectorX<bool> MaxClique(SparseMatrix<bool> adjacency_matrix,
                        MaxCliqueOptions& options){};

}  // namespace planning
}  // namespace drake
