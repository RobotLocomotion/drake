#pragma once

#include <memory>

#include <Eigen/Sparse>

#include "drake/common/ssize.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

/**
 * The problem of finding the maximum clique in a graph is known to a
 * NP-complete. This base class provides a unified interface for various
 * implementations of a solver for this problem which may be solved rigorously,
 * or via heuristics.
 */
class MaxStableSetSolverBase {
 public:
  MaxStableSetSolverBase() {}

  virtual ~MaxStableSetSolverBase() {}

  virtual VectorX<bool> SolveMaxStableSet(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) = 0;

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxStableSetSolverBase);
};

/**
 * Solves the maximum stable set problem to global optimality by solving a
 * mixed-integer program.
 *
 * Note: This solver leverage convex optimization solvers (e.g. Gurobi and/or
 * Mosek). We recommend enabling those solvers if possible
 * (https://drake.mit.edu/bazel.html#proprietary_solvers).
 */
class MaxStableSetSolverViaMIP final : public MaxStableSetSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxStableSetSolverViaMIP);
  MaxStableSetSolverViaMIP(
      const solvers::SolverId& solver_id = solvers::MosekSolver::id(),
      const solvers::SolverOptions& options = solvers::SolverOptions());

  VectorX<bool> SolveMaxStableSet(
      const Eigen::SparseMatrix<bool>& adjacency_matrix);

  solvers::SolverId solver_id() { return solver_id_; }

  solvers::SolverOptions options() { return options_; }

 private:
  solvers::SolverId solver_id_;
  solvers::SolverOptions options_;
};

struct MaxStableSetOptions {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxStableSetOptions)
  MaxStableSetOptions(
      const MaxStableSetSolverBase* m_solver = new MaxStableSetSolverViaMIP());
  ~MaxStableSetOptions() = default;
  MaxStableSetSolverBase* solver;
};

/**
 * Given the adjacency matrix of an undirected graph, find the maximum stable
 * set within the graph. A stable set is a collection of vertices in a graph
 * such that each no pair of vertices is connected by an edge. This problem is
 * known to be NP-complete, and so the choice of solvers in options determines
 * whether the return of this function is the true maximum stable set in the
 * subgraph (which may take very long to compute), or just some large subgraph.
 * @param adjacency_matrix a symmetric (0,1)-matrix encoding the edge
 * relationship
 * @param options options for solving the max stable set problem.
 * @return A binary vector with the samee indexing as the adjacency matrix, with
 * 1 indicating membership in the stable set.
 */
VectorX<bool> MaxStableSet(const Eigen::SparseMatrix<bool>& adjacency_matrix,
                           const MaxStableSetOptions& options);
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
