#pragma once

#include <optional>

#include <Eigen/Sparse>

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

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
  MaxCliqueSolverBase() {}

  virtual ~MaxCliqueSolverBase() {}

  virtual VectorX<bool> SolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) = 0;

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverBase);
};

/**
 * Solves the maximum clique problem to global optimality by solving the
 * mixed-integer program
 *
 * Maximize ∑ᵢ xᵢ subject to
 * xᵢ + xⱼ ≤ 1 if (i,j) is not in the edge
 * xᵢ ∈ {0,1}.
 *
 * Note: This solver requires the availability of a Mixed-Integer Linear
 * Programming solver (e.g. Gurobi and/or Mosek). We recommend enabling those
 * solvers if possible (https://drake.mit.edu/bazel.html#proprietary_solvers).
 */
class MaxCliqueSolverViaMIP final : public MaxCliqueSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverViaMIP);
  MaxCliqueSolverViaMIP() = default;

  MaxCliqueSolverViaMIP(solvers::SolverInterface* mip_solver,
                        std::optional<Eigen::VectorXd> initial_guess,
                        solvers::SolverOptions options);

  VectorX<bool> SolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix);

  solvers::SolverInterface* mip_solver() { return mip_solver_; }

  solvers::SolverOptions solver_options() { return solver_options_; }

  void set_initial_guess(
      const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
    initial_guess_ = initial_guess;
  }

  std::optional<Eigen::VectorXd> get_initial_guess() { return initial_guess_; }

 private:
  /** Optimizer to be used to solve the max clique MIP. If
  not set, the best solver for the given problem is selected. Note that if the
  solver cannot handle the type of optimization problem generated, the calling
  solve method will throw. */
  solvers::SolverInterface* mip_solver_{nullptr};

  /** Initial guess to the MIP for solving max clique. */
  std::optional<Eigen::VectorXd> initial_guess_{std::nullopt};

  /** Options solved to the MIP solver used to solve max clique. */
  solvers::SolverOptions solver_options_{};
};

struct MaxCliqueOptions {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueOptions)
  MaxCliqueOptions(
      const MaxCliqueSolverBase* m_solver = new MaxCliqueSolverViaMIP());
  ~MaxCliqueOptions() = default;
  MaxCliqueSolverBase* solver;
};

/**
 * Given the adjacency matrix of an undirected graph, find the maximum clique
 * within the graph. A clique is a collection of vertices in a graph such that
 * each pair of vertices is connected by an edge (i.e. a fully connected
 * subgraph). This problem is known to be NP-complete, and so the choice of
 * solvers in @param options determines whether the return of this function is
 * the true maximum clique in the subgraph (which may take very long to
 * compute), or only an approximate solution found via heuristics.
 * @param adjacency_matrix a symmetric (0,1)-matrix encoding the edge
 * relationship
 * @param options options for solving the max-clique problem.
 * @return A binary vector with the same indexing as the adjacency matrix, with
 * 1 indicating membership in the clique.
 */
VectorX<bool> MaxClique(const Eigen::SparseMatrix<bool>& adjacency_matrix,
                        const MaxCliqueOptions& options);

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
