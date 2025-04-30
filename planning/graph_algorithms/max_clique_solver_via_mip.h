#pragma once

#include <memory>
#include <optional>

#include <Eigen/Sparse>

#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

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
 *
 * @throws std::exception if no Mixed-Integer Linear Programming solver is
 * available.
 * @throws std::exception if the initial guess has the wrong size for the
 * provided adjacency matrix.
 */
class MaxCliqueSolverViaMip final : public MaxCliqueSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverViaMip);
  MaxCliqueSolverViaMip() = default;

  MaxCliqueSolverViaMip(const std::optional<Eigen::VectorXd>& initial_guess,
                        const solvers::SolverOptions& solver_options);

  void SetSolverOptions(const solvers::SolverOptions& solver_options) {
    solver_options_ = solver_options;
  }

  [[nodiscard]] solvers::SolverOptions GetSolverOptions() const {
    return solver_options_;
  }

  void SetInitialGuess(const std::optional<Eigen::VectorXd>& initial_guess) {
    initial_guess_ = initial_guess;
  }

  [[nodiscard]] std::optional<Eigen::VectorXd> GetInitialGuess() const {
    return initial_guess_;
  }

 private:
  VectorX<bool> DoSolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const final;

  /* Initial guess to the MIP for solving max clique. */
  std::optional<Eigen::VectorXd> initial_guess_{std::nullopt};

  /* Options solved to the MIP solver used to solve max clique. */
  solvers::SolverOptions solver_options_;
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
