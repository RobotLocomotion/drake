#pragma once

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
 * @throws This solver throws if no Mixed-Integer Linear Programming solver is
 * available.
 */
class MaxCliqueSolverViaMip final : public MaxCliqueSolverBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MaxCliqueSolverViaMip);
  MaxCliqueSolverViaMip() = default;

  MaxCliqueSolverViaMip(const std::optional<Eigen::VectorXd>& initial_guess,
                        const solvers::SolverOptions& solver_options);

  //  VectorX<bool> SolveMaxClique(
  //      const Eigen::SparseMatrix<bool>& adjacency_matrix) const;

  [[nodiscard]] solvers::SolverOptions solver_options() const {
    return solver_options_;
  }

  void set_initial_guess(
      const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
    initial_guess_ = initial_guess;
  }

  [[nodiscard]] std::optional<Eigen::VectorXd> get_initial_guess() const {
    return initial_guess_;
  }

 private:
  /** Initial guess to the MIP for solving max clique. */
  std::optional<Eigen::VectorXd> initial_guess_{std::nullopt};

  /** Options solved to the MIP solver used to solve max clique. */
  solvers::SolverOptions solver_options_;

  VectorX<bool> DoSolveMaxClique(
      const Eigen::SparseMatrix<bool>& adjacency_matrix) const final;
};

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
