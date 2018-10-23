#pragma once

#include <memory>

#include "drake/solvers/mathematical_program_solver_interface.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace solvers {
/* The empty solver details. This is the default option for any solver. */
struct NoSolverDetails {};

/**
 * The result returned by solve(). It stores the SolutionResult (whether the
 * program is solved to optimality, detected infeasibility, etc), the optimal
 * value for the decision variables, the optimal cost, and solver specific
 * details.
 */
class MathematicalProgramResult {
 public:
  /**
   * Constructs the result.
   * @note The solver_details is set to NoSolverDetails.
   */
  MathematicalProgramResult();

  /** Getter for the mutable SolutionResult. */
  SolutionResult& get_mutable_result() { return result_; }

  /** Getter for the immutable SolutionResult. */
  SolutionResult result() const { return result_; }

  /** Getter for the mutable decision variable values. */
  Eigen::VectorXd& get_mutable_x_val() { return x_val_; }

  /** Getter for the immutable decision variable values. */
  const Eigen::VectorXd& x_val() const { return x_val_; }

  /** Getter for the mutable optimal cost. */
  double& get_mutable_optimal_cost() { return optimal_cost_; }

  /** Getter for the optimal cost. */
  double optimal_cost() const { return optimal_cost_; }

  /** Getter for the mutable solver id. */
  SolverId& get_mutable_solver_id() { return solver_id_; }

  /** Getter for the solver ID. */
  const SolverId& solver_id() const { return solver_id_; }

  /**
   * Sets the solver details.
   * @param solver_details The details from the solver. solver_details will not
   * own its recource after calling this function.
   */
  void SetSolverDetails(std::unique_ptr<systems::AbstractValue> solver_details);

  /** Getter for the solver details. Throws an error if the solver_details has
   * not been set*/
  const systems::AbstractValue& solver_details() const;

 private:
  SolutionResult result_;
  Eigen::VectorXd x_val_;
  double optimal_cost_;
  SolverId solver_id_;
  std::unique_ptr<systems::AbstractValue> solver_details_;
};
}  // namespace solvers
}  // namespace drake
