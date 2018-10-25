#pragma once

#include <memory>
#include <utility>

// TODO(hongkai.dai): separate SolutionResult and SolverResult from
// mathematical_program_solver_interface, so that MathematicalProgramResult
// won't link to mathematical_program_api.
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
class MathematicalProgramResult final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MathematicalProgramResult)
  /**
   * Constructs the result.
   * @note The solver_details is set to NoSolverDetails.
   */
  MathematicalProgramResult();

  /** Gets SolutionResult. */
  SolutionResult get_solution_result() const { return solution_result_; }

  /** Sets SolutionResult. */
  void set_solution_result(SolutionResult solution_result) {
    solution_result_ = solution_result;
  }

  /** Gets the decision variable values. */
  const Eigen::VectorXd& get_x_val() const { return x_val_; }

  /** Sets the decision variable values. */
  void set_x_val(const Eigen::VectorXd& x_val) { x_val_ = x_val; }

  /** Gets the optimal cost. */
  double get_optimal_cost() const { return optimal_cost_; }

  /** Sets the optimal cost. */
  void set_optimal_cost(double optimal_cost) { optimal_cost_ = optimal_cost; }

  /** Gets the solver ID. */
  const SolverId& get_solver_id() const { return solver_id_; }

  /** Sets the solver ID. */
  void set_solver_id(const SolverId& solver_id) { solver_id_ = solver_id; }

  /** Gets the solver details. Throws an error if the solver_details has not
   * been set*/
  const systems::AbstractValue& get_solver_details() const;

  /** Forces the solver_details to be stored using the given type T.
   * If the storage was already typed as T, this is a no-op.
   * If there were not any solver_details previously, or if it was of a
   * different type, initializes the storage to a default-constructed T.
   * Returns a reference to the mutable solver_details object.
   * The reference remains valid until the next call to this method, or
   * until this MathematicalProgramResult is destroyed. */
  template <typename T>
  T& SetSolverDetailsType() {
    // Leave the storage alone if it already has the correct type.
    if (solver_details_type_ && (*solver_details_type_ == typeid(T))) {
      DRAKE_ASSERT(solver_details_ != nullptr);
      DRAKE_ASSERT(solver_details_->MaybeGetValue<T>() != nullptr);
    } else {
      solver_details_type_ = &typeid(T);
      solver_details_ = std::make_unique<systems::Value<T>>();
    }
    return solver_details_->GetMutableValue<T>();
  }

  /**
   * Convert MathematicalProgramResult to SolverResult.
   */
  SolverResult ConvertToSolverResult() const;

 private:
  SolutionResult solution_result_{};
  Eigen::VectorXd x_val_;
  double optimal_cost_{};
  SolverId solver_id_;
  reset_after_move<const std::type_info*> solver_details_type_;
  copyable_unique_ptr<systems::AbstractValue> solver_details_;
};

}  // namespace solvers
}  // namespace drake
