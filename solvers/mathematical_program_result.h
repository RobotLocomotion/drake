#pragma once

#include <memory>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include "drake/common/symbolic.h"
#include "drake/common/value.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_result.h"

namespace drake {
namespace solvers {
/**
 * The result returned by MathematicalProgram::Solve(). It stores the
 * solvers::SolutionResult (whether the program is solved to optimality,
 * detected infeasibility, etc), the optimal value for the decision variables,
 * the optimal cost, and solver specific details.
 */
class MathematicalProgramResult final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MathematicalProgramResult)

  /**
   * Constructs the result.
   * @note The solver_details is set to nullptr.
   */
  MathematicalProgramResult();

  /** Returns true if the optimization problem is solved successfully; false
   * otherwise.
   * For more information on the solution status, the user could call
   * get_solver_details() to obtain the solver-specific solution status.
   */
  bool is_success() const;

  /**
   * Sets decision_variable_index mapping, that maps each decision variable to
   * its index in the aggregated vector containing all decision variables in
   * MathematicalProgram.
   */
  void set_decision_variable_index(
      std::unordered_map<symbolic::Variable::Id, int> decision_variable_index) {
    decision_variable_index_ = std::move(decision_variable_index);
  }

  /** Sets SolutionResult. */
  void set_solution_result(SolutionResult solution_result) {
    solution_result_ = solution_result;
  }

  /** Gets the decision variable values. */
  const Eigen::VectorXd& get_x_val() const { return x_val_; }

  /** Gets SolutionResult. */
  SolutionResult get_solution_result() const { return solution_result_; }

  /** Sets the decision variable values. */
  void set_x_val(const Eigen::VectorXd& x_val);

  /** Gets the optimal cost. */
  double get_optimal_cost() const { return optimal_cost_; }

  /** Sets the optimal cost. */
  void set_optimal_cost(double optimal_cost) { optimal_cost_ = optimal_cost; }

  /** Gets the solver ID. */
  const SolverId& get_solver_id() const { return solver_id_; }

  /** Sets the solver ID. */
  void set_solver_id(const SolverId& solver_id) { solver_id_ = solver_id; }

  /** Gets the solver details. Throws an error if the solver_details has not
   * been set.*/
  const AbstractValue& get_solver_details() const;

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
      solver_details_ = std::make_unique<Value<T>>();
    }
    return solver_details_->GetMutableValue<T>();
  }

  /**
   * Convert MathematicalProgramResult to SolverResult.
   * @note This function doesn't set optimal_cost_lower_bound. If
   * SolverResult.optimal_cost_lower_bound needs to be set (like in
   * GurobiSolver), then the user will have to set it after calling
   * ConvertToSolverResult.
   */
  SolverResult ConvertToSolverResult() const;

  /**
   * Gets the solution of an Eigen matrix of decision variables.
   * @tparam Derived An Eigen matrix containing Variable.
   * @param var The decision variables.
   * @return The value of the decision variable after solving the problem.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetSolution(const Eigen::MatrixBase<Derived>& var) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(var.rows(), var.cols());
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        value(i, j) = GetSolution(var(i, j));
      }
    }
    return value;
  }

  /**
   * Gets the solution of a single decision variable.
   * @param var The decision variable.
   * @return The value of the decision variable after solving the problem.
   * @throws invalid_argument if `var` is not captured in the mapping @p
   * decision_variable_index, as the input argument of
   * set_decision_variable_index().
   */
  double GetSolution(const symbolic::Variable& var) const;

 private:
  optional<std::unordered_map<symbolic::Variable::Id, int>>
      decision_variable_index_{};
  SolutionResult solution_result_{};
  Eigen::VectorXd x_val_;
  double optimal_cost_{};
  SolverId solver_id_;
  reset_after_move<const std::type_info*> solver_details_type_;
  copyable_unique_ptr<AbstractValue> solver_details_;
};

}  // namespace solvers
}  // namespace drake
