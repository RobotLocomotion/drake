#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/symbolic.h"
#include "drake/common/value.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * Retrieve the value of a single variable @p var from @p variable_values.
 * @param var The variable whose value is going to be retrieved. @p var.get_id()
 * must be a key in @p variable_index.
 * @param variable_index maps the variable ID to its index in @p
 * variable_values.
 * @param variable_values The values of all variables.
 * @return variable_values(variable_index[var.get_id()]) if
 * var.get_id() is a valid key of @p variable_index.
 * @throws an invalid_argument error if var.get_id() is not a valid key of @p
 * variable_index.
 * @pre All the mapped value in variable_index is in the range [0,
 * variable_values.rows())
 */
double GetVariableValue(
    const symbolic::Variable& var,
    const std::optional<std::unordered_map<symbolic::Variable::Id, int>>&
        variable_index,
    const Eigen::Ref<const Eigen::VectorXd>& variable_values);

/**
 * Overload GetVariableValue() function, but for an Eigen matrix of decision
 * variables.
 */
template <typename Derived>
typename std::enable_if_t<
    std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>>
GetVariableValue(
    const Eigen::MatrixBase<Derived>& var,
    const std::optional<std::unordered_map<symbolic::Variable::Id, int>>&
        variable_index,
    const Eigen::Ref<const Eigen::VectorXd>& variable_values) {
  Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
      value(var.rows(), var.cols());
  for (int i = 0; i < var.rows(); ++i) {
    for (int j = 0; j < var.cols(); ++j) {
      value(i, j) =
          GetVariableValue(var(i, j), variable_index, variable_values);
    }
  }
  return value;
}

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
   * MathematicalProgram. Initialize x_val to NAN.
   */
  void set_decision_variable_index(
      std::unordered_map<symbolic::Variable::Id, int> decision_variable_index) {
    decision_variable_index_ = std::move(decision_variable_index);
    x_val_ =
        Eigen::VectorXd::Constant(decision_variable_index_->size(),
                                  std::numeric_limits<double>::quiet_NaN());
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

  /** Gets the solver details for the `Solver` that solved the program. Throws
   * an error if the solver_details has not been set. */
  template <typename Solver>
  const typename Solver::Details& get_solver_details() const {
    return get_abstract_solver_details().
        template get_value<typename Solver::Details>();
  }

  /** (Advanced.) Gets the type-erased solver details. Most users should use
   * get_solver_details() instead. Throws an error if the solver_details has
   * not been set. */
  const AbstractValue& get_abstract_solver_details() const;

  /** (Advanced.) Forces the solver_details to be stored using the given
   * type `T`.  Typically, only an implementation of SolverInterface will
   * call this method.
   * If the storage was already typed as T, this is a no-op.
   * If there were not any solver_details previously, or if it was of a
   * different type, initializes the storage to a default-constructed T.
   * Returns a reference to the mutable solver_details object.
   * The reference remains valid until the next call to this method, or
   * until this MathematicalProgramResult is destroyed. */
  template <typename T>
  T& SetSolverDetailsType() {
    // Leave the storage alone if it already has the correct type.
    if (!solver_details_ ||
        (solver_details_->static_type_info() != typeid(T))) {
      solver_details_ = std::make_unique<Value<T>>();
    }
    return solver_details_->get_mutable_value<T>();
  }

  /**
   * Gets the solution of all decision variables.
   */
  const Eigen::VectorXd& GetSolution() const { return x_val_; }

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
    return GetVariableValue(var, decision_variable_index_, x_val_);
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

  /**
   * Substitutes the value of all decision variables into the Expression.
   * @param e The decision variable.
   * @return the Expression that is the result of the substitution.
   */
  symbolic::Expression GetSolution(const symbolic::Expression& e) const;

  /**
   * Substitutes the value of all decision variables into the
   * Matrix<Expression>.
   * @tparam Derived An Eigen matrix containing Expression.
   * @return the Matrix<Expression> that is the result of the substitution.
   *
   * @exclude_from_pydrake_mkdoc{Including this confuses mkdoc, resulting in
   * doc_was_unable_to_choose_unambiguous_name. }
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Expression>::value,
      Eigen::Matrix<symbolic::Expression, Derived::RowsAtCompileTime,
          Derived::ColsAtCompileTime>>::type
  GetSolution(const Eigen::MatrixBase<Derived>& m) const {
    Eigen::Matrix<symbolic::Expression, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(m.rows(), m.cols());
    for (int i = 0; i < m.rows(); ++i) {
      for (int j = 0; j < m.cols(); ++j) {
        value(i, j) = GetSolution(m(i, j));
      }
    }
    return value;
  }

  /**
   * Evaluate a Binding at the solution.
   * @param binding A binding between a constraint/cost and the variables.
   * @pre The binding.variables() must be the within the decision variables in
   * the MathematicalProgram that generated this %MathematicalProgramResult.
   * @pre The user must have called set_decision_variable_index() function.
   */
  template <typename Evaluator>
  Eigen::VectorXd EvalBinding(const Binding<Evaluator>& binding) const {
    DRAKE_ASSERT(decision_variable_index_.has_value());
    Eigen::VectorXd binding_x(binding.GetNumElements());
    for (int i = 0; i < binding_x.rows(); ++i) {
      binding_x(i) =
          x_val_(decision_variable_index_->at(binding.variables()(i).get_id()));
    }
    Eigen::VectorXd binding_y(binding.evaluator()->num_outputs());
    binding.evaluator()->Eval(binding_x, &binding_y);
    return binding_y;
  }

  /**
   * @anchor solution_pools
   * @name Solution Pools
   * Some solvers (like Gurobi, Cplex, etc) can store a pool of (suboptimal)
   * solutions for mixed integer programming model.
   * @{
   */
  /**
   * Gets the suboptimal solution corresponding to a matrix of decision
   * variables. See @ref solution_pools "solution pools"
   * @param var The decision variables.
   * @param solution_number The index of the sub-optimal solution.
   * @pre @p solution_number should be in the range [0,
   * num_suboptimal_solution()).
   * @return The suboptimal values of the decision variables after solving the
   * problem.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetSuboptimalSolution(const Eigen::MatrixBase<Derived>& var,
                        int solution_number) const {
    return GetVariableValue(var, decision_variable_index_,
                            suboptimal_x_val_[solution_number]);
  }

  /**
   * Gets the suboptimal solution of a decision variable. See @ref
   * solution_pools "solution pools"
   * @param var The decision variable.
   * @param solution_number The index of the sub-optimal solution.
   * @pre @p solution_number should be in the range [0,
   * num_suboptimal_solution()).
   * @return The suboptimal value of the decision variable after solving the
   * problem.
   */
  double GetSuboptimalSolution(const symbolic::Variable& var,
                               int solution_number) const;

  /**
   * Number of suboptimal solutions stored inside MathematicalProgramResult.
   * See @ref solution_pools "solution pools".
   */
  int num_suboptimal_solution() const {
    return static_cast<int>(suboptimal_x_val_.size());
  }

  /**
   * Gets the suboptimal objective value. See @ref solution_pools "solution
   * pools".
   * @param solution_number The index of the sub-optimal solution. @pre @p
   * solution_number should be in the range [0, num_suboptimal_solution()).
   */
  double get_suboptimal_objective(int solution_number) const {
    return suboptimal_objectives_[solution_number];
  }

  /**
   * Adds the suboptimal solution to the result. See @ref solution_pools
   * "solution pools".
   * @param suboptimal_objective The objective value computed from this
   * suboptimal solution.
   * @param suboptimal_x The values of the decision variables in this suboptimal
   * solution.
   */
  void AddSuboptimalSolution(double suboptimal_objective,
                             const Eigen::VectorXd& suboptimal_x);
  //@}

 private:
  std::optional<std::unordered_map<symbolic::Variable::Id, int>>
      decision_variable_index_{};
  SolutionResult solution_result_{};
  Eigen::VectorXd x_val_;
  double optimal_cost_{};
  SolverId solver_id_;
  copyable_unique_ptr<AbstractValue> solver_details_;
  // Some solvers (like Gurobi, Cplex, etc) can store a pool of (suboptimal)
  // solutions for mixed integer programming model.
  // suboptimal_objectives_[i] is the objective value computed with the
  // suboptimal solution suboptimal_x_val_[i].
  std::vector<Eigen::VectorXd> suboptimal_x_val_{};
  std::vector<double> suboptimal_objectives_{};
};

}  // namespace solvers
}  // namespace drake
