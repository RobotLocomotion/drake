#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/symbolic.h"
#include "drake/common/value.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
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
 * @throws std::exception if var.get_id() is not a valid key of @p
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
    std::is_same_v<typename Derived::Scalar, symbolic::Variable>,
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

  /** Sets the dual solution associated with a given constraint. */
  template <typename C>
  void set_dual_solution(
      const Binding<C>& constraint,
      const Eigen::Ref<const Eigen::VectorXd>& dual_solution) {
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    dual_solutions_.emplace(constraint_cast, dual_solution);
  }

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
  typename std::enable_if_t<
      std::is_same_v<typename Derived::Scalar, symbolic::Variable>,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>
  GetSolution(const Eigen::MatrixBase<Derived>& var) const {
    return GetVariableValue(var, decision_variable_index_, x_val_);
  }

  /**
   * Gets the solution of a single decision variable.
   * @param var The decision variable.
   * @return The value of the decision variable after solving the problem.
   * @throws std::exception if `var` is not captured in the mapping @p
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
   * Substitutes the value of all decision variables into the coefficients of
   * the symbolic polynomial.
   * @param p A symbolic polynomial. Its indeterminates can't intersect with the
   * set of decision variables of the MathematicalProgram from which this result
   * is obtained.
   * @return the symbolic::Polynomial as the result of the substitution.
   */
  symbolic::Polynomial GetSolution(const symbolic::Polynomial& p) const;

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
  typename std::enable_if_t<
      std::is_same_v<typename Derived::Scalar, symbolic::Expression>,
      Eigen::Matrix<symbolic::Expression, Derived::RowsAtCompileTime,
          Derived::ColsAtCompileTime>>
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

  // TODO(hongkai.dai): add the interpretation for other type of constraints
  // when we implement them.
  /**
   * Gets the dual solution associated with a constraint.
   *
   * For constraints in the form lower <= f(x) <= upper (including linear
   * inequality, linear equality, bounding box constraints, and general
   * nonlinear constraints), we interpret the dual variable value as the "shadow
   * price" of the original problem. Namely if we change the constraint bound by
   * one unit (each unit is infinitesimally small), the change of the optimal
   * cost is the value of the dual solution times the unit. Mathematically
   * dual_solution = ∂optimal_cost / ∂bound.
   *
   * For a linear equality constraint Ax = b where b ∈ ℝⁿ, the vector of dual
   * variables has n rows, and dual_solution(i) is the value of the dual
   * variable for the constraint A(i,:)*x = b(i).
   *
   * For a linear inequality constraint lower <= A*x <= upper where lower and
   * upper ∈ ℝⁿ, dual_solution also has n rows. dual_solution(i) is the value of
   * the dual variable for constraint lower(i) <= A(i,:)*x <= upper(i). If
   * neither side of the constraint is active, then dual_solution(i) is 0. If
   * the left hand-side lower(i) <= A(i, :)*x is active (meaning lower(i) = A(i,
   * :)*x at the solution), then dual_solution(i) is non-negative (because the
   * objective is to minimize a cost, increasing the lower bound means the
   * constraint set is tighter, hence the optimal solution cannot decrease. Thus
   * the shadow price is non-negative). If the right hand-side A(i,
   * :)*x<=upper(i) is active (meaning A(i,:)*x=upper(i) at the solution), then
   * dual_solution(i) is non-positive.
   *
   * For a bounding box constraint lower <= x <= upper, the interpretation of
   * the dual solution is the same as the linear inequality constraint.
   *
   * For a Lorentz cone or rotated Lorentz cone constraint that Ax + b is in
   * the cone, depending on the solver, the dual solution has different
   * meanings:
   * 1. If the solver is Gurobi, then the user can only obtain the dual solution
   *    by explicitly setting the options for computing dual solution.
   *    @code
   *    auto constraint = prog.AddLorentzConeConstraint(...);
   *    GurobiSolver solver;
   *    // Explicitly tell the solver to compute the dual solution for Lorentz
   *    // cone or rotated Lorentz cone constraint, check
   *    // https://www.gurobi.com/documentation/9.0/refman/qcpdual.html for
   *    // more information.
   *    SolverOptions options;
   *    options.SetOption(GurobiSolver::id(), "QCPDual", 1);
   *    MathematicalProgramResult result = solver.Solve(prog, {}, options);
   *    Eigen::VectorXd dual_solution = result.GetDualSolution(constraint);
   *    @endcode
   *    The dual solution has size 1, dual_solution(0) is the shadow price for
   *    the constraint z₁² + ... +zₙ² ≤ z₀² for Lorentz cone constraint, and
   *    the shadow price for the constraint z₂² + ... +zₙ² ≤ z₀z₁ for rotated
   *    Lorentz cone constraint, where z is the slack variable representing z =
   *    A*x+b and z in the Lorentz cone/rotated Lorentz cone.
   * 2. For nonlinear solvers like IPOPT, the dual solution for Lorentz cone
   *    constraint (with EvalType::kConvex) is the shadow price for
   *    z₀ - sqrt(z₁² + ... +zₙ²) ≥ 0, where z = Ax+b.
   * 3. For other convex conic solver such as SCS, Mosek, CSDP, etc, the dual
   *    solution to the (rotated) Lorentz cone constraint doesn't have the
   *    "shadow price" interpretation, but should lie in the dual cone, and
   *    satisfy the KKT condition. For more information, refer to
   *    https://docs.mosek.com/9.2/capi/prob-def-conic.html#duality-for-conic-optimization
   *    as an explanation.
   *
   * The interpretation for the dual variable to conic constraint x ∈ K can be
   * different. Here K is a convex cone, including exponential cone, power
   * cone, PSD cone, etc. When the problem is solved by a convex solver (like
   * SCS, Mosek, CSDP, etc), often it has a dual variable z ∈ K*, where K* is
   * the dual cone. Here the dual variable DOESN'T have the interpretation of
   * "shadow price", but should satisfy the KKT condition, while the dual
   * variable stays inside the dual cone.
   */
  template <typename C>
  Eigen::VectorXd GetDualSolution(const Binding<C>& constraint) const {
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    auto it = dual_solutions_.find(constraint_cast);
    if (it == dual_solutions_.end()) {
      // Throws a more meaningful error message when the user wants to retrieve
      // a dual solution from a Gurobi result for a program containing second
      // order cone constraints, but forgot to explicitly turn on the flag to
      // compute Gurobi QCP dual.
      if (solver_id_.name() == "Gurobi") {
        throw std::invalid_argument(fmt::format(
            "You used {} to solve this optimization problem. If the problem is "
            "solved to optimality and doesn't contain binary/integer "
            "variables, but you failed to get the dual solution, "
            "check that you have explicitly told Gurobi solver to "
            "compute the dual solution for the second order cone constraints "
            "by setting the solver options. One example is as follows: "
            "SolverOptions options; "
            "options.SetOption(GurobiSolver::id(), \"QCPDual\", 1); "
            "auto result=Solve(prog, std::nullopt, options);",
            solver_id_.name()));
      }
      throw std::invalid_argument(fmt::format(
          "Either this constraint does not belong to the "
          "mathematical program for which the result is obtained, or "
          "{} does not currently support getting dual solution yet.",
          solver_id_.name()));
    } else {
      return it->second;
    }
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
  typename std::enable_if_t<
      std::is_same_v<typename Derived::Scalar, symbolic::Variable>,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>
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

  /** @anchor get_infeasible_constraints
   * @name Get infeasible constraints
   * Some solvers (e.g. SNOPT) provide a "best-effort solution" even when they
   * determine that a problem is infeasible.  This method will return the
   * descriptions corresponding to the constraints for which `CheckSatisfied`
   * evaluates to false given the reported solution.  This can be very useful
   * for debugging. Note that this feature is available only when the
   * optimization problem is solved through certain solvers (like SNOPT, IPOPT)
   * which provide a "best-effort solution". Some solvers (like Gurobi) don't
   * return the "best-effort solution" when the problem is infeasible, and this
   * feature is hence unavailable.
   */
  //@{

  /**
   * See @ref get_infeasible_constraints for more information.
   * @param prog The MathematicalProgram that was solved to obtain `this`
   * MathematicalProgramResult.
   * @param tolerance A positive tolerance to check the constraint violation.
   * If no tolerance is provided, this method will attempt to obtain the
   * constraint tolerance from the solver, or insert a conservative default
   * tolerance.
   *
   * Note: Currently most constraints have the empty string as the
   * description, so the NiceTypeName of the Constraint is used instead.  Use
   * e.g.
   * `prog.AddConstraint(x == 1).evaluator().set_description(str)`
   * to make this method more specific/useful. */
  std::vector<std::string> GetInfeasibleConstraintNames(
      const MathematicalProgram& prog,
      std::optional<double> tolerance = std::nullopt) const;

  /**
   * See @ref get_infeasible_constraints for more information.
   * @param prog The MathematicalProgram that was solved to obtain `this`
   * MathematicalProgramResult.
   * @param tolerance A positive tolerance to check the constraint violation.
   * If no tolerance is provided, this method will attempt to obtain the
   * constraint tolerance from the solver, or insert a conservative default
   * tolerance.
   * @return infeasible_bindings A vector of all infeasible bindings
   * (constraints together with the associated variables) at the best-effort
   * solution.
   */
  std::vector<Binding<Constraint>> GetInfeasibleConstraints(
      const MathematicalProgram& prog,
      std::optional<double> tolerance = std::nullopt) const;
  // @}

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
  // Stores the dual variable solutions for each constraint.
  std::unordered_map<Binding<Constraint>, Eigen::VectorXd> dual_solutions_{};
};

}  // namespace solvers
}  // namespace drake
