#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * This class is used by implementations of the class SolverInterface to report
 * their results to the mathematical program. It is guaranteed to have a
 * defined solver id; all other fields can be left undefined. Reading those
 * values should be guarded by a test on whether the field has been defined.
 */
class SolverResult {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverResult)

  explicit SolverResult(const SolverId& solver_id) : solver_id_(solver_id) {}

  const SolverId& solver_id() const { return solver_id_; }

  /**
   * Sets the decision variable values. Eventually the decision variable values
   * in this class will be passed to MathematicalProgram, to set the values for
   * ALL variables in the mathematical program, such that
   * MathematicalProgram::decision_variables_(i) will have value `values(i)`.
   */
  void set_decision_variable_values(
      const Eigen::Ref<const Eigen::VectorXd>& values);

  const optional<Eigen::VectorXd>& decision_variable_values() const {
    return decision_variable_values_;
  }

  /**
   * Sets the optimal cost. Eventually this optimal cost will be passed to
   * MathematicalProgram, when calling
   * MathematicalProgram::SetSolverResult(...);

   */
  void set_optimal_cost(double optimal_cost) { optimal_cost_ = optimal_cost; }

  const optional<double>& optimal_cost() const { return optimal_cost_; }

  /**
   * Sets the lower bound of the optimal cost. Eventually this lower bound of
   * the optimal cost will be passed to MathematicalProgram, when calling
   * MathematicalProgram::SetSolverResult(...);
   * Some problems, such as mixed-integer convex program, can find a lower bound
   * of its optimal cost, before finding the global optimal.
   */
  void set_optimal_cost_lower_bound(double val) {
    optimal_cost_lower_bound_ = val;
  }

  const optional<double>& optimal_cost_lower_bound() const {
    return optimal_cost_lower_bound_;
  }

 private:
  SolverId solver_id_;
  optional<Eigen::VectorXd> decision_variable_values_{nullopt};
  optional<double> optimal_cost_{nullopt};
  optional<double> optimal_cost_lower_bound_{nullopt};
};

}  // namespace solvers
}  // namespace drake
