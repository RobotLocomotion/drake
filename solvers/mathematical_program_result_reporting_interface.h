#pragma once

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
class MathematicalProgramResultReportingInterface {
 public:
  virtual ~MathematicalProgramResultReportingInterface() {}

  virtual void SetSolverId(SolverId) = 0;

  virtual void SetOptimalCost(double optimal_cost) = 0;

  /**
   * Setter for lower bound on optimal cost. This function is meant
   * to be called by the appropriate solver, not by the user. It sets
   * the lower bound of the cost found by the solver, during the optimization
   * process. For example, for mixed-integer optimization, the branch-and-bound
   * algorithm can find the lower bound of the optimal cost, during the
   * branching process.
   */
  virtual void SetLowerBoundCost(double lower_bound_cost) = 0;

  virtual void SetDecisionVariableValue(const symbolic::Variable& var,
                                        double value) = 0;
  /**
   * Sets the values of all decision variables, such that the value of
   * \p decision_variables_(i) is \p values(i).
   * @param values The values set to all the decision variables.
   */
  virtual void SetDecisionVariableValues(
      const Eigen::Ref<const Eigen::VectorXd>& values) = 0;

  virtual void SetDecisionVariableValues(
      const Eigen::Ref<const VectorXDecisionVariable>& variables,
      const Eigen::Ref<const Eigen::VectorXd>& values) = 0;
};
}  // namespace solvers
}  // namespace drake
