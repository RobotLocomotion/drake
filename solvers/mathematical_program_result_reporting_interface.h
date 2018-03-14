#pragma once

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
namespace internal {
/**
 * Implementations of MathematicalProgramSolverInterface use this
 * MathematicalProgramResultReportingInterface to report the optimization
 * results to MathematicalProgram.
 * The functions inside this class should only be called
 * from each solver class, instead of by users.
 */
class MathematicalProgramResultReportingInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgramResultReportingInterface)

  MathematicalProgramResultReportingInterface() {}

  virtual ~MathematicalProgramResultReportingInterface() {}

  /**
   * Sets the ID of the solver in the program.
   */
  virtual void SetSolverId(SolverId) = 0;

  /**
   * Sets the optimal cost.
   */
  virtual void SetOptimalCost(double optimal_cost) = 0;

  /**
   * Sets the lower bound on optimal cost. It sets the lower bound of the
   * cost found by the solver, during the optimization process. For example, for
   * mixed-integer optimization, the branch-and-bound algorithm can find the
   * lower bound of the optimal cost, during the branching process.
   */
  virtual void SetLowerBoundCost(double lower_bound_cost) = 0;

  /**
   * Sets the value of a single decision variable.
   * @param var The decision variable whose value is set.
   * @param value The value of the decision variable.
   */
  virtual void SetDecisionVariableValue(const symbolic::Variable& var,
                                           double value) = 0;
  /**
   * Sets the values of all decision variables.
   * @param values The values set to all the decision variables.
   */
  virtual void SetDecisionVariableValues(
      const Eigen::Ref<const Eigen::VectorXd>& values) = 0;

  /**
   * Sets the values of decision variables, such that the value of
   * \p variables(i) is \p values(i).
   * @param variables The decision variables whose values are set.
   * @param values The values of the decision variables.
   */
  virtual void SetDecisionVariableValues(
      const Eigen::Ref<const VectorXDecisionVariable>& variables,
      const Eigen::Ref<const Eigen::VectorXd>& values) = 0;
};
}  // namespace internal
}  // namespace solvers
}  // namespace drake
