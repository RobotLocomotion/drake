#pragma once

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * This is the interface class for the solvers, derived from
 * MathematicalProgramSolverInterface, to report result back to
 * MathematicalProgram. The functions inside this class should only be called
 * from each solver class, instead of by users.
 */
class MathematicalProgramResultReportingInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgramResultReportingInterface)

  MathematicalProgramResultReportingInterface() {}

  virtual ~MathematicalProgramResultReportingInterface() {}

  /**
   * Reports the ID of the solver in the program.
   */
  virtual void ReportSolverId(SolverId) = 0;

  /**
   * Reports the optimal cost.
   */
  virtual void ReportOptimalCost(double optimal_cost) = 0;

  /**
   * Reports the lower bound on optimal cost. It sets the lower bound of the
   * cost found by the solver, during the optimization process. For example, for
   * mixed-integer optimization, the branch-and-bound algorithm can find the
   * lower bound of the optimal cost, during the branching process.
   */
  virtual void ReportLowerBoundCost(double lower_bound_cost) = 0;

  /**
   * Reports the value of a single decision variable.
   * @param var The decision variable whose value is set.
   * @param value The value of the decision variable.
   */
  virtual void ReportDecisionVariableValue(const symbolic::Variable& var,
                                           double value) = 0;
  /**
   * Reports the values of all decision variables, such that the value of
   * \p decision_variables_(i) is \p values(i).
   * @param values The values set to all the decision variables.
   */
  virtual void ReportDecisionVariableValues(
      const Eigen::Ref<const Eigen::VectorXd>& values) = 0;

  /**
   * Reports the values of decision variables, such that the value of
   * \p variables(i) is \p values(i).
   * @param variables The decision variables whose values are set.
   * @param values The values of the decision variables.
   */
  virtual void ReportDecisionVariableValues(
      const Eigen::Ref<const VectorXDecisionVariable>& variables,
      const Eigen::Ref<const Eigen::VectorXd>& values) = 0;
};
}  // namespace solvers
}  // namespace drake
