#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class SnoptSolverResult : public MathematicalProgramSolverResult {
 public:
  SnoptSolverResult(SolutionSummary summary, int snopt_info,
                    double objective_value)
      : MathematicalProgramSolverResult(summary),
        snopt_info_(snopt_info),
        objective_value_(objective_value) {}

  /** Getter for snopt info. Refer to Section 8.6 (EXIT conditions) of SNOPT
   * manual
   * http://web.stanford.edu/group/SOL/guides/sndoc7.pdf for more information.
   */
  int snopt_info() const { return snopt_info_; }

  /** Getter for objective value.*/
  double objective_value() const { return objective_value_; }

 private:
  // The returned code from calling snopta_. Refer to
  // http://web.stanford.edu/group/SOL/guides/sndoc7.pdf for more details.
  const int snopt_info_;

  const double objective_value_;
  // TODO(hongkai.dai): Add Lagrangian multiplier and constraint value.
};
class SnoptSolver : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<SnoptSolverResult> Solve(
      MathematicalProgram* const prog) const {
    return std::unique_ptr<SnoptSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override { return "SNOPT"; }

  SnoptSolverResult* Solve_impl(MathematicalProgram* const prog) const override;
};

}  // namespace solvers
}  // namespace drake
