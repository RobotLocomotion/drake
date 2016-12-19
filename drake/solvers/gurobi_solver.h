#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * This contains contains some of the solution result by calling Gurobi.
 */
class GurobiSolverResult : public MathematicalProgramSolverResult {
 public:
  GurobiSolverResult(SolutionSummary summary, int gurobi_error, int status)
      : MathematicalProgramSolverResult(summary),
        gurobi_error_(gurobi_error),
        status_(status) {}

  /** Getter for gurobi_error. For more details, refer to
   * http://www.gurobi.com/documentation/6.5/refman/error_codes.html
   */
  int gurobi_error() const { return gurobi_error_; }

  /** Getter for gurobi status. For more details, refer to
   * http://www.gurobi.com/documentation/6.5/refman/optimization_status_codes.html
   */
  int status() const { return status_; }

 private:
  int gurobi_error_;  // The error encountered by calling Gurobi functions,
                      // refer
  // to http://www.gurobi.com/documentation/6.5/refman/error_codes.html
  // for more details.
  int status_;  // The status after the optimize call has returned.
                // Refer to
  // http://www.gurobi.com/documentation/6.5/refman/optimization_status_codes.html
  // for more details.
};

class GurobiSolver : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // Gurobi was available during compilation.
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<GurobiSolverResult> Solve(MathematicalProgram& prog) const {
    return std::unique_ptr<GurobiSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override { return "Gurobi"; }

  GurobiSolverResult* Solve_impl(MathematicalProgram& prog) const override;
};

}  // end namespace solvers
}  // end namespace drake
