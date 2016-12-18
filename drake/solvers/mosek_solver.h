#pragma once

#include <string>

#include <mosek.h>

#include <Eigen/Core>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class MosekSolverResult : public MathematicalProgramSolverResult {
 public:
  MosekSolverResult(SolutionSummary summary, double primal_objective,
                    double dual_objective, MSKprostae problem_status,
                    MSKsolstae solution_status, MSKrescodee response_code) :
      MathematicalProgramSolverResult(summary), primal_objective_(primal_objective),
      dual_objective_(dual_objective), problem_status_(problem_status),
      solution_status_(solution_status), response_code_(response_code) {}

 private:
  // Refer to http://docs.mosek.com/7.0/capi/The_solution_summary.html for more
  // explanations on each term in the result
  const double primal_objective_;
  const double dual_objective_;
  const MSKprostae problem_status_;
  const MSKsolstae solution_status_;
  const MSKrescodee response_code_;
  // TODO(hongkai.dai): Add constraints violation.
};

class MosekSolver : public MathematicalProgramSolverInterface {
 public:
  /** available()
  * Defined true if Mosek was included during compilation, false otherwise.
  */
  bool available() const { return available_impl();}

  std::string SolverName() const { return SolverName_impl();}

  std::unique_ptr<MosekSolverResult> Solve(MathematicalProgram& prog) const;

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const {return "Mosek";}

  MosekSolverResult* Solve_impl(MathematicalProgram& prog) const override;

};

}  // namespace solvers
}  // namespace drake
