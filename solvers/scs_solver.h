#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
/** The SCS solver details after calling Solve function in ScsSolver. The user
 * can obtain the details from
 * MathematicalProgramResult.get_solver_details().GetValue<ScsSolverDetails>();
 */
struct ScsSolverDetails {
  /// The status of the solver at termination. Please refer to
  /// https://github.com/cvxgrp/scs/blob/master/include/glbopts.h
  /// Note that the SCS code on github master might be slightly more up-to-date
  /// than the version used in Drake.
  int scs_status{};
  /// These are the information returned by SCS at termination, please refer to
  /// "SCS_INFO" struct in
  /// https://github.com/cvxgrp/scs/blob/master/include/scs.h Number of
  /// iterations taken at termination.
  /// Equal to SCS_INFO.iter
  int iter{};
  /// Primal objective value at termination.
  /// Equal to SCS_INFO.pobj
  double primal_objective{};
  /// Dual objective value at termination.
  /// Equal to SCS_INFO.dobj
  double dual_objective{};
  /// Primal equality residue.
  /// Equal to SCS_INFO.res_pri
  double primal_residue{};
  /// infeasibility certificate residue.
  /// Equal to SCS_INFO.res_infeas
  double residue_infeasibility{};
  /// unbounded certificate residue.
  /// Equal to SCS_INFO.res_unbdd
  double residue_unbounded{};
  /// relative duality gap.
  /// Equal to SCS_INFO.rel_gap.
  double relative_duality_gap{};
  /// Time taken for SCS to setup in milliseconds.
  /// Equal to SCS_INFO.setup_time.
  double scs_setup_time{};
  /// Time taken for SCS to solve in millisecond.
  /// Equal to SCS_INFO.solve_time.
  double scs_solve_time{};

  /// The dual variable values at termination.
  Eigen::VectorXd y;
  /// The primal equality constraint slack, namely
  /// Ax + s = b where x is the primal variable.
  Eigen::VectorXd s;
};

class ScsSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScsSolver)

  ScsSolver() = default;
  ~ScsSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // SCS was available during compilation.
  bool available() const override { return is_available(); };

  static bool is_available();

  SolutionResult Solve(MathematicalProgram& prog) const override;

  void Solve(const MathematicalProgram& prog,
             const optional<Eigen::VectorXd>& initial_guess,
             const optional<SolverOptions>& solver_options,
             MathematicalProgramResult* result) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);

  // If the user want the solver to print out debug message, then set this to
  // true; otherwise set it to false. The default is false.
  // TODO(hongkai.dai): add function to set the option through
  // MathematicalProgram.
  DRAKE_DEPRECATED(
      "Use SolverOptions::SetOption(ScsSolver::id(), \"verbose\", true) to set "
      "the verbose option.")
  void SetVerbose(bool) {
    throw std::runtime_error(
        "Use SolverOptions::SetOption(ScsSolver::id(), \"verbose\", true) to "
        "set the verbose options to true.");
  }
};

}  // namespace solvers
}  // namespace drake
