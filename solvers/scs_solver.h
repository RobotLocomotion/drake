#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * The SCS solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<ScsSolver>() to obtain the
 * details.
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

class ScsSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScsSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = ScsSolverDetails;

  ScsSolver();
  ~ScsSolver() final;

  // TODO(hongkai.dai): add function to set the verbosity through SolverOptions
  // or MathematicalProgram.

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
