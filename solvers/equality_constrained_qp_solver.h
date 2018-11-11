#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class EqualityConstrainedQPSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EqualityConstrainedQPSolver)

  EqualityConstrainedQPSolver() = default;
  ~EqualityConstrainedQPSolver() override = default;

  bool available() const override { return is_available(); };

  static bool is_available();

  /**
   * Solve the qudratic program with equality constraint.
   * The user can set the following options
   *  FeasibilityTolOptionName() The feasible solution (both primal and dual
   *  variables) should satisfy their constraints, with error no
   *  larger than this value. The default is Eigen::dummy_precision().
   */
  SolutionResult Solve(MathematicalProgram& prog) const override;

  /**
   * Solve the qudratic program with equality constraint.
   * This program doesn't depend on the initial guess.
   * The user can set the following options
   *  FeasibilityTolOptionName(). The feasible solution (both primal and dual
   *  variables) should satisfy their constraints, with error no
   *  larger than this value. The default is Eigen::dummy_precision().
   *  solver_options will take priority over any options stored inside prog.
   */
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

  /** Returns the string as a key value in SolverOption, to set the feasibility
   * tolerance.
   */
  static std::string FeasibilityTolOptionName();
};

}  // namespace solvers
}  // namespace drake
