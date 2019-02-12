#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {

/**
 * The details of SNOPT solvers after calling Solve function. The users can get
 * the details by
 * MathematicalProgramResult::get_solver_details().GetValue<SnoptSolverDetails>();
 */
struct SnoptSolverDetails {
  /**
   * The exit condition of the solver. Please refer to section "EXIT conditions"
   * in "User's Guide for SNOPT Version 7: Software for Large-Scale Nonlinear
   * Programming" by Philip E. Gill to interprete the exit condition.
   */
  int info{};

  /** The final value of the dual variables for the bound constraint x_lower <=
   * x <= x_upper.
   */
  Eigen::VectorXd xmul;
  /** The final value of the vector of problem functions F(x).
   */
  Eigen::VectorXd F;
  /** The final value of the dual variables (Lagrange multipliers) for the
   * general constraints F_lower <= F(x) <= F_upper.
   */
  Eigen::VectorXd Fmul;
};

class SnoptSolver : public SolverInterface  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptSolver)

  SnoptSolver() = default;
  ~SnoptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  bool available() const override { return is_available(); };

  static bool is_available();

  SolutionResult Solve(MathematicalProgram& prog) const override;

  void Solve(const MathematicalProgram& prog,
             const optional<Eigen::VectorXd>& initial_guess,
             const optional<SolverOptions>& solver_options,
             MathematicalProgramResult* result) const override;

  SolverId solver_id() const override;

  /// @return same as SolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);

  /// @return if the solver is thread safe. SNOPT f2c interface uses global
  /// variables, hence it is not thread safe. SNOPT fortran interface is thread
  /// safe.
  static bool is_thread_safe();

  /// For some reason, SNOPT 7.4 fails to detect a simple LP being unbounded.
  static bool is_bounded_lp_broken();
};

}  // namespace solvers
}  // namespace drake
