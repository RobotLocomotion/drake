#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/**
 * The SNOPT solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<SnoptSolver>() to obtain the
 * details.
 */
struct SnoptSolverDetails {
  /**
   * The exit condition of the solver. Please refer to section "EXIT conditions"
   * in "User's Guide for SNOPT Version 7: Software for Large-Scale Nonlinear
   * Programming" by Philip E. Gill to interpret the exit condition.
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

/**
 * An implementation of SolverInterface for the commercially-licensed SNOPT
 * solver (https://ccom.ucsd.edu/~optimizers/solvers/snopt/).
 *
 * Builds of Drake from source do not compile SNOPT by default, so therefore
 * SolverInterface::available() will return false. You must opt-in to build
 * SNOPT per the documentation at https://drake.mit.edu/bazel.html#snopt.
 *
 * <a href="https://drake.mit.edu/installation.html">Drake's
 * pre-compiled binary releases</a> do incorporate SNOPT, so therefore
 * SolverInterface::available() will return true.
 * Thanks to Philip E. Gill and Elizabeth Wong for their kind support.
 *
 * There is no license configuration required to use SNOPT, so
 * SolverInterface::enabled() will always return true.
 */
class SnoptSolver final : public SolverBase  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = SnoptSolverDetails;

  SnoptSolver();
  ~SnoptSolver() final;

  /// For some reason, SNOPT 7.4 fails to detect a simple LP being unbounded.
  static bool is_bounded_lp_broken();

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
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
