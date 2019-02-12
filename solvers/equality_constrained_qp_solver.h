#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/**
 * Solves a quadratic program with equality constraint.
 *
 * This program doesn't depend on the initial guess.
 *
 * The user can set the following options:
 *
 * - FeasibilityTolOptionName(). The feasible solution (both primal and dual
 *   variables) should satisfy their constraints, with error no larger than
 *   this value. The default is Eigen::dummy_precision().
 */
class EqualityConstrainedQPSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EqualityConstrainedQPSolver)

  EqualityConstrainedQPSolver();
  ~EqualityConstrainedQPSolver() final;

  /// @returns string key for SolverOptions to set the feasibility tolerance.
  static std::string FeasibilityTolOptionName();

  /// A static flavor of the base class method solver_id().
  static SolverId id();

  /// A static flavor of the base class method available().
  static bool is_available();

  /// A static flavor of the base class method AreProgramAttributesSatisfied().
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
