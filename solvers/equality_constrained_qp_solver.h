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
   *  FeasibilityTol. The feasible solution (both primal and dual
   *  variables) should satisfy their constraints, with error no
   *  larger than this value. The default is Eigen::dummy_precision().
   */
  SolutionResult Solve(MathematicalProgram& prog) const override;

  void Solve(const MathematicalProgram&, const optional<Eigen::VectorXd>&,
             const optional<SolverOptions>&,
             MathematicalProgramResult*) const override {
    throw std::runtime_error("Not implemented yet.");
  }

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);
};

}  // namespace solvers
}  // namespace drake
