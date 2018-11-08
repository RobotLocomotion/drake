#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class OsqpSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OsqpSolver)

  OsqpSolver() = default;
  ~OsqpSolver() override = default;

  // This solver is implemented in various pieces depending on if Osqp was
  // available during compilation.
  bool available() const override { return is_available(); };

  static bool is_available();

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
