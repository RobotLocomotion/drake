#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {

class LinearSystemSolver : public SolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSolver)

  LinearSystemSolver() = default;
  ~LinearSystemSolver() override = default;

  bool available() const override { return is_available(); };

  static bool is_available();

  /// Find the least-square solution to the linear system A * x = b.
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
};

}  // namespace solvers
}  // namespace drake
