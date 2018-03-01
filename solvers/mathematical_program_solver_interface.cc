#include "drake/solvers/mathematical_program_solver_interface.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
void MathematicalProgramSolverInterface::SetSolverIdInsideMathematicalProgram(
    MathematicalProgram* prog) const {
  SetSolverIdKey key(solver_id());
  prog->SetSolverId(key);
}
}  // namespace solvers
}  // namespace drake
