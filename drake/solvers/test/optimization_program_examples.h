// This file contains a list of optimization problems, to be tested by various
// solvers

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
void testLinearPrograms(const MathematicalProgramSolverInterface& solver);

void testQuadraticPrograms(const MathematicalProgramSolverInterface& solver);

void testSecondOrderConicPrograms(const MathematicalProgramSolverInterface& solver);

} // namespace test
} // namespace solvers
} // namespace drake
