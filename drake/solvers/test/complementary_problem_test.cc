#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace {
// A problem from J.F. Bard, Convex two-level optimization,
// Mathematical Programming 40(1), 15-27, 1988.
// min (x-5)² + (2*y + 1)²
// s.t 2*(y-1) - 1.5*x + l(0) - 0.5*l(1) + l(2) = 0
//     0 <= l(0) ⊥ 3 * x - y - 3 >= 0
//     0 <= l(1) ⊥ -x + 0.5*y + 4 >= 0
//     0 <= l(2) ⊥ -x - y + 7 >= 0
//     x >= 0, y >= 0
GTEST_TEST(TestComplementaryProblem, bard1) {
MathematicalProgram prog;
auto x = prog.NewContinuousVariables<1>()(0);
auto y = prog.NewContinuousVariables<1>()(0);
auto l = prog.NewContinuousVariables<3>();
prog.AddCost(std::pow(x - 5, 2) + std::pow(2 * y + 1, 2));
prog.AddLinearConstraint(2 * (y-1) - 1.5 * x + l(0) - 0.5 * l(1) + l(2) == 0);
// TODO(hongkai.dai): write this linear complementary constraint in symbolic form.
Eigen::Matrix<double, 5, 5> M;
M << 3, -1, 0, 0, 0,
     -1, 0.5, 0, 0, 0,
     -1, -1, 0, 0, 0,
     0, 0, 1, 0, 0,
     0
}
}  // namespace
}  // namespace solvers
}  // namespace drake
