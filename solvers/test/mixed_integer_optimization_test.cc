#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/test/add_solver_util.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
namespace {
void GetMixedIntegerLinearProgramSolvers(
    std::list<std::unique_ptr<SolverInterface>>* solvers) {
  AddSolverIfAvailable<GurobiSolver>(solvers);
  AddSolverIfAvailable<MosekSolver>(solvers);
}
}  // namespace

// Take the example from Gurobi manual
// https://www.gurobi.com/documentation/8.0/examples/mip1_c_c.html
// min -x(0) - x(1) - 2*x(2)
// s.t  -inf <= x(0) + 2x(1) + 3*x(2) <= 4
//         1 <= x(0) + x(1)           <= inf
//         x(0), x(1), x(2) are binary
// The optimal solution is x(0) = 1, x(1) = 0, x(2) = 1;
GTEST_TEST(TestMixedIntegerOptimization, TestMixedIntegerLinearProgram1) {
  std::list<std::unique_ptr<SolverInterface>> solvers;
  GetMixedIntegerLinearProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    MathematicalProgram prog;
    auto x = prog.NewBinaryVariables(3, "x");
    Eigen::Vector3d c(-1, -1, -2);
    prog.AddLinearCost(c, x);
    Eigen::RowVector3d a1(1, 2, 3);
    prog.AddLinearConstraint(a1, -std::numeric_limits<double>::infinity(), 4,
                             x);
    Eigen::RowVector2d a2(1, 1);
    prog.AddLinearConstraint(a2, 1, std::numeric_limits<double>::infinity(),
                             x.head<2>());

    const MathematicalProgramResult result = RunSolver(prog, *solver);

    Eigen::Vector3d x_expected(1, 0, 1);
    const auto& x_value = result.GetSolution(x);
    EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                MatrixCompareType::absolute));
  }
}

// Take the example from
// http://www.cs.cmu.edu/~zkolter/course/15-780-s14/mip.pdf
// min 2*z1 + z2 - 2*z3
// s.t 0.7*z1 + 0.5*z2+z3 >= 1.8
// z1, z2, z3 are integers.
// The optimal solution is (1, 1, 1)
GTEST_TEST(TestMixedIntegerOptimization, TestMixedIntegerLinearProgram2) {
  std::list<std::unique_ptr<SolverInterface>> solvers;
  GetMixedIntegerLinearProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    MathematicalProgram prog;
    auto x = prog.NewBinaryVariables<3>("x");
    Eigen::Vector3d c(2, 1, -2);
    prog.AddLinearCost(c, x);
    Eigen::RowVector3d a1(0.7, 0.5, 1);
    prog.AddLinearConstraint(a1, 1.8, std::numeric_limits<double>::infinity(),
                             x);

    const MathematicalProgramResult result = RunSolver(prog, *solver);

    Eigen::Vector3d x_expected(1, 1, 1);
    const auto& x_value = result.GetSolution(x);
    EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                MatrixCompareType::absolute));
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
