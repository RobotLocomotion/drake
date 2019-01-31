#include "drake/solvers/linear_system_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {

namespace {
void TestLinearSystemExample(LinearSystemExample1* example) {
  const MathematicalProgramResult result = Solve(*(example->prog()));
  EXPECT_TRUE(result.is_success());
  example->CheckSolution(result);
}
}  // namespace

GTEST_TEST(testLinearSystemSolver, trivialExample) {
  LinearSystemExample1 example1{};
  TestLinearSystemExample(&example1);

  LinearSystemExample2 example2{};
  TestLinearSystemExample(&example1);

  LinearSystemExample3 example3{};
  TestLinearSystemExample(&example1);
}

/**
 * Simple linear system without a solution
 * 3 * x = 1
 * 2 * x + y = 2
 * x - y = 0
 */
GTEST_TEST(testLinearSystemSolver, InfeasibleProblem) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(3 * x(0) == 1 && 2 * x(0) + x(1) == 2 &&
        x(0) - x(1) == 0);

  const MathematicalProgramResult result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  // The solution should minimize the error ||b - A * x||₂
  // x_expected is computed as (Aᵀ*A)⁻¹*(Aᵀ*b)
  Eigen::Vector2d x_expected(12.0 / 27, 21.0 / 27);
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, 1E-12,
                              MatrixCompareType::absolute));
  EXPECT_EQ(result.get_optimal_cost(),
            MathematicalProgram::kGlobalInfeasibleCost);
}

/**
 * Under-determined linear system
 * 3 * x + y = 1
 * x + z = 2
 */
GTEST_TEST(testLinearSystemSolver, UnderDeterminedProblem) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddLinearConstraint(3 * x(0) + x(1) == 1 && x(0) + x(2) == 2);

  const MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  // The solution should minimize the norm ||x||₂
  // x_expected is computed as the solution to
  // [2*I -Aᵀ] * [x] = [0]
  // [ A   0 ]   [λ]   [b]
  Eigen::Vector3d x_expected(5.0 / 11, -4.0 / 11, 17.0 / 11);
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, 1E-12,
                              MatrixCompareType::absolute));
}

GTEST_TEST(testLinearSystemSolver, linearMatrixEqualityExample) {
  LinearMatrixEqualityExample example{};
  const auto result = Solve(*(example.prog()));
  EXPECT_EQ(result.get_solver_id(), LinearSystemSolver::id());
  example.CheckSolution(result);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
