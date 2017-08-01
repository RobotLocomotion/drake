#include "drake/solvers/linear_system_solver.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {

namespace {
void TestLinearSystemExample(LinearSystemExample1* example) {
  example->prog()->Solve();
  CheckSolver(*(example->prog()), LinearSystemSolver::id());
  example->CheckSolution();
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

TEST_F(LinearSystemNoSolutionExample, testSolution) {
  auto result = prog_.Solve();
  EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
  CheckSolver(prog_, LinearSystemSolver::id());
}

GTEST_TEST(testLinearSystemSolver, linearMatrixEqualityExample) {
  LinearMatrixEqualityExample example{};
  example.prog()->Solve();
  CheckSolver(*(example.prog()), LinearSystemSolver::id());
  example.CheckSolution();
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
