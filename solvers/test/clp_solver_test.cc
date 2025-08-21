#include "drake/solvers/clp_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/quadratic_program_examples.h"

#if defined(__APPLE__)
constexpr bool kApple = true;
#else
constexpr bool kApple = false;
#endif

namespace drake {
namespace solvers {
namespace test {
TEST_P(LinearProgramTest, TestLP) {
  ClpSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    ClpTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestInfeasible) {
  ClpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    auto& details = result.get_solver_details<ClpSolver>();
    if (details.clp_version == "1.17.8") {
      // This version of CLP is buggy and reports the wrong answer.
      return;
    }
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_TRUE(std::isinf(result.get_optimal_cost()));
    EXPECT_GT(result.get_optimal_cost(), 0.);
    // This code is defined in ClpModel::status()
    const int CLP_INFEASIBLE = 1;
    EXPECT_EQ(details.status, CLP_INFEASIBLE);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestUnbounded) {
  ClpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
    EXPECT_TRUE(std::isinf(result.get_optimal_cost()));
    EXPECT_LT(result.get_optimal_cost(), 0.);
    // This code is defined in ClpModel::status()
    const int CLP_UNBOUNDED = 2;
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, CLP_UNBOUNDED);
  }
}

TEST_F(DuplicatedVariableLinearProgramTest1, Test) {
  ClpSolver solver;
  if (solver.available()) {
    CheckSolution(solver);
  }
}

GTEST_TEST(TestDual, DualSolution1) {
  ClpSolver solver;
  if (solver.available()) {
    TestLPDualSolution1(solver);
  }
}

GTEST_TEST(TestDual, DualSolution2) {
  ClpSolver solver;
  if (solver.available()) {
    TestLPDualSolution2(solver);
  }
}

GTEST_TEST(TestDual, DualSolution3) {
  ClpSolver solver;
  if (solver.available()) {
    TestLPDualSolution3(solver);
  }
}

GTEST_TEST(TestDual, DualSolution4) {
  ClpSolver solver;
  if (solver.available()) {
    TestLPDualSolution4(solver);
  }
}

GTEST_TEST(TestDual, DualSolution5) {
  ClpSolver solver;
  TestLPDualSolution5(solver);
}

GTEST_TEST(QPtest, TestUnconstrainedQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  prog.AddQuadraticCost(x(0) * x(0));

  ClpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-10;
    EXPECT_NEAR(result.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, 0);
  }

  // Add additional quadratic costs
  prog.AddQuadraticCost((x(1) + x(2) - 2) * (x(1) + x(2) - 2));
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-10;
    EXPECT_NEAR(result.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(result.GetSolution(x(1)) + result.GetSolution(x(2)), 2, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, 0);
  }

  // Add linear costs.
  prog.AddLinearCost(4 * x(0) + 5);
  // Now the cost is (x₀ + 2)² + (x₁ + x₂ - 2)² + 1
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-10;
    EXPECT_NEAR(result.GetSolution(x(0)), -2, tol);
    EXPECT_NEAR(result.GetSolution(x(1)) + result.GetSolution(x(2)), 2, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 1, tol);
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, 0);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  ClpSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    ClpTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  ClpSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(QPtest, TestQuadraticCostVariableOrder) {
  ClpSolver solver;
  if (solver.available()) {
    TestQuadraticCostVariableOrder(solver);
  }
}

GTEST_TEST(QPtest, TestInfeasible) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 2);
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddLinearConstraint(x(1) >= 2);

  ClpSolver solver;
  // The program is infeasible.
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, 1);
  }
}

GTEST_TEST(ClpSolverTest, Version) {
  ClpSolver solver;
  MathematicalProgram prog;
  MathematicalProgramResult result = solver.Solve(prog);
  if (solver.available()) {
    auto& details = result.get_solver_details<ClpSolver>();
    EXPECT_NE(details.clp_version, "");
  }
}

GTEST_TEST(ClpSolverTest, DualSolution1) {
  // Test GetDualSolution().
  ClpSolver solver;
  TestQPDualSolution1(solver);
}

GTEST_TEST(ClpSolverTest, DualSolution2) {
  // Test GetDualSolution().
  // This QP has non-zero dual solution for linear inequality constraint.
  ClpSolver solver;
  TestQPDualSolution2(solver);
}

GTEST_TEST(ClpSolverTest, DualSolution3) {
  // Test GetDualSolution().
  // This QP has non-zero dual solution for the bounding box constraint.
  ClpSolver solver;
  TestQPDualSolution3(solver, 0.0, 1.0e-4);
}

GTEST_TEST(ClpSolverTest, EqualityConstrainedQPDualSolution1) {
  ClpSolver solver;
  TestEqualityConstrainedQPDualSolution1(solver);
}

GTEST_TEST(ClpSolverTest, EqualityConstrainedQPDualSolution2) {
  ClpSolver solver;
  TestEqualityConstrainedQPDualSolution2(solver);
}

GTEST_TEST(ClpSolverTest, TestNonconvexQP) {
  ClpSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, true);
  }
}

// This is a code coverage test, not a functional test.  If the code that
// handles verbosity options has a segfault or always throws an exception,
// then this would catch it.
GTEST_TEST(ClpSolverTest, TestVerbosity) {
  LinearProgram0 example(CostForm::kSymbolic, ConstraintForm::kSymbolic);
  const MathematicalProgram& prog = *example.prog();

  SolverOptions options;
  options.SetOption(CommonSolverOption::kPrintToConsole, 1);

  ClpSolver solver;
  if (solver.available()) {
    // This will print stuff to the console, but we don't have any
    // easy way to check that.
    solver.Solve(prog, {}, options);
  }
}
GTEST_TEST(ClpSolverTest, TestNumericalScaling) {
  ClpSolver solver;
  TestLPPoorScaling1(solver);
  TestLPPoorScaling2(solver);
  // Try another scaling option. Set scaling equal to 2. Somehow in CLP with
  // this scaling mode, the problem is not solved successfully.
  SolverOptions solver_options;
  solver_options.SetOption(ClpSolver::id(), "scaling", 2);
  TestLPPoorScaling1(solver, false, 1E-14, solver_options);
  TestLPPoorScaling2(solver, false, 1E-4, solver_options);
}

// The following simple QP is feasible, but CLP cannot currently solve it. Once
// CLP can solve this QP successfully, we can remove the warning in the
// ConstructClpModel function (in clp_solver.cc). See #22985 for details.
GTEST_TEST(ClpSolverTest, QuadraticProgram22985) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  prog.AddLinearConstraint(x[0] + x[1] == 1);
  prog.AddBoundingBoxConstraint(0, 1, x[1]);
  prog.AddQuadraticErrorCost(1, Eigen::Vector2d(-0.1, 0), x);

  ClpSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.is_success(), kApple);  // CLP succeeds on macOS only.
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
