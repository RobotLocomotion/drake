#include "drake/solvers/clarabel_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/exponential_cone_program_examples.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"
#include "drake/solvers/test/sos_examples.h"

namespace drake {
namespace solvers {
namespace test {
const double kTol = 1E-5;
GTEST_TEST(LinearProgramTest, TestGeneralLP) {
  // Test a linear program with only equality constraint.
  // min x(0) + 2 * x(1)
  // s.t x(0) + x(1) = 2
  // The problem is unbounded.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddLinearCost(x(0) + 2 * x(1));
  prog.AddLinearConstraint(x(0) + x(1) == 2);
  ClarabelSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
  }

  // Now add the constraint x(1) <= 1. The problem is
  // min x(0) + 2 * x(1)
  // s.t x(0) + x(1) = 2
  //     x(1) <= 1
  // the problem should still be unbounded.
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 1,
                                x(1));
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
  }

  // Now add the constraint x(0) <= 5. The problem is
  // min x(0) + 2x(1)
  // s.t x(0) + x(1) = 2
  //     x(1) <= 1
  //     x(0) <= 5
  // the problem should be feasible. The optimal cost is -1, with x = (5, -3)
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 5,
                                x(0));
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    EXPECT_NEAR(result.get_optimal_cost(), -1, kTol);
    const Eigen::Vector2d x_expected(5, -3);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, kTol,
                                MatrixCompareType::absolute));
  }

  // Now change the cost to 3x(0) - x(1) + 5, and add the constraint 2 <= x(0)
  // The problem is
  // min 3x(0) - x(1) + 5
  // s.t x(0) + x(1) = 2
  //     2 <= x(0) <= 5
  //          x(1) <= 1
  // The optimal cost is 11, the optimal solution is x = (2, 0)
  prog.AddLinearCost(2 * x(0) - 3 * x(1) + 5);
  prog.AddBoundingBoxConstraint(
      2, 6 /* this upper bound = 6 is intentionally redundant.*/, x(0));
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    EXPECT_NEAR(result.get_optimal_cost(), 11, kTol);
    const Eigen::Vector2d x_expected(2, 0);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, kTol,
                                MatrixCompareType::absolute));
  }
}

GTEST_TEST(LinearProgramTest, TestInfeasibleEqualityOnlyLP) {
  // Test a linear program with only equality constraints
  // min x(0) + 2 * x(1)
  // s.t x(0) + x(1) = 1
  //     2x(0) + x(1) = 2
  //     x(0) - 2x(1) = 3
  // This problem is infeasible.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddLinearCost(x(0) + 2 * x(1));
  prog.AddLinearEqualityConstraint(x(0) + x(1) == 1 && 2 * x(0) + x(1) == 2);
  prog.AddLinearEqualityConstraint(x(0) - 2 * x(1) == 3);
  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    auto result = clarabel_solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
  }
}

GTEST_TEST(LinearProgramTest, TestAllLPConstraintTypes) {
  // Test a linear program with bounding box, linear equality and inequality
  // constraints
  // min x(0) + 2 * x(1) + 3 * x(2) + 2
  // s.t  x(0) + x(1) = 2
  //      x(0) + 2x(2) = 3
  //      -2 <= x(0) + 4x(1) <= 10
  //      -5 <= x(1) + 2x(2) <= 9
  //      -x(0) + 2x(2) <= 7
  //      -x(1) + 3x(2) >= -10
  //       x(0) <= 10
  //      1 <= x(2) <= 9
  // The optimal cost is 8, with x = (1, 1, 1)
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  prog.AddLinearCost(x(0) + 2 * x(1) + 3 * x(2) + 2);
  prog.AddLinearEqualityConstraint(x(0) + x(1) == 2 && x(0) + 2 * x(2) == 3);
  Eigen::Matrix<double, 3, 3> A;
  // clang-format off
  A << 1, 4, 0,
       0, 1, 2,
       -1, 0, 2;
  // clang-format on
  prog.AddLinearConstraint(
      A, Eigen::Vector3d(-2, -5, -std::numeric_limits<double>::infinity()),
      Eigen::Vector3d(10, 9, 7), x);
  prog.AddLinearConstraint(-x(1) + 3 * x(2) >= -10);
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 10,
                                x(0));
  prog.AddBoundingBoxConstraint(1, 9, x(2));

  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    auto result = clarabel_solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    EXPECT_NEAR(result.get_optimal_cost(), 8, kTol);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), Eigen::Vector3d(1, 1, 1),
                                kTol, MatrixCompareType::absolute));
  }
}

TEST_P(LinearProgramTest, TestLP) {
  ClarabelSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    ClarabelTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestInfeasible) {
  ClarabelSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestUnbounded) {
  ClarabelSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
    EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
    // Make sure that we have written Clarabel's variable value into `result`.
    EXPECT_TRUE(result.GetSolution(prog_->decision_variables())
                    .array()
                    .isFinite()
                    .all());
  }
}

TEST_F(DuplicatedVariableLinearProgramTest1, Test) {
  ClarabelSolver solver;
  if (solver.is_available()) {
    CheckSolution(solver, std::nullopt, kTol);
  }
}

GTEST_TEST(TestLPDualSolution1, Test) {
  ClarabelSolver solver;
  if (solver.is_available()) {
    TestLPDualSolution1(solver, kTol);
  }
}

GTEST_TEST(TestLPDualSolution2, Test) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestLPDualSolution2(solver, kTol);
  }
}

GTEST_TEST(TestLPDualSolution3, Test) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestLPDualSolution3(solver, kTol);
  }
}

GTEST_TEST(TestLPDualSolution4, Test) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestLPDualSolution4(solver, kTol);
  }
}

GTEST_TEST(TestLPDualSolution5, Test) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestLPDualSolution5(solver, kTol);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  ClarabelSolver solver;
  if (solver.available()) {
    prob()->RunProblem(&solver);
  }
}

INSTANTIATE_TEST_SUITE_P(
    ClarabelTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(TestDuplicatedVariableQuadraticProgram, Test) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestDuplicatedVariableQuadraticProgram(solver, 1E-5);
  }
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    SolveAndCheckSolution(clarabel_solver, {}, kTol);
  }
}

INSTANTIATE_TEST_SUITE_P(
    ClarabelTest, TestEllipsoidsSeparation,
    ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    SolveAndCheckSolution(clarabel_solver, kTol);
  }
}

INSTANTIATE_TEST_SUITE_P(ClarabelTest, TestQPasSOCP,
                         ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    SolveAndCheckSolution(clarabel_solver, {}, 3E-4);
  }
}

INSTANTIATE_TEST_SUITE_P(
    ClarabelTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem1) {
  MaximizeGeometricMeanTrivialProblem1 prob;
  ClarabelSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    // Practically I observe Clarabel requires looser tolerance for this test. I
    // don't know why.
    prob.CheckSolution(result, 3 * kTol);
  }
}

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem2) {
  MaximizeGeometricMeanTrivialProblem2 prob;
  ClarabelSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    prob.CheckSolution(result, kTol);
  }
}

GTEST_TEST(TestSOCP, SmallestEllipsoidCoveringProblem) {
  ClarabelSolver solver;
  SolveAndCheckSmallestEllipsoidCoveringProblems(solver, {}, kTol);
}

GTEST_TEST(TestSOCP, LorentzConeDual) {
  ClarabelSolver solver;
  SolverOptions solver_options;
  TestSocpDualSolution1(solver, solver_options, kTol);
}

GTEST_TEST(TestSOCP, RotatedLorentzConeDual) {
  ClarabelSolver solver;
  SolverOptions solver_options;
  TestSocpDualSolution2(solver, solver_options, kTol);
}

GTEST_TEST(TestSOCP, TestSocpDuplicatedVariable1) {
  ClarabelSolver solver;
  TestSocpDuplicatedVariable1(solver, std::nullopt, 1E-6);
}

GTEST_TEST(TestSOCP, TestSocpDuplicatedVariable2) {
  ClarabelSolver solver;
  TestSocpDuplicatedVariable2(solver, std::nullopt, 1E-6);
}

GTEST_TEST(TestExponentialConeProgram, ExponentialConeTrivialExample) {
  ClarabelSolver solver;
  if (solver.available()) {
    // Currently we don't support retrieving dual solution for exponential cone
    // constraints from Clarabel yet.
    ExponentialConeTrivialExample(solver, 2E-4, false);
  }
}

GTEST_TEST(TestExponentialConeProgram, MinimizeKLDivengence) {
  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    MinimizeKLDivergence(clarabel_solver, 1E-4);
  }
}

GTEST_TEST(TestExponentialConeProgram, MinimalEllipsoidConveringPoints) {
  ClarabelSolver clarabel_solver;
  if (clarabel_solver.available()) {
    MinimalEllipsoidCoveringPoints(clarabel_solver, 1E-4);
  }
}

GTEST_TEST(TestExponentialConeProgram, MatrixLogDeterminantLower) {
  ClarabelSolver scs_solver;
  if (scs_solver.available()) {
    MatrixLogDeterminantLower(scs_solver, kTol);
  }
}
GTEST_TEST(TestSos, UnivariateQuarticSos) {
  UnivariateQuarticSos dut;
  ClarabelSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(dut.prog());
    dut.CheckResult(result, kTol);
  }
}

GTEST_TEST(TestSos, BivariateQuarticSos) {
  BivariateQuarticSos dut;
  ClarabelSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(dut.prog());
    dut.CheckResult(result, kTol);
  }
}

GTEST_TEST(TestSos, SimpleSos1) {
  SimpleSos1 dut;
  ClarabelSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(dut.prog());
    dut.CheckResult(result, kTol);
  }
}

GTEST_TEST(TestSos, MotzkinPolynomial) {
  MotzkinPolynomial dut;
  ClarabelSolver solver;
  if (solver.is_available()) {
    const auto result = solver.Solve(dut.prog());
    dut.CheckResult(result, kTol);
  }
}

GTEST_TEST(TestSos, UnivariateNonnegative1) {
  UnivariateNonnegative1 dut;
  ClarabelSolver solver;
  if (solver.is_available()) {
    const auto result = solver.Solve(dut.prog());
    dut.CheckResult(result, kTol);
  }
}

GTEST_TEST(TestOptions, SetMaxIter) {
  SimpleSos1 dut;
  ClarabelSolver solver;
  if (solver.available()) {
    SolverOptions solver_options;
    auto result = solver.Solve(dut.prog(), std::nullopt, solver_options);
    EXPECT_TRUE(result.is_success());
    ASSERT_GT(result.get_solver_details<ClarabelSolver>().iterations, 1);
    // Now change the max iteration to 1.
    solver_options.SetOption(solver.id(), "max_iter", 1);
    result = solver.Solve(dut.prog(), std::nullopt, solver_options);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
  }
}

GTEST_TEST(TestOptions, unrecognized) {
  SimpleSos1 dut;
  ClarabelSolver solver;
  if (solver.available()) {
    SolverOptions solver_options;
    solver_options.SetOption(solver.id(), "bad_unrecognized", 1);
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.Solve(dut.prog(), std::nullopt, solver_options),
        ".*unrecognized solver options bad_unrecognized.*");
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
