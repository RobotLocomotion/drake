#include "drake/solvers/clarabel_solver.h"

#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/exponential_cone_program_examples.h"
#include "drake/solvers/test/l2norm_cost_examples.h"
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

using testing::HasSubstr;

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

GTEST_TEST(QPtest, TestQuadraticCostVariableOrder) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestQuadraticCostVariableOrder(solver);
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

GTEST_TEST(TestSOCP, TestSocpDuplicatedVariable3) {
  ClarabelSolver solver;
  TestSocpDuplicatedVariable3(solver, std::nullopt, 1E-4);
}

GTEST_TEST(TestL2NormCost, ShortestDistanceToThreePoints) {
  ClarabelSolver solver;
  ShortestDistanceToThreePoints tester{};
  tester.CheckSolution(solver);
}

GTEST_TEST(TestL2NormCost, ShortestDistanceFromCylinderToPoint) {
  ClarabelSolver solver;
  ShortestDistanceFromCylinderToPoint tester{};
  tester.CheckSolution(solver);
}

GTEST_TEST(TestL2NormCost, ShortestDistanceFromPlaneToTwoPoints) {
  ClarabelSolver solver;
  ShortestDistanceFromPlaneToTwoPoints tester{};
  tester.CheckSolution(solver, std::nullopt, 5E-4);
}

GTEST_TEST(TestSemidefiniteProgram, TrivialSDP) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestTrivialSDP(solver, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, CommonLyapunov) {
  ClarabelSolver solver;
  if (solver.available()) {
    FindCommonLyapunov(solver, {}, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, OuterEllipsoid) {
  ClarabelSolver solver;
  if (solver.available()) {
    FindOuterEllipsoid(solver, {}, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, EigenvalueProblem) {
  ClarabelSolver solver;
  if (solver.available()) {
    SolveEigenvalueProblem(solver, {}, kTol, /*check_dual=*/true);
  }
}

GTEST_TEST(TestSemidefiniteProgram, SolveSDPwithQuadraticCosts) {
  ClarabelSolver solver;
  if (solver.available()) {
    SolveSDPwithQuadraticCosts(solver, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, TestSDPDualSolution1) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestSDPDualSolution1(solver, kTol, /*complemantarity_tol=*/1E-5);
  }
}

GTEST_TEST(TestSemidefiniteProgram, SolveSDPwithSecondOrderConeExample1) {
  ClarabelSolver solver;
  if (solver.available()) {
    SolveSDPwithSecondOrderConeExample1(solver, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, SolveSDPwithSecondOrderConeExample2) {
  ClarabelSolver solver;
  if (solver.available()) {
    SolveSDPwithSecondOrderConeExample2(solver, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, SolveSDPwithOverlappingVariables) {
  ClarabelSolver solver;
  if (solver.available()) {
    SolveSDPwithOverlappingVariables(solver, kTol);
  }
}

GTEST_TEST(TestSemidefiniteProgram, TestTrivial1x1SDP) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestTrivial1x1SDP(solver, 1E-5, /*check_dual=*/true, /*dual_tol=*/1E-5);
  }
}

GTEST_TEST(TestSemidefiniteProgram, TestTrivial2x2SDP) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestTrivial2x2SDP(solver, 1E-5, /*check_dual=*/true, /*dual_tol=*/1E-5);
  }
}

GTEST_TEST(TestSemidefiniteProgram, Test1x1with3x3SDP) {
  ClarabelSolver solver;
  if (solver.available()) {
    Test1x1with3x3SDP(solver, 1E-4, /*check_dual=*/true, /*dual_tol=*/1E-4);
  }
}

GTEST_TEST(TestSemidefiniteProgram, Test2x2with3x3SDP) {
  ClarabelSolver solver;
  if (solver.available()) {
    Test2x2with3x3SDP(solver, 1E-3, /*check_dual=*/true, /*dual_tol*/ 1E-2);
  }
}

GTEST_TEST(TestSemidefiniteProgram, TestTrivial1x1LMI) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestTrivial1x1LMI(solver, 1E-5, /*check_dual=*/true, /*dual_tol=*/1E-7);
  }
}

GTEST_TEST(TestSemidefiniteProgram, Test2X2LMI) {
  ClarabelSolver solver;
  if (solver.available()) {
    Test2x2LMI(solver, 1E-7, /*check_dual=*/true, /*dual_tol=*/1E-7);
  }
}

GTEST_TEST(TestSemidefiniteProgram, TestHankel) {
  ClarabelSolver solver;
  if (solver.available()) {
    TestHankel(solver, 1E-5, /*check_dual=*/true, /*dual_tol=*/1E-5);
  }
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
  ClarabelSolver solver;
  if (solver.available()) {
    MatrixLogDeterminantLower(solver, kTol);
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

GTEST_TEST(TestOptions, MaxThreads) {
  SimpleSos1 dut;
  const int kMaxThreadsValue = 4;
  SolverOptions solver_options;
  solver_options.SetOption(CommonSolverOption::kMaxThreads, kMaxThreadsValue);
  ClarabelSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(dut.prog(), std::nullopt, solver_options);
    EXPECT_TRUE(result.is_success());
    // We can't really check that the threads setting was obeyed -- it doesn't
    // even appear in the logs for eyeball inspection.
  }
}

GTEST_TEST(TestOptions, StandaloneReproduction) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>("x");
  prog.AddLinearEqualityConstraint(x(0) + x(1) == 1);
  prog.AddLinearConstraint(x(0) + x(1) + x(2) >= 0);
  prog.AddLorentzConeConstraint(Vector2<symbolic::Expression>(x(0), x(1)));
  prog.AddExponentialConeConstraint(
      Vector3<symbolic::Expression>(x(2), x(0), x(1)));
  const auto Y = prog.NewSymmetricContinuousVariables<3>("Y");
  prog.AddPositiveSemidefiniteConstraint(Y);

  ClarabelSolver solver;
  if (solver.available()) {
    SolverOptions solver_options;
    const std::string repro_file_name = temp_directory() + "/reproduction.py";
    solver_options.SetOption(
        CommonSolverOption::kStandaloneReproductionFileName, repro_file_name);
    solver.Solve(prog, std::nullopt, solver_options);

    // Read in the reproduction file.
    std::ifstream input_stream(repro_file_name);
    ASSERT_TRUE(input_stream.is_open());
    std::stringstream buffer;
    buffer << input_stream.rdbuf();
    std::string repro_str = buffer.str();

    EXPECT_THAT(repro_str, HasSubstr("import clarabel"));
    EXPECT_THAT(repro_str, HasSubstr("ZeroConeT"));
    EXPECT_THAT(repro_str, HasSubstr("NonnegativeConeT"));
    EXPECT_THAT(repro_str, HasSubstr("SecondOrderConeT"));
    EXPECT_THAT(repro_str, HasSubstr("PSDTriangleConeT"));
    EXPECT_THAT(repro_str, HasSubstr("ExponentialConeT"));
    EXPECT_THAT(repro_str, HasSubstr("solve"));
  }
}

// Ensure that when we have no linear constraints, we do not generate programs
// with empty Zero nor Nonnegative cones.
GTEST_TEST(TestOptions, EmptyCones) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>("x");
  prog.AddLorentzConeConstraint(Vector2<symbolic::Expression>(x(0), x(1)));
  prog.AddExponentialConeConstraint(
      Vector3<symbolic::Expression>(x(2), x(0), x(1)));
  const auto Y = prog.NewSymmetricContinuousVariables<3>("Y");
  prog.AddPositiveSemidefiniteConstraint(Y);

  ClarabelSolver solver;
  if (solver.available()) {
    SolverOptions solver_options;
    const std::string repro_file_name = temp_directory() + "/reproduction.py";
    solver_options.SetOption(
        CommonSolverOption::kStandaloneReproductionFileName, repro_file_name);
    solver.Solve(prog, std::nullopt, solver_options);

    // Read in the reproduction file.
    std::ifstream input_stream(repro_file_name);
    ASSERT_TRUE(input_stream.is_open());
    std::stringstream buffer;
    buffer << input_stream.rdbuf();
    std::string repro_str = buffer.str();

    EXPECT_THAT(repro_str, HasSubstr("import clarabel"));
    EXPECT_THAT(repro_str, Not(HasSubstr("ZeroConeT")));
    EXPECT_THAT(repro_str, Not(HasSubstr("NonnegativeConeT")));
    EXPECT_THAT(repro_str, HasSubstr("SecondOrderConeT"));
    EXPECT_THAT(repro_str, HasSubstr("PSDTriangleConeT"));
    EXPECT_THAT(repro_str, HasSubstr("ExponentialConeT"));
    EXPECT_THAT(repro_str, HasSubstr("solve"));
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
        ".*not recognized.*bad_unrecognized.*");
  }
}

GTEST_TEST(TestZeroStepSize, ZeroStepSize) {
  // This is a program configuration that causes Clarabel to crash (and hence
  // crash Drake) in version 0.6.0. In version 0.7.1, this configuration causes
  // the solver to report InsufficientProgress.
  ClarabelSolver solver;
  MathematicalProgram prog;
  const auto y = prog.NewContinuousVariables<2>("y");
  MatrixX<symbolic::Expression> mat(2, 2);
  mat << y(0, 0), 0.5, 0.5, y(1);
  prog.AddLinearMatrixInequalityConstraint(mat);
  prog.AddLogDeterminantLowerBoundConstraint(mat, 1);
  prog.AddLinearCost(-y(0));
  SolverOptions options;
  options.SetOption(solver.id(), "max_step_fraction", 1e-10);
  options.SetOption(CommonSolverOption::kPrintToConsole, true);
  if (solver.available()) {
    auto result = solver.Solve(prog, std::nullopt, options);
    // The program has cost unbounded above and so the dual is infeasible, but
    // the step size fraction forces the solver to make insufficient progress.
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kSolverSpecificError);
    EXPECT_EQ(result.get_solver_details<ClarabelSolver>().status,
              "InsufficientProgress");
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
