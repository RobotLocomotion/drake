#include "drake/solvers/scs_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(LinearProgramTest, Test0) {
  // Test a linear program with only equality constraint.
  // min x(0) + 2 * x(1)
  // s.t x(0) + x(1) = 2
  // The problem is unbounded.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddLinearCost(x(0) + 2 * x(1));
  prog.AddLinearConstraint(x(0) + x(1) == 2);
  ScsSolver solver;
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kUnbounded);
  }

  // Now add the constraint x(1) <= 1. The problem is
  // min x(0) + 2 * x(1)
  // s.t x(0) + x(1) = 2
  //     x(1) <= 1
  // the problem should still be unbounded.
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 1,
                                x(1));
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kUnbounded);
  }

  // The accuracy of SCS is not high, so we choose 1E-2 here, 10x the
  // epsilon defined in SCS.
  const double tol{1E-2};
  // Now add the constraint x(0) <= 5. The problem is
  // min x(0) + 2x(1)
  // s.t x(0) + x(1) = 2
  //     x(1) <= 1
  //     x(0) <= 5
  // the problem should be feasible. The optimal cost is -1, with x = (5, -3)
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 5,
                                x(0));
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
    EXPECT_NEAR(prog.GetOptimalCost(), -1, tol);
    const Eigen::Vector2d x_expected(5, -3);
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), x_expected, tol,
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
  prog.AddBoundingBoxConstraint(2, 6, x(0));
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
    EXPECT_NEAR(prog.GetOptimalCost(), 11, tol);
    const Eigen::Vector2d x_expected(2, 0);
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), x_expected, tol,
                                MatrixCompareType::absolute));
  }
}

GTEST_TEST(LinearProgramTest, Test1) {
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
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    SolutionResult sol_result = scs_solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kInfeasibleConstraints);
  }
}

GTEST_TEST(LinearProgramTest, Test2) {
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

  ScsSolver scs_solver;
  if (scs_solver.available()) {
    const double tol{1E-2};
    const SolutionResult sol_result = scs_solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
    EXPECT_NEAR(prog.GetOptimalCost(), 8, tol);
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), Eigen::Vector3d(1, 1, 1),
                                tol, MatrixCompareType::absolute));
  }
}

TEST_P(LinearProgramTest, TestLP) {
  ScsSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    SCSTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestInfeasible) {
  ScsSolver solver;
  if (solver.available()) {
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(prog_->GetOptimalCost(),
              MathematicalProgram::kGlobalInfeasibleCost);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestUnbounded) {
  ScsSolver solver;
  if (solver.available()) {
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kUnbounded);
    EXPECT_EQ(prog_->GetOptimalCost(), MathematicalProgram::kUnboundedCost);
  }
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    // SCS is inaccurate, choose a large tolerance.
    SolveAndCheckSolution(scs_solver, 1.3e-3);
  }
}

INSTANTIATE_TEST_CASE_P(SCSTest, TestEllipsoidsSeparation,
                        ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    // SCS is inaccurate, choose a large tolerance.
    SolveAndCheckSolution(scs_solver, 2E-4);
  }
}

INSTANTIATE_TEST_CASE_P(SCSTest, TestQPasSOCP,
                        ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    // SCS is inaccurate, choose a large tolerance.
    SolveAndCheckSolution(scs_solver, 2E-3);
  }
}

INSTANTIATE_TEST_CASE_P(
    SCSTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

TEST_P(QuadraticProgramTest, TestQP) {
  ScsSolver solver;
  if (solver.available()) {
    prob()->RunProblem(&solver);
  }
}

INSTANTIATE_TEST_CASE_P(
    ScsTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  ScsSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(TestSemidefiniteProgram, TrivialSDP) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    TestTrivialSDP(scs_solver, 2E-3);
  }
}

GTEST_TEST(TestSemidefiniteProgram, CommonLyapunov) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    FindCommonLyapunov(scs_solver, 5E-4);
  }
}

GTEST_TEST(TestSemidefiniteProgram, OuterEllipsoid) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    FindOuterEllipsoid(scs_solver, 1E-3);
  }
}

GTEST_TEST(TestSemidefiniteProgram, EigenvalueProblem) {
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    SolveEigenvalueProblem(scs_solver, 1E-3);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
