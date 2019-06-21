#include "drake/solvers/csdp_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/csdp_test_examples.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(TestSemidefiniteProgram, SolveSDPwithOverlappingVariables) {
  CsdpSolver solver;
  if (solver.available()) {
    test::SolveSDPwithOverlappingVariables(solver, 1E-7);
  }
}

TEST_F(CsdpDocExample, Solve) {
  CsdpSolver solver;
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(*prog_, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    const double tol = 5E-7;
    EXPECT_NEAR(result.get_optimal_cost(), -2.75, tol);
    Eigen::Matrix2d X1_expected;
    X1_expected << 0.125, 0.125, 0.125, 0.125;
    EXPECT_TRUE(CompareMatrices(result.GetSolution(X1_), X1_expected, tol));
    Eigen::Matrix3d X2_expected;
    X2_expected.setZero();
    X2_expected(0, 0) = 2.0 / 3;
    EXPECT_TRUE(CompareMatrices(result.GetSolution(X2_), X2_expected, tol));
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(y_), Eigen::Vector2d::Zero(), tol));
    const CsdpSolverDetails& solver_details =
        result.get_solver_details<CsdpSolver>();
    EXPECT_EQ(solver_details.return_code, 0);
    EXPECT_NEAR(solver_details.primal_objective, 2.75, tol);
    EXPECT_NEAR(solver_details.dual_objective, 2.75, tol);
    EXPECT_TRUE(
        CompareMatrices(solver_details.y_val, Eigen::Vector2d(0.75, 1), tol));
    Eigen::Matrix<double, 7, 7> Z_expected;
    Z_expected.setZero();
    Z_expected(0, 0) = 0.25;
    Z_expected(0, 1) = -0.25;
    Z_expected(1, 0) = -0.25;
    Z_expected(1, 1) = 0.25;
    Z_expected(3, 3) = 2.0;
    Z_expected(4, 4) = 2.0;
    Z_expected(5, 5) = 0.75;
    Z_expected(6, 6) = 1.0;
    EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(solver_details.Z_val),
                                Z_expected, tol));
  }
}

TEST_F(TrivialSDP1, Solve) {
  CsdpSolver solver;
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(*prog_, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-7;
    EXPECT_NEAR(result.get_optimal_cost(), -2.0 / 3.0, tol);
    const Eigen::Matrix3d X1_expected = Eigen::Matrix3d::Constant(1.0 / 3);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(X1_), X1_expected, tol));
  }
}
}  // namespace

namespace test {
TEST_F(InfeasibleLinearProgramTest0, TestInfeasible) {
  CsdpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestUnbounded) {
  CsdpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
