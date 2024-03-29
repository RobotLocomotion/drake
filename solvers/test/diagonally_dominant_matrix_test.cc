#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
GTEST_TEST(DiagonallyDominantMatrixConstraint, ReturnYTest) {
  // Test the returned variables Y.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<5>();
  auto Y = prog.AddPositiveDiagonallyDominantMatrixConstraint(
      X.cast<symbolic::Expression>());
  EXPECT_EQ(Y.rows(), 5);
  EXPECT_EQ(Y.cols(), 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (i != j) {
        EXPECT_EQ(Y(i, j), Y(j, i));
      } else {
        EXPECT_EQ(Y(i, i), X(i, i));
      }
    }
  }
}

GTEST_TEST(DiagonallyDominantMatrixConstraint, FeasibilityCheck) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<2>();
  auto Y = prog.AddPositiveDiagonallyDominantMatrixConstraint(
      X.cast<symbolic::Expression>());
  EXPECT_EQ(Y(0, 1), Y(1, 0));
  EXPECT_EQ(Y(0, 0), X(0, 0));
  EXPECT_EQ(Y(1, 1), X(1, 1));

  auto X_constraint = prog.AddBoundingBoxConstraint(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      VectorDecisionVariable<3>(X(0, 0), X(0, 1), X(1, 1)));

  auto set_X_value = [&X_constraint](const Eigen::Vector3d& x_upper_triangle) {
    X_constraint.evaluator()->UpdateLowerBound(x_upper_triangle);
    X_constraint.evaluator()->UpdateUpperBound(x_upper_triangle);
  };

  // [1 0.9;0.9 2] is diagonally dominant
  set_X_value(Eigen::Vector3d(1, 0.9, 2));
  MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // [1 -0.9; -0.9 2] is diagonally dominant
  set_X_value(Eigen::Vector3d(1, -0.9, 2));
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // [1 1.1; 1.1 2] is not diagonally dominant
  set_X_value(Eigen::Vector3d(1, 1.1, 2));
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_TRUE(
      result.get_solution_result() == SolutionResult::kInfeasibleConstraints ||
      result.get_solution_result() == SolutionResult::kInfeasibleOrUnbounded);
  // [1 -1.1; -1.1 2] is not diagonally dominant
  set_X_value(Eigen::Vector3d(1, -1.1, 2));
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_TRUE(
      result.get_solution_result() == SolutionResult::kInfeasibleConstraints ||
      result.get_solution_result() == SolutionResult::kInfeasibleOrUnbounded);
}

GTEST_TEST(DiagonallyDominantMatrixConstraint, three_by_three_vertices) {
  // I can manually compute the polytope of (a, b, c) to make
  //     [1 a b]
  // A = [a 2 c]
  //     [b c 3]
  // to be a diagonally dominant matrix. The vertices of the polytope are
  // (0, ±1, 0), (±1, 0, 0), (0, ±1, ±2), (±1, 0, ±1), (0, 0, ±2)
  // By optimizing the LP
  // min nᵀ* (a, b, c)
  // s.t   A is diagonally dominant
  // with different vector n, we can recover the vertices of the polytope as
  // the optimal solution to this LP.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  prog.AddBoundingBoxConstraint(Eigen::Vector3d(1, 2, 3),
                                Eigen::Vector3d(1, 2, 3), X.diagonal());
  prog.AddPositiveDiagonallyDominantMatrixConstraint(
      X.cast<symbolic::Expression>());

  auto cost =
      prog.AddLinearCost(Eigen::Vector3d::Zero(), 0,
                         VectorDecisionVariable<3>(X(0, 1), X(0, 2), X(1, 2)));

  auto solve_and_check =
      [&prog, &X](
          const Eigen::Vector3d& sol_expected,
          double psd_tol,  // tolerance for checking whether a matrix is psd
          double cost_equality_tol  // tolerance for checking that two solutions
                                    // achieve the same cost
      ) {
        const auto result = Solve(prog);
        auto prog_expected = prog.Clone();
        prog_expected->AddLinearEqualityConstraint(X(0, 1), sol_expected(0));
        prog_expected->AddLinearEqualityConstraint(X(0, 2), sol_expected(1));
        prog_expected->AddLinearEqualityConstraint(X(1, 2), sol_expected(2));
        const auto result_expected = Solve(*prog_expected);

        if (result.get_solver_id() != SnoptSolver::id()) {
          // Do not check when we use SNOPT. It is known that our SnoptSolver
          // wrapper doesn't solve this problem correctly, see
          // https://github.com/RobotLocomotion/drake/pull/9382
          // TODO(hongkai.dai): fix the problem in SnoptSolver wrapper and
          // enable this test with Snopt.

          // Check that expected solution is feasible and achieves the same cost
          // as the optimal solution. This avoids the issue of the expected
          // solution potentially being one of many possible solutions to the
          // optimization problem.
          EXPECT_TRUE(result.is_success());
          EXPECT_TRUE(result_expected.is_success());
          EXPECT_NEAR(result.get_optimal_cost(),
                      result_expected.get_optimal_cost(), cost_equality_tol);

          // The matrix should be positive semidefinite.
          const Eigen::Matrix3d X_sol_expected = result_expected.GetSolution(X);
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_expected(
              X_sol_expected);
          EXPECT_TRUE(
              (eigen_solver_expected.eigenvalues().array() >= -psd_tol).all());

          const Eigen::Matrix3d X_sol = result.GetSolution(X);
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(X_sol);
          EXPECT_TRUE((eigen_solver.eigenvalues().array() >= -psd_tol).all());
        }
      };

  const double psd_tol{1E-6};
  const double cost_equality_tol{1E-8};

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 0, 0));
  solve_and_check(Eigen::Vector3d(-1, 0, 0), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(-1, 0, 0));
  solve_and_check(Eigen::Vector3d(1, 0, 0), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 1, 0));
  solve_and_check(Eigen::Vector3d(0, -1, 0), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, -1, 0));
  solve_and_check(Eigen::Vector3d(0, 1, 0), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 0, 1));
  solve_and_check(Eigen::Vector3d(0, 0, -2), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 0, -1));
  solve_and_check(Eigen::Vector3d(0, 0, 2), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 1, 1));
  solve_and_check(Eigen::Vector3d(0, -1, -2), psd_tol, cost_equality_tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(2, 0, 1));
  solve_and_check(Eigen::Vector3d(-1, 0, -1), psd_tol, cost_equality_tol);
}

}  // namespace solvers
}  // namespace drake
