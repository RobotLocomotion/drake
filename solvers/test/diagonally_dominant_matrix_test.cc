#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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

  auto solve_and_check = [&prog, &X](const Eigen::Vector3d& sol_expected,
                                     double tol) {
    const auto result = Solve(prog);
    if (result.get_solver_id() != SnoptSolver::id()) {
      // Do not check when we use SNOPT. It is known that our SnoptSolver
      // wrapper doesn't solve this problem correctly, see
      // https://github.com/RobotLocomotion/drake/pull/9382
      // TODO(hongkai.dai): fix the problem in SnoptSolver wrapper and enable
      // this test with Snopt.
      EXPECT_TRUE(result.is_success());
      EXPECT_TRUE(CompareMatrices(result.GetSolution(VectorDecisionVariable<3>(
                                      X(0, 1), X(0, 2), X(1, 2))),
                                  sol_expected, tol));
      // The matrix should be positive semidefinite.
      const Eigen::Matrix3d X_sol = result.GetSolution(X);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(X_sol);
      EXPECT_TRUE((eigen_solver.eigenvalues().array() >= -tol).all());
    }
  };

  const double tol{1E-6};

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 0, 0));
  solve_and_check(Eigen::Vector3d(-1, 0, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(-1, 0, 0));
  solve_and_check(Eigen::Vector3d(1, 0, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 1, 0));
  solve_and_check(Eigen::Vector3d(0, -1, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, -1, 0));
  solve_and_check(Eigen::Vector3d(0, 1, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 0, 1));
  solve_and_check(Eigen::Vector3d(0, 0, -2), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 0, -1));
  solve_and_check(Eigen::Vector3d(0, 0, 2), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 1, 1));
  solve_and_check(Eigen::Vector3d(0, -1, -2), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(2, 0, 1));
  solve_and_check(Eigen::Vector3d(-1, 0, -1), tol);
}

GTEST_TEST(ReplacePSDConstraintWithDDConstraint, SinglePsdConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto psd_constraint = prog.AddPositiveSemidefiniteConstraint(X);

  // Add an arbitrary linear constraint on X.
  Eigen::MatrixXd A(2, 3);
  // clang-format off
  A << 1, 0, 1,
      0, -1, 1;
  // clang-format on
  Eigen::VectorXd lb(2);
  lb << -10, -7;
  Eigen::VectorXd ub(2);
  ub << 11, 9;
  auto affine_constraint_upper =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) <= ub);
  auto affine_constraint_lower =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) >= lb);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);

  auto dd_constraint = prog.TightenPSDConstraintToDDConstraint(psd_constraint);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog.linear_constraints()), X.rows() * X.rows() + 2);
}

GTEST_TEST(ReplacePSDConstraintWithDDConstraint, MultiPsdConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto Y = prog.NewSymmetricContinuousVariables<4>();
  auto psd_constraint_X = prog.AddPositiveSemidefiniteConstraint(X);
  auto psd_constraint_Y = prog.AddPositiveSemidefiniteConstraint(Y);

  // Add an arbitrary linear constraint on X.
  Eigen::MatrixXd A(2, 3);
  // clang-format off
  A << 1, 0, 1,
      0, -1, 1;
  // clang-format on
  Eigen::VectorXd lb(2);
  lb << -10, -7;
  Eigen::VectorXd ub(2);
  ub << 11, 9;
  auto affine_constraint_upper =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) <= ub);
  auto affine_constraint_lower =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) >= lb);
  auto Y_eq_constraint =
      prog.AddLinearEqualityConstraint(Y == Eigen::MatrixXd::Identity(4, 4));

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 2);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 1);

  auto dd_constraint_X =
      prog.TightenPSDConstraintToDDConstraint(psd_constraint_X);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_constraints()), X.rows() * X.rows() + 2);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 1);

  auto dd_constraint_Y =
      prog.TightenPSDConstraintToDDConstraint(psd_constraint_Y);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog.linear_constraints()),
            Y.rows() * Y.rows() + X.rows() * X.rows() + 2);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 1);
}

GTEST_TEST(ReplacePSDConstraintWithDDConstraint,
           ReplacePsdConstraintNotInProgramVariableNotInProgram) {
  MathematicalProgram prog1;
  auto X1 = prog1.NewSymmetricContinuousVariables<3>();
  auto psd_constraint1 = prog1.AddPositiveSemidefiniteConstraint(X1);

  MathematicalProgram prog2;
  auto X2 = prog2.NewSymmetricContinuousVariables<3>();
  auto psd_constraint2 = prog2.AddPositiveSemidefiniteConstraint(X2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      prog1.TightenPSDConstraintToDDConstraint(psd_constraint2),
      ".*is not a decision variable.*");
}

GTEST_TEST(ReplacePSDConstraintWithDDConstraint,
           ReplacePsdConstraintNotInProgramVariableInProgram) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  // A constraint not in the program.
  auto constraint = internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(X.rows()),
      Eigen::Map<VectorXDecisionVariable>(X.data(), X.size()));
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  prog.TightenPSDConstraintToDDConstraint(constraint);
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  // Still adds the DD constraint even though the constraint was not found in
  // the program.
  EXPECT_EQ(ssize(prog.linear_constraints()), X.rows() * X.rows());
}

}  // namespace solvers
}  // namespace drake
