#include "drake/solvers/test/exponential_cone_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
const double kInf = std::numeric_limits<double>::infinity();

void ExponentialConeTrivialExample(const SolverInterface& solver, double tol,
                                   bool check_dual) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>()(0);
  auto y = prog.NewContinuousVariables<1>()(0);
  auto z = prog.NewContinuousVariables<1>()(0);
  auto exp_constr =
      prog.AddExponentialConeConstraint(Vector3<symbolic::Expression>(x, y, z));
  auto lin_eq_constr = prog.AddLinearEqualityConstraint(y + z == 1);
  prog.AddLinearCost(x);

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  // Check the solution. We can rewrite the problem as a single variable
  // optimization min y * exp(1 - y / y). The gradient of this cost function is
  // (y - 1)/y * exp(1-y / y). The optimal value is when the gradient is 0.
  // Namely y =1
  EXPECT_NEAR(result.GetSolution(y), 1, tol);
  EXPECT_NEAR(result.GetSolution(z), 0, tol);

  // Check the dual solution.
  if (check_dual) {
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(exp_constr),
                                Eigen::Vector3d(1, -1, -1), tol));
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(lin_eq_constr),
                                Vector1d(1), tol));
  }
}

void MinimizeKLDivergence(const SolverInterface& solver, double tol) {
  // The probability p(x)
  const Eigen::Vector4d p(0.1, 0.2, 0.3, 0.4);

  MathematicalProgram prog;
  // q is the other probability distribution
  const auto q = prog.NewContinuousVariables<4>();
  // A valid probability should sum up to 1.
  prog.AddLinearEqualityConstraint(Eigen::RowVector4d::Ones(), 1, q);
  // A valid probability should be non-negative.
  prog.AddBoundingBoxConstraint(0, kInf, q);

  // prog minimizes the KL divergence KL(p || q) = ∑ₓ p(x) * log(p(x) / q(x)).
  // Equivalently, the KL divergence is ∑ₓ -p(x) * log(q(x) / p(x)).
  // We introduce a slack variable t(x), such that t(x) >= -p(x)* log(q(x) /
  // p(x)) namely (q(x), p(x), -t(x)) is in the exponential cone, and we
  // minimize ∑ₓ t(x).
  const auto t = prog.NewContinuousVariables<4>();
  for (int i = 0; i < 4; ++i) {
    prog.AddExponentialConeConstraint(
        Vector3<symbolic::Expression>(q(i), p(i), -t(i)));
  }
  prog.AddLinearCost(t.cast<symbolic::Expression>().sum());

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(q), p, tol));
  EXPECT_TRUE(
      CompareMatrices(result.GetSolution(t), Eigen::Vector4d::Zero(), tol));
}

void MinimalEllipsoidCoveringPoints(const SolverInterface& solver, double tol) {
  // We choose 4 special points as the vertices of a box in 2D. The minimal
  // ellipsoid must have all these 4 points on the ellipsoid boundary.
  Eigen::Matrix<double, 2, 4> pts;
  pts << 1, 1, -1, -1, 1, -1, 1, -1;
  // Arbitrarily scale, rotate, and translate the box.
  const double theta = M_PI / 7;
  Eigen::Matrix2d R;
  R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
  const Eigen::Vector2d scaling_factor(0.3, 0.5);
  pts = R * scaling_factor.asDiagonal() * pts;
  // pts += Eigen::Vector2d(0.2, 0.5) * Eigen::RowVector4d::Ones();
  // Create the MathematicalProgram, such that the ellipsoid covers pts.
  MathematicalProgram prog;
  auto S = prog.NewSymmetricContinuousVariables<2>();
  auto b = prog.NewContinuousVariables<2>();
  auto c = prog.NewContinuousVariables<1>()(0);
  Matrix3<symbolic::Expression> ellipsoid_psd;
  ellipsoid_psd << S, b.cast<symbolic::Expression>() / 2,
      b.cast<symbolic::Expression>().transpose() / 2, c;
  prog.AddPositiveSemidefiniteConstraint(ellipsoid_psd);
  prog.AddMaximizeLogDeterminantSymmetricMatrixCost(
      S.cast<symbolic::Expression>());
  for (int i = 0; i < 4; ++i) {
    prog.AddLinearConstraint(
        pts.col(i).dot(S.cast<symbolic::Expression>() * pts.col(i)) +
            pts.col(i).dot(b.cast<symbolic::Expression>()) + c <=
        1);
  }

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  auto S_sol = result.GetSolution(S);
  auto b_sol = result.GetSolution(b);
  auto c_sol = result.GetSolution(c);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(
        pts.col(i).dot(S_sol * pts.col(i)) + b_sol.dot(pts.col(i)) + c_sol, 1,
        tol);
  }
  // Check the volume and the cost matches with the expected minimal volume.
  // We know that for the smallest ellipsoid, S* has eigen values
  // (0.5 / scaling_factor(0)², 0.5 / scaling_factor(1)²). det(S) is just
  // (0.25 / (scaling_factor(0)² * scaling_factor(1)²));
  const double expected_cost =
      -std::log(0.25 / std::pow(scaling_factor(0) * scaling_factor(1), 2));
  EXPECT_NEAR(result.get_optimal_cost(), expected_cost, tol);
  EXPECT_NEAR(-std::log(S_sol.determinant()), expected_cost, tol);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
