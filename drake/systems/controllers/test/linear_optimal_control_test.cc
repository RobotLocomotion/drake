#include "drake/systems/controllers/linear_optimal_control.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/primitives/linear_system.h"

GTEST_TEST(TestLQR, DoubleIntegrator) {
  using namespace Eigen;
  using namespace drake;

  // Double integrator dynamics: qddot = u.
  Matrix2d A;
  Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  systems::LinearSystem<double> sys(A, B, Matrix<double, 0, 2>::Zero(),
                                    Matrix<double, 0, 1>::Zero());

  // Trivial cost:
  Matrix2d Q;
  Matrix<double, 1, 1> R;
  Q << 1, 0, 0, 1;
  R << 1;

  // Analytical solution
  Matrix<double, 1, 2> K;
  K << 1, std::sqrt(3);

  double tol = 1e-10;

  // Test LinearSystem version of the LQR
  auto linear_lqr = LinearQuadraticRegulator(sys, Q, R);

  EXPECT_TRUE(CompareMatrices(linear_lqr->A(), Matrix<double, 0, 0>::Zero(),
                              tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->B(), Matrix<double, 0, 1>::Zero(),
                              tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->C(), Matrix<double, 1, 0>::Zero(),
                              tol, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(linear_lqr->D(), -K, tol, MatrixCompareType::absolute));

  // Call it as a generic System (by passing in a Context).
  // Should get the same result, but as an affine system.
  auto context = sys.CreateDefaultContext();
  auto lqr = LinearQuadraticRegulator(sys, *context, Q, R);

  EXPECT_TRUE(CompareMatrices(lqr->A(), Matrix<double, 0, 0>::Zero(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->B(), Matrix<double, 0, 1>::Zero(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->xDot0(), Matrix<double, 0, 1>::Zero(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->C(), Matrix<double, 1, 0>::Zero(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->D(), -K, tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->y0(), Matrix<double, 1, 1>::Zero(), tol,
                              MatrixCompareType::absolute));
}