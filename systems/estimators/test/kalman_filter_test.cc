#include "drake/systems/estimators/kalman_filter.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators {

namespace {

GTEST_TEST(TestContinuousTimeKalman, DoubleIntegrator) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  Eigen::Matrix<double, 1, 2> C;
  Vector1d D;
  A << 0, 1, 0, 0;
  B << 0, 1;
  C << 1, 0;
  D << 0;

  // Specify simple noise covariance matrices.
  Eigen::Matrix2d W;
  Vector1d V;
  W << 1, 0, 0, 1;
  V << 1;

  // Specify the known analytical solution.
  Eigen::Matrix<double, 2, 1> L;
  L << std::sqrt(3), 1;

  double tol = 1e-10;

  // Test the matrix-only version.
  EXPECT_TRUE(CompareMatrices(SteadyStateKalmanFilter(A, C, W, V), L, tol));

  // Test LinearSystem version of the Kalman filter.
  auto sys = std::make_shared<const LinearSystem<double>>(A, B, C, D);
  auto linear_filter = SteadyStateKalmanFilter(sys, W, V);
  EXPECT_TRUE(CompareMatrices(linear_filter->L(), L, tol));

  // Call it as a generic System (by passing in a Context).
  // Should get the same result, but as an affine system.
  auto context = sys->CreateDefaultContext();
  sys->get_input_port().FixValue(context.get(), 0.0);
  context->SetContinuousState(Eigen::Vector2d::Zero());
  auto filter = SteadyStateKalmanFilter(sys, *context, W, V);
  context.reset();
  EXPECT_TRUE(CompareMatrices(filter->L(), L, tol));
}

GTEST_TEST(TestDiscreteTimeKalman, DoubleIntegrator) {
  const double h = 0.01;
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  Eigen::Matrix<double, 1, 2> C;
  Vector1d D;
  A << 1, h, 0, 1;
  B << 0.5 * h * h, h;
  C << 1, 0;
  D << 0;

  // Specify simple noise covariance matrices.
  Eigen::Matrix2d W;
  Vector1d V;
  W << h, 0, 0, h;
  V << 1 / h;

  // Solution using the matlab command A*dlqe(A,eye(2),C,W,V).
  Eigen::Matrix<double, 2, 1> L;
  L << 0.0172705080770414, 0.00991377137943259;

  double tol = 1e-13;

  // Test the matrix-only version.
  EXPECT_TRUE(
      CompareMatrices(DiscreteTimeSteadyStateKalmanFilter(A, C, W, V), L, tol));

  // Test LinearSystem version of the Kalman filter.
  auto sys = std::make_shared<const LinearSystem<double>>(A, B, C, D, h);
  auto linear_filter = SteadyStateKalmanFilter(sys, W, V);
  EXPECT_TRUE(CompareMatrices(linear_filter->L(), L, tol));

  // Call it as a generic System (by passing in a Context).
  // Should get the same result, but as an affine system.
  auto context = sys->CreateDefaultContext();
  sys->get_input_port().FixValue(context.get(), 0.0);
  context->SetDiscreteState(Eigen::Vector2d::Zero());
  auto filter = SteadyStateKalmanFilter(sys, *context, W, V);
  context.reset();
  EXPECT_TRUE(CompareMatrices(filter->L(), L, tol));
}

}  // namespace

}  // namespace estimators
}  // namespace systems
}  // namespace drake
