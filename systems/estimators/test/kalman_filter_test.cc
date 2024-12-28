#include "drake/systems/estimators/kalman_filter.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators {

namespace {

GTEST_TEST(TestKalman, DoubleIntegrator) {
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

  // Also check the to-be-deprecated overload that accepts a unique_ptr context.
  auto owned = std::make_unique<LinearSystem<double>>(A, B, C, D);
  context = owned->CreateDefaultContext();
  owned->get_input_port().FixValue(context.get(), 0.0);
  context->SetContinuousState(Eigen::Vector2d::Zero());
  filter = SteadyStateKalmanFilter(std::move(owned), std::move(context), W, V);
  EXPECT_TRUE(CompareMatrices(filter->L(), L, tol));
}

}  // namespace

}  // namespace estimators
}  // namespace systems
}  // namespace drake
