#include "drake/systems/analysis/discrete_time_approximation.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

GTEST_TEST(DiscreteTimeApproximation, AffineAndLinear) {
  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Vector<double, 2> f0;
  Eigen::Matrix<double, 1, 2> C;
  Eigen::Matrix<double, 1, 1> D;
  Eigen::Vector<double, 1> y0;
  A << 0, 1, 0, 0;
  B << 0, 1;
  f0 << 2, 1;
  C << 1, 0;
  D << 1;
  y0 << 1;

  // Reject discrete system as argument.
  EXPECT_ANY_THROW(DiscreteTimeApproximation<double>(
      AffineSystem<double>(A, B, f0, C, D, y0, 0.1), 0.1));

  const double h = 0.031415926;

  Eigen::Matrix<double, 2, 2> Ad;
  Eigen::Matrix<double, 2, 1> Bd;
  Eigen::Vector<double, 2> f0d;
  Eigen::Matrix<double, 1, 2> Cd = C;
  Eigen::Matrix<double, 1, 1> Dd = D;
  Eigen::Vector<double, 1> y0d = y0;
  Ad << 1, h, 0, 1;
  Bd << 0.5 * h * h, h;
  f0d << 2 * h + 0.5 * h * h, h;

  AffineSystem<double> continuous_affine_sys(A, B, f0, C, D, y0);
  auto discrete_affine_sys =
      DiscreteTimeApproximation<double>(continuous_affine_sys, h);

  const double kEpsilon = 1e-10;
  EXPECT_TRUE(discrete_affine_sys->A().isApprox(Ad, kEpsilon));
  EXPECT_TRUE(discrete_affine_sys->B().isApprox(Bd, kEpsilon));
  EXPECT_TRUE(discrete_affine_sys->f0().isApprox(f0d, kEpsilon));
  EXPECT_TRUE(discrete_affine_sys->C().isApprox(Cd, kEpsilon));
  EXPECT_TRUE(discrete_affine_sys->D().isApprox(Dd, kEpsilon));
  EXPECT_TRUE(discrete_affine_sys->y0().isApprox(y0d, kEpsilon));

  LinearSystem<double> continuous_linear_sys(A, B, C, D);
  auto discrete_linear_sys =
      DiscreteTimeApproximation<double>(continuous_affine_sys, h);

  EXPECT_TRUE(discrete_linear_sys->A().isApprox(Ad, kEpsilon));
  EXPECT_TRUE(discrete_linear_sys->B().isApprox(Bd, kEpsilon));
  EXPECT_TRUE(discrete_linear_sys->C().isApprox(Cd, kEpsilon));
  EXPECT_TRUE(discrete_linear_sys->D().isApprox(Dd, kEpsilon));
}

}  // namespace
}  // namespace systems
}  // namespace drake
