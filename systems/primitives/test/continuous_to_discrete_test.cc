#include "drake/systems/primitives/continuous_to_discrete.h"

#include <gtest/gtest.h>

#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

GTEST_TEST(LinearSystemContinuousToDiscrete, test) {
  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Matrix<double, 1, 2> C;
  Eigen::Matrix<double, 1, 1> D;
  A << 0, 1, 0, 0;
  B << 0, 1;
  C << 1, 0;

  // reject discrete system as argument
  EXPECT_ANY_THROW(
      ContinuousToDiscrete(LinearSystem<double>(A, B, C, D, 0.1), 0.1));

  LinearSystem<double> continuous_system(A, B, C, D);

  // reject time_period=0.0
  EXPECT_ANY_THROW(ContinuousToDiscrete(continuous_system, 0));

  const double h = 0.031415926;
  auto discrete_system = ContinuousToDiscrete(continuous_system, h);

  Eigen::Matrix<double, 2, 2> Ad;
  Eigen::Matrix<double, 2, 1> Bd;
  Eigen::Matrix<double, 1, 2> Cd = C;
  Eigen::Matrix<double, 1, 1> Dd = D;
  Ad << 1, h, 0, 1;
  Bd << 0.5 * h * h, h;

  const double epsilon = 1e-10;
  EXPECT_TRUE(discrete_system->A().isApprox(Ad, epsilon));
  EXPECT_TRUE(discrete_system->B().isApprox(Bd, epsilon));
  EXPECT_TRUE(discrete_system->C().isApprox(Cd, epsilon));
  EXPECT_TRUE(discrete_system->D().isApprox(Dd, epsilon));
}

GTEST_TEST(AffineSystemContinuousToDiscrete, test) {
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

  // reject discrete system as argument
  EXPECT_ANY_THROW(
      ContinuousToDiscrete(AffineSystem<double>(A, B, f0, C, D, y0, 0.1), 0.1));

  AffineSystem<double> continuous_system(A, B, f0, C, D, y0);

  // reject time_period=0.0
  EXPECT_ANY_THROW(ContinuousToDiscrete(continuous_system, 0));

  const double h = 0.031415926;
  auto discrete_system = ContinuousToDiscrete(continuous_system, h);

  Eigen::Matrix<double, 2, 2> Ad;
  Eigen::Matrix<double, 2, 1> Bd;
  Eigen::Vector<double, 2> f0d;
  Eigen::Matrix<double, 1, 2> Cd = C;
  Eigen::Matrix<double, 1, 1> Dd = D;
  Eigen::Vector<double, 1> y0d = y0;
  Ad << 1, h, 0, 1;
  Bd << 0.5 * h * h, h;
  f0d << 2 * h + 0.5 * h * h, h;

  const double epsilon = 1e-10;
  EXPECT_TRUE(discrete_system->A().isApprox(Ad, epsilon));
  EXPECT_TRUE(discrete_system->B().isApprox(Bd, epsilon));
  EXPECT_TRUE(discrete_system->f0().isApprox(f0d, epsilon));
  EXPECT_TRUE(discrete_system->C().isApprox(Cd, epsilon));
  EXPECT_TRUE(discrete_system->D().isApprox(Dd, epsilon));
  EXPECT_TRUE(discrete_system->y0().isApprox(y0d, epsilon));
}

}  // namespace systems
}  // namespace drake
