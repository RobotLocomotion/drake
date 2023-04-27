#include "drake/systems/analysis/linear_system_property.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
GTEST_TEST(Controllable, test1) {
  // Controllable but not observable.
  Eigen::Matrix2d A;
  // clang-format off
  A << -2, 0,
       0, -1;
  // clang-format on
  const Eigen::Vector2d B(2, 1);
  const Eigen::RowVector2d C(3, 0);
  EXPECT_TRUE(Controllable(A, B));
  EXPECT_FALSE(Observable(A, C));
}

GTEST_TEST(Controllable, test2) {
  // Observable but not controllable.
  Eigen::Matrix2d A;
  // clang-format off
  A << -2, 0,
       0, -1;
  // clang-format on
  const Eigen::Vector2d B(2, 0);
  const Eigen::RowVector2d C(3, 2);
  EXPECT_FALSE(Controllable(A, B));
  EXPECT_TRUE(Observable(A, C));
}

GTEST_TEST(Stabilizable, test1) {
  // Stabilizable but not controllable.
  Eigen::Matrix2d A;
  // clang-format off
  A << -2, 0,
        0, 2;
  // clang-format on
  const Eigen::Vector2d B(0, 1);
  EXPECT_TRUE(Stabilizable(A, B, true /* continuous time*/));
  EXPECT_FALSE(Stabilizable(A, B, false /* discrete time*/));
  EXPECT_FALSE(Controllable(A, B));
}

GTEST_TEST(Detectable, test1) {
  // Detectable but not observable.
  Eigen::Matrix3d A;
  // clang-format off
  A << -0.4, 0, 0,
          0, 3, 0,
          0, 0, -2;
  // clang-format on
  const Eigen::RowVector3d C(1, 1, 0);
  EXPECT_TRUE(Detectable(A, C, true /* continuous time*/));
  EXPECT_FALSE(Detectable(A, C, false /* discrete time*/));
  EXPECT_FALSE(Observable(A, C));
}
}  // namespace systems
}  // namespace drake
