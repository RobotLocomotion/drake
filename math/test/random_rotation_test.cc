#include "drake/math/random_rotation.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace math {
namespace {

GTEST_TEST(RandomRotationTest, Symbolic) {
  RandomGenerator generator;

  const Eigen::AngleAxis<symbolic::Expression> axis_angle =
      UniformlyRandomAngleAxis<symbolic::Expression>(&generator);
  EXPECT_EQ(axis_angle.angle().GetVariables().size(), 1);
  EXPECT_EQ(axis_angle.axis()[0].GetVariables().size(), 3);

  const Eigen::Quaternion<symbolic::Expression> quaternion =
      UniformlyRandomQuaternion<symbolic::Expression>(&generator);
  EXPECT_EQ(quaternion.x().GetVariables().size(), 4);

  const RotationMatrix<symbolic::Expression> rotation_matrix =
      UniformlyRandomRotationMatrix<symbolic::Expression>(&generator);
  EXPECT_EQ(rotation_matrix.matrix()(0, 0).GetVariables().size(), 4);

  // TODO(russt): Add UniformlyRandomRPY test once RollPitchYaw supports
  // symbolic::Expression.
}

}  // namespace
}  // namespace math
}  // namespace drake
