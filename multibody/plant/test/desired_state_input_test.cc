#include "drake/multibody/plant/desired_state_input.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;

GTEST_TEST(DesiredStateInputTest, FullApi) {
  DesiredStateInput<double> xd(/* num_model_instances = */ 5);
  const Vector3d q1(1.0, 2.0, 3.0);
  const Vector3d v1(4.0, 5.0, 6.0);
  xd.SetModelInstanceDesiredStates(ModelInstanceIndex(1), q1, v1);

  const Vector2d q3(1.0, 2.0);
  const Vector2d v3(3.0, 4.0);
  xd.SetModelInstanceDesiredStates(ModelInstanceIndex(3), q3, v3);

  EXPECT_EQ(xd.num_model_instances(), 5);
  EXPECT_FALSE(xd.is_armed(ModelInstanceIndex(0)));
  EXPECT_TRUE(xd.is_armed(ModelInstanceIndex(1)));
  EXPECT_FALSE(xd.is_armed(ModelInstanceIndex(2)));
  EXPECT_TRUE(xd.is_armed(ModelInstanceIndex(3)));
  EXPECT_FALSE(xd.is_armed(ModelInstanceIndex(4)));

  EXPECT_EQ(xd.positions(ModelInstanceIndex(1)), q1);
  EXPECT_EQ(xd.velocities(ModelInstanceIndex(1)), v1);
  EXPECT_EQ(xd.positions(ModelInstanceIndex(3)), q3);
  EXPECT_EQ(xd.velocities(ModelInstanceIndex(3)), v3);

  xd.ClearModelInstanceDesiredStates(ModelInstanceIndex(1));
  EXPECT_FALSE(xd.is_armed(ModelInstanceIndex(1)));
  EXPECT_TRUE(xd.is_armed(ModelInstanceIndex(3)));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
