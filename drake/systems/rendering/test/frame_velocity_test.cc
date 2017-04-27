#include "drake/systems/rendering/frame_velocity.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

// Tests that a SpatialVelocity can be set from and read as a FrameVelocity.
GTEST_TEST(FrameVelocityTest, SetAndRead) {
  // The velocity is initialized to zero.
  FrameVelocity<double> vec;
  EXPECT_EQ(Vector6<double>::Zero(), vec.get_value());

  Vector6<double> data;
  data << 1, 2, 3, 4, 5, 6;
  multibody::SpatialVelocity<double> vel(data);
  vec.set_velocity(vel);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(1, 2, 3),
                              vec.get_velocity().rotational()));
  EXPECT_TRUE(CompareMatrices(Vector3<double>(4, 5, 6),
                              vec.get_velocity().translational()));
}

// Tests that FrameVelocity is copyable and assignable.
GTEST_TEST(FrameVelocityTest, CopyAndAssign) {
  FrameVelocity<double> vec;
  vec[2] = 42;

  // Self-assignment.
  vec = vec;
  EXPECT_EQ(42, vec[2]);

  // Copying.
  FrameVelocity<double> copy(vec);
  EXPECT_EQ(42, copy.GetAtIndex(2));

  // Assignment.
  copy[3] = 43;
  vec = copy;
  EXPECT_EQ(43, vec.GetAtIndex(3));
}

GTEST_TEST(FrameVelocityTest, Clone) {
  FrameVelocity<double> vec;
  Vector6<double> data;
  data << 1, 2, 3, 4, 5, 6;
  multibody::SpatialVelocity<double> vel(data);
  vec.set_velocity(vel);

  auto clone = vec.Clone();
  auto typed_clone = dynamic_cast<FrameVelocity<double>*>(clone.get());
  ASSERT_NE(nullptr, typed_clone);
  ASSERT_EQ(data, clone->get_value());
}

};  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
