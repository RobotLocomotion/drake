#include "drake/multibody/rigid_body_frame.h"

#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace test {
namespace {

using std::make_unique;

// Tests ability to clone a RigidBodyFrame.
GTEST_TEST(RigidBodyFrameTest, TestClone) {
  const std::string kName = "MyRigidBodyFrame";
  const Vector3d xyz(1, 2, 3);
  const Vector3d rpy(4, 5, 6);

  // A nullptr is used since the rigid body pointer is not cloned.
  RigidBodyFrame<double> original_frame(
      kName, nullptr /* rigid body */, xyz, rpy);

  auto cloned_frame = original_frame.Clone();
  EXPECT_TRUE(original_frame.CompareToClone(*cloned_frame));

  // Ensures that a modified clone does not match.
  cloned_frame->set_name(kName + "_mismatch");
  EXPECT_FALSE(original_frame.CompareToClone(*cloned_frame));
}

}  // namespace
}  // namespace test
}  // namespace multibody
}  // namespace drake
