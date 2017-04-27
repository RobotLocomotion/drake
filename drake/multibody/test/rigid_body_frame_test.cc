#include "drake/multibody/rigid_body_frame.h"

#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/multibody/test/rigid_body_frame_compare_to_clone.h"

using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace test {
namespace {

using std::make_unique;

// Tests ability to clone a RigidBodyFrame.
GTEST_TEST(RigidBodyFrameTest, TestClone) {
  const std::string kBodyName = "FooBody";
  const int kModelInstanceId = 1234;

  const std::string kName = "MyRigidBodyFrame";
  RigidBody<double> original_body;
  original_body.set_name(kBodyName);
  original_body.set_model_instance_id(kModelInstanceId);
  const Vector3d xyz(1, 2, 3);
  const Vector3d rpy(4, 5, 6);

  // A nullptr is used since the rigid body pointer is not cloned.
  RigidBodyFrame<double> original_frame(
      kName, &original_body, xyz, rpy);

  auto cloned_body = original_body.Clone();
  auto cloned_frame = original_frame.Clone(cloned_body.get());
  EXPECT_TRUE(rigid_body_frame::CompareToClone(original_frame, *cloned_frame));

  // Ensures that a modified clone does not match.
  cloned_frame->set_name(kName + "_mismatch");
  EXPECT_FALSE(rigid_body_frame::CompareToClone(original_frame, *cloned_frame));
}

}  // namespace
}  // namespace test
}  // namespace multibody
}  // namespace drake
