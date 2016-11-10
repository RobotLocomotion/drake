#include <gtest/gtest.h>
#include  <memory>

#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/helical_joint.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"

namespace drake {
namespace systems {
namespace plants {
namespace joints {
namespace {

using std::make_unique;
using std::unique_ptr;

class DrakeJointTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Defines some parameters that are needed to instantiate joints.
    std::string name = "foo";
    Eigen::Vector3d axis;
    axis << 1, 0, 0;
    Eigen::Isometry3d transform_to_parent_body = Eigen::Isometry3d::Identity();
    double pitch = 1.0;

    // Instantiates one of each type of joint.
    fixed_joint_ =
        make_unique<FixedJoint>(name, transform_to_parent_body);
    helical_joint_ =
        make_unique<HelicalJoint>(name, transform_to_parent_body, axis, pitch);
    prismatic_joint_ =
        make_unique<PrismaticJoint>(name, transform_to_parent_body, axis);
    quaternion_floating_joint_ =
        make_unique<QuaternionFloatingJoint>(name, transform_to_parent_body);
    revolute_joint_ =
        make_unique<RevoluteJoint>(name, transform_to_parent_body, axis);
    roll_pitch_yaw_joint_ =
        make_unique<RollPitchYawFloatingJoint>(name, transform_to_parent_body);
  }

 public:
  unique_ptr<FixedJoint> fixed_joint_;
  unique_ptr<HelicalJoint> helical_joint_;
  unique_ptr<PrismaticJoint> prismatic_joint_;
  unique_ptr<QuaternionFloatingJoint> quaternion_floating_joint_;
  unique_ptr<RevoluteJoint> revolute_joint_;
  unique_ptr<RollPitchYawFloatingJoint> roll_pitch_yaw_joint_;
};

TEST_F(DrakeJointTests, TestIfJointIsFixed) {
  EXPECT_TRUE(fixed_joint_->is_fixed());
  EXPECT_FALSE(helical_joint_->is_fixed());
  EXPECT_FALSE(prismatic_joint_->is_fixed());
  EXPECT_FALSE(quaternion_floating_joint_->is_fixed());
  EXPECT_FALSE(revolute_joint_->is_fixed());
  EXPECT_FALSE(roll_pitch_yaw_joint_->is_fixed());
}

}  // namespace
}  // namespace joints
}  // namespace plants
}  // namespace systems
}  // namespace drake
