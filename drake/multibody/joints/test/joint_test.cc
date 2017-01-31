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
    const std::string name = "foo";
    const Eigen::Vector3d axis(1, 0, 0);
    const Eigen::Isometry3d transform_to_parent_body =
        Eigen::Isometry3d::Identity();
    const double pitch = 1.0;

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

TEST_F(DrakeJointTests, TestCloneAndCompare) {
  // Verifies that a joint is equal to itself.
  EXPECT_EQ(*fixed_joint_, *fixed_joint_);
  EXPECT_EQ(*helical_joint_, *helical_joint_);
  EXPECT_EQ(*prismatic_joint_, *prismatic_joint_);
  EXPECT_EQ(*quaternion_floating_joint_, *quaternion_floating_joint_);
  EXPECT_EQ(*revolute_joint_, *revolute_joint_);

  // Verifies that each type of joint is equal to its clone.
  EXPECT_EQ(*fixed_joint_, *fixed_joint_->Clone());
  EXPECT_EQ(*helical_joint_, *helical_joint_->Clone());
  EXPECT_EQ(*prismatic_joint_, *prismatic_joint_->Clone());
  EXPECT_EQ(*quaternion_floating_joint_, *quaternion_floating_joint_->Clone());
  EXPECT_EQ(*revolute_joint_, *revolute_joint_->Clone());

  // Verifies that two joints with different names do not match.
  EXPECT_NE(*fixed_joint_,
            FixedJoint("NonMatchingName",
                       fixed_joint_->get_transform_to_parent_body()));

  const std::vector<DrakeJoint*> joints = {
      fixed_joint_.get(),
      helical_joint_.get(),
      prismatic_joint_.get(),
      quaternion_floating_joint_.get(),
      revolute_joint_.get()};

  // Verifies that two joints with different transforms and/or types do not
  // match.
  for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
    Eigen::Isometry3d non_matching_transform =
        joints.at(i)->get_transform_to_parent_body();
    non_matching_transform.linear()(0, 0) += 1;
    EXPECT_NE(*joints.at(i),
              FixedJoint(fixed_joint_->get_name(), non_matching_transform));
  }

  // Verifies each type of joint can be cloned and that the clone is equal to
  // the original.
  for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
    EXPECT_EQ(*joints.at(i), *joints.at(i)->Clone());
  }

  // Verifies that two joints of different types do not match.
  for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
    for (int j = 0; j < static_cast<int>(joints.size()); ++j) {
      if (j != i) {
        EXPECT_NE(*joints.at(i), *joints.at(j));
      }
    }
  }
}

}  // namespace
}  // namespace joints
}  // namespace plants
}  // namespace systems
}  // namespace drake
