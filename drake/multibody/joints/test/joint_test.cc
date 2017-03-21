#include <memory>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/helical_joint.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/joints/test/joint_compare_to_clone.h"

namespace drake {

using multibody::CompareDrakeJointToClone;
using multibody::CompareFixedJointToClone;
using multibody::CompareHelicalJointToClone;
using multibody::ComparePrismaticJointToClone;
using multibody::CompareRevoluteJointToClone;
using multibody::CompareQuaternionFloatingJointToClone;

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
  // Verifies that each type of joint is equal to its clone.
  EXPECT_TRUE(CompareFixedJointToClone(
      *fixed_joint_,
      *dynamic_cast<FixedJoint*>(fixed_joint_->Clone().get())));
  EXPECT_TRUE(CompareHelicalJointToClone(
      *helical_joint_,
      *dynamic_cast<HelicalJoint*>(helical_joint_->Clone().get())));
  EXPECT_TRUE(ComparePrismaticJointToClone(
      *prismatic_joint_,
      *dynamic_cast<PrismaticJoint*>(prismatic_joint_->Clone().get())));
  EXPECT_TRUE(CompareQuaternionFloatingJointToClone(
      *quaternion_floating_joint_,
      *dynamic_cast<QuaternionFloatingJoint*>(
          quaternion_floating_joint_->Clone().get())));
  EXPECT_TRUE(CompareRevoluteJointToClone(
      *revolute_joint_,
      *dynamic_cast<RevoluteJoint*>(revolute_joint_->Clone().get())));

  // Verifies that two joints with different names do not match.
  EXPECT_FALSE(CompareFixedJointToClone(*fixed_joint_, FixedJoint(
      "NonMatchingName", fixed_joint_->get_transform_to_parent_body())));

  const std::vector<DrakeJoint*> joints = {
      fixed_joint_.get(),
      helical_joint_.get(),
      prismatic_joint_.get(),
      quaternion_floating_joint_.get(),
      revolute_joint_.get()};

  // Verifies that two joints with different transforms and/or types do not
  // match.
  Eigen::Isometry3d non_matching_transform =
      joints.at(0)->get_transform_to_parent_body();
  const double random_angle = 0.7;
  Vector3<double> random_axis(-0.4, 0.3, 0.5);
  random_axis.normalize();
  const AngleAxis<double> non_zero_angle_axis(random_angle, random_axis);
  non_matching_transform.linear() =
      (AngleAxis<double>(non_matching_transform.linear()) *
          non_zero_angle_axis).toRotationMatrix();
  EXPECT_FALSE(CompareDrakeJointToClone(*joints.at(0),
      FixedJoint(fixed_joint_->get_name(), non_matching_transform)));

  // Verifies each type of joint can be cloned and that the clone is equal to
  // the original.
  for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
    EXPECT_TRUE(CompareDrakeJointToClone(
        *joints.at(i), *joints.at(i)->Clone()));
  }

  // Verifies that two joints of different types do not match.
  for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
    for (int j = 0; j < static_cast<int>(joints.size()); ++j) {
      if (j != i) {
        EXPECT_FALSE(CompareDrakeJointToClone(*joints.at(i), *joints.at(j)));
      }
    }
  }
}

}  // namespace
}  // namespace joints
}  // namespace plants
}  // namespace systems
}  // namespace drake
