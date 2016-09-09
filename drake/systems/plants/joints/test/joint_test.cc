#include <gtest/gtest.h>

#include "drake/systems/plants/joints/FixedJoint.h"
#include "drake/systems/plants/joints/HelicalJoint.h"
#include "drake/systems/plants/joints/PrismaticJoint.h"
#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"
#include "drake/systems/plants/joints/RevoluteJoint.h"
#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"

namespace drake {
namespace systems {
namespace plants {
namespace joints {
namespace {

// Note that in the tests below, "dut" stands for "Device Under Test".

GTEST_TEST(DrakeJointTests, TestFixedJoint) {
  FixedJoint dut("foo", Eigen::Isometry3d::Identity());
  EXPECT_TRUE(dut.is_fixed());
}

GTEST_TEST(DrakeJointTests, TestHelicalJoint) {
  Eigen::Vector3d axis;
  axis << 1, 0, 0;

  double pitch = 1.0;

  HelicalJoint dut("foo", Eigen::Isometry3d::Identity(), axis, pitch);
  EXPECT_FALSE(dut.is_fixed());
}

GTEST_TEST(DrakeJointTests, TestPrismaticJoint) {
  Eigen::Vector3d translation_axis_in;
  translation_axis_in << 1, 0, 0;

  PrismaticJoint dut("foo", Eigen::Isometry3d::Identity(),
     translation_axis_in);

  EXPECT_FALSE(dut.is_fixed());
}

GTEST_TEST(DrakeJointTests, TestQuaternionFloatingJoint) {
  QuaternionFloatingJoint dut("foo", Eigen::Isometry3d::Identity());
  EXPECT_FALSE(dut.is_fixed());
}

GTEST_TEST(DrakeJointTests, TestRevoluteJoint) {
  Eigen::Vector3d rotation_axis;
  rotation_axis << 1, 0, 0;

  RevoluteJoint dut("foo", Eigen::Isometry3d::Identity(), rotation_axis);
  EXPECT_FALSE(dut.is_fixed());
}

GTEST_TEST(DrakeJointTests, TestRollPitchYawFloatingJoint) {
  Eigen::Vector3d rotation_axis;
  rotation_axis << 1, 0, 0;

  RollPitchYawFloatingJoint dut("foo", Eigen::Isometry3d::Identity());
  EXPECT_FALSE(dut.is_fixed());
}

}  // namespace
}  // namespace joints
}  // namespace plants
}  // namespace systems
}  // namespace drake
