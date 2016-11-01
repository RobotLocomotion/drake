#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::CompareMatrices;
using drake::MatrixCompareType;
using drake::parsers::ModelInstanceIdTable;
using drake::systems::plants::joints::kQuaternion;

GTEST_TEST(DrakeJointTests, TestZeroOffset) {
  RigidBodySystem rbs;

  // Loads the SDF model with zero offset between the model's root frame and
  // the world frame.
  ModelInstanceIdTable model_instance_id_table =
      rbs.AddModelInstanceFromFile(
          drake::GetDrakePath() +
              "/systems/plants/test/drake_joint/zero_offset_joint.urdf",
          joints::kQuaternion);

  // Defines the name of the model. This must match the name of the model in the
  // URDF.
  const std::string kModelName = "model_with_zero_offset_joint";

  // Verifies that the model name exists in model_instance_id_table. Aborts if
  // the model name does not exist.
  EXPECT_TRUE(model_instance_id_table.find(kModelName) !=
      model_instance_id_table.end());

  // Obtains the model instance ID.
  int model_instance_id = model_instance_id_table.at(kModelName);

  // Defines the name of the joint. This must match the name of the joint in the
  // URDF.
  const std::string kJointName = "joint1";

  // Gets the body whose joint is the one we're looking for. This should return
  // the joint's child body.
  RigidBody* body = rbs.getRigidBodyTree()->FindChildBodyOfJoint(kJointName,
      model_instance_id);

  EXPECT_EQ(body->get_name(), "link2");

  // Gets the rigid body's joint and its transform to parent body.
  const DrakeJoint& joint = body->getJoint();

  const Eigen::Isometry3d& transform_to_parent_body =
      joint.get_transform_to_parent_body();

  // Since this joint's child body's frame and parent body's frame are
  // coincident, transform_to_parent_body should be identity.
  EXPECT_TRUE(CompareMatrices(
     transform_to_parent_body.matrix(),
     Eigen::Isometry3d::Identity().matrix(),
     Eigen::NumTraits<double>::epsilon(),
     MatrixCompareType::absolute));

  // Evaluates the functional correctness of transform_to_parent_body.
  // Since the joint has zero pose, we expect a point expressed in the child
  // body's frame to have the same coordinates when expressed in the parent
  // body's frame.
  Vector3<double> point_in_child_body = Vector3<double>::Zero();
  point_in_child_body << 1, 2, 3;
  Vector3<double> point_in_parent_body = transform_to_parent_body *
      point_in_child_body;
  Vector3<double> expected_point_in_parent_body = Vector3<double>::Zero();
  expected_point_in_parent_body << 1, 2, 3;
  EXPECT_EQ(point_in_parent_body, expected_point_in_parent_body);
}

GTEST_TEST(DrakeJointTests, TestNonZeroOffset) {
  RigidBodySystem rbs;

  // Loads the SDF model with zero offset between the model's root frame and
  // the world frame.
  ModelInstanceIdTable model_instance_id_table =
      rbs.AddModelInstanceFromFile(
          drake::GetDrakePath() +
              "/systems/plants/test/drake_joint/non_zero_offset_joint.urdf",
          joints::kQuaternion);

  // Defines the name of the model. This must match the name of the model in the
  // URDF.
  const std::string kModelName = "model_with_non_zero_offset_joint";

  // Verifies that the model name exists in model_instance_id_table. Aborts if
  // the model name does not exist.
  EXPECT_TRUE(model_instance_id_table.find(kModelName) !=
      model_instance_id_table.end());

  // Obtains the model instance ID.
  int model_instance_id = model_instance_id_table.at(kModelName);

  // Defines the name of the joint. This must match the name of the joint in the
  // URDF.
  const std::string kJointName = "joint1";

  // Gets the body whose joint is the one we're looking for. This should return
  // the joint's child body.
  RigidBody* body = rbs.getRigidBodyTree()->FindChildBodyOfJoint(kJointName,
      model_instance_id);

  EXPECT_EQ(body->get_name(), "link2");

  // Gets the rigid body's joint and its transform to parent body.
  const DrakeJoint& joint = body->getJoint();

  const Eigen::Isometry3d& transform_to_parent_body =
      joint.get_transform_to_parent_body();

  // Verifies that transform_to_parent_body is correct. The pose of joint 1 is
  // Z = 1. Thus, the expected tranform has a Z offset of 1.
  Eigen::Isometry3d expected_transform_to_parent_body;
  {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                  xyz = Eigen::Vector3d::Zero();
  xyz << 0, 0, 1;
  expected_transform_to_parent_body.matrix()
      << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_TRUE(CompareMatrices(
     transform_to_parent_body.matrix(),
     expected_transform_to_parent_body.matrix(),
     Eigen::NumTraits<double>::epsilon(),
     MatrixCompareType::absolute));

  // Evaluates the functional correctness of transform_to_parent_body.
  // Since the joint has a pose of Z=1, we expect a point expressed in the child
  // body's frame to have a Z coordinate that is +1 when expressed in the parent
  // body's frame.
  Vector3<double> point_in_child_body;
  point_in_child_body << 1, 2, 3;
  Vector3<double> point_in_parent_body = transform_to_parent_body *
      point_in_child_body;
  Vector3<double> expected_point_in_parent_body;
  expected_point_in_parent_body << 1, 2, 4;
  EXPECT_EQ(point_in_parent_body, expected_point_in_parent_body);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
