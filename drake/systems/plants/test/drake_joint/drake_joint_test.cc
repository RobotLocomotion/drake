#include <gtest/gtest.h>

// #include "drake/common/eigen_types.h"
// #include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::parsers::ModelInstanceIdTable;

GTEST_TEST(DrakeJointTests, TestZeroOffset) {
  RigidBodySystem rbs;

  // Loads the SDF model with zero offset between the model's root frame and
  // the world frame.
  ModelInstanceIdTable model_instance_id_table =
      rbs.AddModelInstanceFromFile(
          drake::GetDrakePath() +
              "/systems/plants/test/drake_joint/zero_offset_joint.urdf",
          DrakeJoint::QUATERNION);

  // Defines the name of the model. This must match the name of the model in the
  // URDF.
  const std::string kModelName = "model_with_zero_offset_joint";

  // Verifies that the model name exists in model_instance_id_table. Aborts if
  // the model name does not exist.
  EXPECT_TRUE(model_instance_id_table.find(kModelName) !=
  	  model_instance_id_table.end());

  // Obtains the model instance ID.
  int model_instance_id = model_instance_id_table.at(
  	"model_with_zero_offset_joint");

  // Defines the name of the joint. This must match the name of the joint in the
  // URDF.
  const std::string kJointName = "joint1";

  // Gets the rigid body whose joint is the one we're looking for.
  RigidBody* body = rbs.getRigidBodyTree()->findJoint(kJointName,
  	  model_instance_id);

  EXPECT_EQ(body->get_name(), "link1");

  // Gets the rigid body's joint and its transform to parent body.
  const DrakeJoint& joint = body->getJoint();

  const Eigen::Isometry3d& transform_to_parent_body =
      joint.get_transform_to_parent_body();

  // Since this joint's child body's frame and parent body's frame are
  // coincident, transform_to_parent_body should be identity.
  EXPECT_EQ(transform_to_parent_body, Eigen::Isometry3d::Identity());
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
