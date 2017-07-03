#include "drake/examples/double_pendulum/sdf_helpers.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

static const char* const kTestSdfPath =
    "/examples/double_pendulum/models/test.sdf";

GTEST_TEST(SDFHelpersTest, ParsingTest) {
  const std::string sdf_path = GetDrakePath() + kTestSdfPath;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  ParseModelFromFile(sdf_path, tree.get());
  tree->compile();

  const RigidBody<double>* body = tree->FindBody("body");
  EXPECT_TRUE(body->has_as_parent(tree->world()));
  const DrakeJoint& world_to_body_joint = body->getJoint();
  EXPECT_TRUE(world_to_body_joint.is_fixed());
  const RigidBody<double>* arm = tree->FindBody("arm");
  EXPECT_TRUE(arm->has_as_parent(*body));
  const DrakeJoint& body_to_arm_joint = arm->getJoint();
  EXPECT_FALSE(body_to_arm_joint.is_fixed());
}

}  // namespace
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
