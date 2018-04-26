#include "drake/examples/double_pendulum/sdf_helpers.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/examples/double_pendulum/test/rigid_body_types_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

static const char* const kTestSdfPath =
    "drake/examples/double_pendulum/models/test.sdf";

GTEST_TEST(SDFHelpersTest, ParsingTest) {
  // Parse tree from test SDF.
  const std::string sdf_path = FindResourceOrThrow(kTestSdfPath);
  RigidBodyTree<double> parsed_tree;
  ParseModelFromFile(sdf_path, &parsed_tree);
  parsed_tree.compile();

  // Build reference body.
  RigidBody<double> body_ref;
  body_ref.set_mass(60.0);
  body_ref.set_center_of_mass(
      Vector3<double>(0.0, 0.0, 0.9));
  body_ref.set_name("body");
  const Isometry3<double>::TranslationType body_element_translation(0, 0, 0.9);
  const Quaternion<double> body_element_rotation =
      Quaternion<double>::Identity();
  // This transform for both collision and visual elements.
  const Isometry3<double> body_element_transform =
      body_element_translation * body_element_rotation;
  DrakeShapes::Box body_geometry(Vector3<double>(0.2, 0.2, 1.8));
  drake::multibody::collision::Element body_collision(body_geometry,
                                                      body_element_transform);
  body_ref.AddCollisionElement("", &body_collision);
  DrakeShapes::VisualElement body_visual(body_element_transform);
  body_visual.setGeometry(body_geometry);
  body_ref.AddVisualElement(body_visual);

  // Build reference arm.
  RigidBody<double> arm_ref;
  arm_ref.set_mass(2.0);
  arm_ref.set_name("arm");

  // This transform applies for the link.
  const Isometry3<double>::TranslationType arm_translation(0, 0, 1.5);
  const AngleAxis<double> arm_rotation(-1.5708, Vector3<double>::UnitX());
  const Isometry3<double> arm_transform = arm_translation * arm_rotation;

  // This transform applies for both collision and visual elements.
  const Isometry3<double>::TranslationType arm_element_translation(-0.05, 0, 0);
  const AngleAxis<double> arm_element_rotation(
      1.5708, Vector3<double>::UnitY());
  const Isometry3<double> arm_element_transform =
      arm_element_translation * arm_element_rotation;

  DrakeShapes::Cylinder arm_geometry(0.05, 0.4);
  drake::multibody::collision::Element arm_collision(arm_geometry,
                                                     arm_element_transform);
  arm_ref.AddCollisionElement("", &arm_collision);
  DrakeShapes::VisualElement arm_visual(arm_element_transform);
  arm_visual.setGeometry(arm_geometry);
  arm_ref.AddVisualElement(arm_visual);

  // Check parsed tree bodies and joints.
  RigidBody<double>* body = parsed_tree.FindBody("body");
  ASSERT_TRUE(body != nullptr);
  EXPECT_TRUE(body->has_as_parent(parsed_tree.world()));
  EXPECT_TRUE(test::AreBodiesEquivalent(*body, body_ref));

  const DrakeJoint& world_to_body_joint = body->getJoint();
  EXPECT_TRUE(world_to_body_joint.is_fixed());

  RigidBody<double>* arm = parsed_tree.FindBody("arm");
  ASSERT_TRUE(arm != nullptr);
  EXPECT_TRUE(arm->has_as_parent(*body));
  EXPECT_TRUE(test::AreBodiesEquivalent(*arm, arm_ref));

  const DrakeJoint& body_to_arm_joint = arm->getJoint();
  EXPECT_EQ(body_to_arm_joint.get_name(), "shoulder");
  EXPECT_FALSE(body_to_arm_joint.is_fixed());
  EXPECT_TRUE(arm_transform.isApprox(
     body_to_arm_joint.get_transform_to_parent_body()));
}

}  // namespace
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
