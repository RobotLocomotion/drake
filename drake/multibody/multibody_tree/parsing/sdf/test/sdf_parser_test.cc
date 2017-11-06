#include "drake/multibody/multibody_tree/parsing/sdf/sdf_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {
namespace {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

static const char* const kTestSdfPath =
    "drake/multibody/multibody_tree/parsing/sdf/models/double_pendulum.sdf";

GTEST_TEST(SDFParserTest, ParsingTest) {
  // Parse tree from test SDF.
  const std::string sdf_path = FindResourceOrThrow(kTestSdfPath);
  const std::string kModelName = "double_pendulum_with_base";

  SDFParser parser;

  auto sdf_spec = parser.ParseSDFModelFromFile(sdf_path);

  PRINT_VAR(sdf_spec->version());
  PRINT_VAR(sdf_spec->get_num_models());
  sdf_spec->GetModelIdByName("double_pendulum_with_base");
  const SDFModel& model = sdf_spec->GetModelByName(kModelName);
  PRINT_VAR(model.name());

  EXPECT_EQ(sdf_spec->version(), "1.6");
  EXPECT_TRUE(sdf_spec->HasModel(kModelName));

#if 0
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
#endif
}

}  // namespace
}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
