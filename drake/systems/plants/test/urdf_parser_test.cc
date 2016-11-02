#include <iostream>

#include <gtest/gtest.h>

#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

GTEST_TEST(URDFParserTest, ParseJointProperties) {
  // Defines a URDF string describing a robot with two links and one joint.
  std::string urdf_string =
      "<?xml version=\"1.0\" ?>"
      "<robot name=\"my_robot\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
      "  <link name=\"bar\">"
      "  </link>"
      "  <link name=\"baz\">"
      "    <inertial>"
      "      <mass value=\"100.0\"/>"
      "      <origin rpy=\"0 0 0\" xyz=\"-0.0622 0.0023 0.3157\"/>"
      "      <inertia ixx=\"1.577\" ixy=\"-0.032\" ixz=\"0.102\" iyy=\"1.602\" "
      "iyz=\"0.047\" izz=\"0.565\"/>"
      "    </inertial>"
      "    <visual>"
      "      <geometry>"
      "        <box size=\"0.061842 1.05132 0.030921\"/>"
      "      </geometry>"
      "    </visual>"
      "  </link>"
      "  <joint name=\"foo_joint\" type=\"continuous\">"
      "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
      "    <axis xyz=\"1 0 0\"/>"
      "    <parent link=\"bar\"/>"
      "    <child link=\"baz\"/>"
      "    <dynamics damping=\"1.2\" friction=\"3.4\"/>"
      "    <limit effort=\"123\" lower=\"-1.57079632679\" "
      "upper=\"0.78539816339\" velocity=\"3\"/>"
      "  </joint>"
      "  <transmission name=\"foo_transmission\" "
      "type=\"pr2_mechanism_model/SimpleTransmission\">"
      "    <actuator name=\"foo_motor\"/>"
      "    <joint name=\"foo_joint\"/>"
      "    <mechanicalReduction>0.5</mechanicalReduction>"
      "  </transmission>"
      "</robot>";

  // Instantiates a RigidBodyTree using the URDF string defined above.
  std::unique_ptr<RigidBodyTree> rigid_body_tree(new RigidBodyTree());
  drake::parsers::urdf::AddModelInstanceFromUrdfStringWithRpyJointToWorld(
      urdf_string, rigid_body_tree.get());

  // Obtains the child link of food_joint.
  RigidBody* foo_joint_link =
      rigid_body_tree->FindChildBodyOfJoint("foo_joint");
  EXPECT_TRUE(foo_joint_link != nullptr);

  // Obtains a reference to foo_joint and verifies its parameters are correct.
  const DrakeJoint& foo_joint = foo_joint_link->getJoint();
  EXPECT_EQ(foo_joint.get_name(), "foo_joint");
  EXPECT_FALSE(foo_joint.is_floating());
  EXPECT_EQ(foo_joint.get_num_positions(), 1);
  EXPECT_EQ(foo_joint.get_num_velocities(), 1);

  // Obtains a reference to foo_transmission and verifies its parameters.
  const std::string actuator_name = "foo_motor";
  const RigidBodyActuator& foo_actuator =
      rigid_body_tree->GetActuator(actuator_name);

  EXPECT_EQ(foo_actuator.effort_limit_min_, -123);
  EXPECT_EQ(foo_actuator.effort_limit_max_, 123);
  EXPECT_EQ(foo_actuator.reduction_, 0.5);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
