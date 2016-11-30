#include "drake/multibody/parser_urdf.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree.h"

using std::endl;
using std::ifstream;
using std::make_unique;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using multibody::joints::kQuaternion;
using parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfStringWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfStringSearchingInRosPackages;

namespace systems {
namespace plants {
namespace test {
namespace {

GTEST_TEST(URDFParserTest, ParseJointProperties) {
  // Defines a URDF string describing a robot with two links and one joint.
  string urdf_string =
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
  auto tree = make_unique<RigidBodyTree<double>>();
  AddModelInstanceFromUrdfStringWithRpyJointToWorld(
      urdf_string, tree.get());

  // Obtains the child link of food_joint.
  RigidBody<double>* foo_joint_link = tree->FindChildBodyOfJoint("foo_joint");
  EXPECT_TRUE(foo_joint_link != nullptr);

  // Obtains a reference to foo_joint and verifies its parameters are correct.
  const DrakeJoint& foo_joint = foo_joint_link->getJoint();
  EXPECT_EQ(foo_joint.get_name(), "foo_joint");
  EXPECT_FALSE(foo_joint.is_floating());
  EXPECT_EQ(foo_joint.get_num_positions(), 1);
  EXPECT_EQ(foo_joint.get_num_velocities(), 1);

  // Obtains a reference to foo_transmission and verifies its parameters.
  const string actuator_name = "foo_motor";
  const RigidBodyActuator& foo_actuator = tree->GetActuator(actuator_name);

  EXPECT_EQ(foo_actuator.effort_limit_min_, -123);
  EXPECT_EQ(foo_actuator.effort_limit_max_, 123);
  EXPECT_EQ(foo_actuator.reduction_, 0.5);
}

GTEST_TEST(URDFParserTest, TestParseMaterial) {
  const string file_no_conflict_1 = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/non_conflicting_materials_1.urdf";
  const string file_no_conflict_2 = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/non_conflicting_materials_2.urdf";
  const string file_no_conflict_3 = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/non_conflicting_materials_3.urdf";
  const string file_duplicate = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/duplicate_materials.urdf";
  const string file_conflict = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/conflicting_materials.urdf";

  auto tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_no_conflict_1, tree.get()));

  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_no_conflict_2, tree.get()));

  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_no_conflict_3, tree.get()));

  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_DEATH(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_duplicate, tree.get()), ".*");

  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_DEATH(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_conflict, tree.get()), ".*");

  // This URDF defines the same color multiple times in different links.
  const string file_same_color_diff_links = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/duplicate_but_same_materials.urdf";
  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_same_color_diff_links, tree.get()));
}

string ReadTextFile(const string& file) {
  ifstream text_file(file);
  DRAKE_DEMAND(text_file.is_open());

  stringstream buffer;
  string line;
  while (getline(text_file, line)) {
    buffer << line << endl;
  }
  text_file.close();
  return buffer.str();
}

// Tests that a URDF string can be loaded using a quaternion floating joint.
// This was added as a result of #4248.
GTEST_TEST(URDFParserTest, TestAddWithQuaternionFloatingDof) {
  const string model_file = drake::GetDrakePath() +
      "/multibody/test/parser_urdf_test/zero_dof_robot.urdf";

  const string model_string = ReadTextFile(model_file);
  PackageMap ros_package_map;

  auto tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfStringSearchingInRosPackages(
      model_string, ros_package_map, "." /* root_dir */, kQuaternion,
      nullptr /* weld_to_frame */, tree.get()));

  EXPECT_EQ(tree->get_num_positions(), 7);
  EXPECT_EQ(tree->get_num_velocities(), 6);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
