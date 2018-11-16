#include "drake/multibody/parsers/urdf_parser.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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
using parsers::urdf::AddModelInstanceFromUrdfFile;
using parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages;
using parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfStringWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfString;


namespace parsers {
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
  AddModelInstanceFromUrdfStringWithRpyJointToWorld(urdf_string, tree.get());

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
  const string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const string file_no_conflict_1 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_1.urdf");
  const string file_no_conflict_2 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_2.urdf");
  const string file_no_conflict_3 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_3.urdf");

  auto tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_no_conflict_1, tree.get()));

  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_no_conflict_2, tree.get()));

  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_no_conflict_3, tree.get()));

  // This URDF defines the same color multiple times in different links.
  const string file_same_color_diff_links = FindResourceOrThrow(
      resource_dir + "duplicate_but_same_materials.urdf");
  tree = make_unique<RigidBodyTree<double>>();
  EXPECT_NO_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_same_color_diff_links, tree.get()));
}


GTEST_TEST(URDFParserTest, TestDuplicateMaterials) {
  const string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const string file_duplicate = FindResourceOrThrow(
      resource_dir + "duplicate_materials.urdf");

  auto tree = make_unique<RigidBodyTree<double>>();
  EXPECT_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_duplicate, tree.get()), std::runtime_error);
}

GTEST_TEST(URDFParserTest, TestConflictingMaterials) {
  const string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const string file_conflict = FindResourceOrThrow(
      resource_dir + "conflicting_materials.urdf");

  auto tree = make_unique<RigidBodyTree<double>>();
  EXPECT_THROW(AddModelInstanceFromUrdfFileWithRpyJointToWorld(
      file_conflict, tree.get()), std::runtime_error);
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
  const string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const string model_file = FindResourceOrThrow(
      resource_dir + "zero_dof_robot.urdf");
  const string model_string = ReadTextFile(model_file);

  auto tree = make_unique<RigidBodyTree<double>>();
  ASSERT_NO_THROW(AddModelInstanceFromUrdfString(
      model_string, "." /* root_dir */, kQuaternion,
      nullptr /* weld_to_frame */, tree.get()));

  EXPECT_EQ(tree->get_num_positions(), 7);
  EXPECT_EQ(tree->get_num_velocities(), 6);
}

// Tests that AddModelInstanceFromUrdfFile works.
GTEST_TEST(URDFParserTest, TestAddModelInstanceFromUrdfFile) {
  const string model_file = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");

  auto tree = make_unique<RigidBodyTree<double>>();
  ASSERT_NO_THROW(AddModelInstanceFromUrdfFile(
      model_file, kQuaternion, nullptr /* weld_to_frame */, tree.get()));

  EXPECT_EQ(tree->get_num_positions(), 37);
  EXPECT_EQ(tree->get_num_velocities(), 36);
}

// Tests that AddModelInstanceFromUrdfFileSearchingInRosPackages works.
GTEST_TEST(URDFParserTest,
    TestAddModelInstanceFromUrdfFileSearchingInRosPackages) {
  const string model_file = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");

  PackageMap package_map;
  package_map.Add("Atlas", FindResourceOrThrow("drake/examples/atlas"));

  auto tree = make_unique<RigidBodyTree<double>>();
  ASSERT_NO_THROW(AddModelInstanceFromUrdfFileSearchingInRosPackages(
      model_file, package_map, kQuaternion,
      nullptr /* weld_to_frame */, tree.get()));

  EXPECT_EQ(tree->get_num_positions(), 37);
  EXPECT_EQ(tree->get_num_velocities(), 36);
}

// Tests that AddModelInstanceFromUrdfString()'s weld_to_frame parameter works.
// This prevents a regression of #5928.
GTEST_TEST(URDFParserTest, TestAddModelInstanceFromUrdfStringWeldToFrame) {
  const string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const string model_file = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_1.urdf");
  const string model_string = ReadTextFile(model_file);
  const string kModelName = "non_conflicting_materials_1";

  auto tree = make_unique<RigidBodyTree<double>>();
  const ModelInstanceIdTable table1 = AddModelInstanceFromUrdfString(
      model_string, "." /* root_dir */, kQuaternion,
      nullptr /* weld_to_frame */, tree.get());
  const int model_instance_id_1 = table1.at(kModelName);
  // In the following variable, "B" represents the base_link's frame while "F"
  // is the frame in which the new model instance identified by
  // `model_instance_id_2` is defined.
  const Eigen::Isometry3d X_BF = Eigen::Isometry3d::Identity();
  RigidBody<double>* base_body = tree->FindBody("base_link",
      kModelName, model_instance_id_1);
  auto F = std::make_shared<RigidBodyFrame<double>>("base_frame",
    base_body, X_BF);
  const ModelInstanceIdTable table2 = AddModelInstanceFromUrdfString(
      model_string, "." /* root_dir */, kQuaternion, F /* weld_to_frame */,
      tree.get());
  const int model_instance_id_2 = table2.at(kModelName);

  // If the URDF parser's weld_to_frame parameter wasn't working, there would be
  // two base bodies, one for each model instance in the tree.
  EXPECT_EQ(tree->FindBaseBodies().size(), 1u);
  EXPECT_EQ(tree->FindBaseBodies(model_instance_id_1).size(), 1u);
  EXPECT_EQ(tree->FindBaseBodies(model_instance_id_2).size(), 0u);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
