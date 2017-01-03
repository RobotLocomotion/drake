#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/rigid_body_tree_kinematic_property.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace parsers {

// The test YAML config looks like this:
//
// body_groups:
//    b_group1:
//      []
//
//    b_group2:
//      [link1]
//
//    b_group2:
//      [link3]
//
//    b_group3:
//      [world]
//
// joint_groups:
//    j_group1:
//      []
//
//    j_group2:
//      [base, joint1]
//
// Please refer to the full config file for more details.
void TestKinematicsProperty(multibody::joints::FloatingBaseType type) {
  std::string urdf = drake::GetDrakePath() +
                     "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  std::string config = drake::GetDrakePath() + "/multibody/parsers/test/" +
                       "rigid_body_tree_kinematic_property_test.config";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf, type, robot.get());

  RigidBodyTreeKinematicProperty<double> kin_prop(*robot);

  YAML::Node file = YAML::LoadFile(config);
  kin_prop.LoadFromYAMLFile(file);

  EXPECT_TRUE(kin_prop.has_position_group("j_group1"));
  EXPECT_TRUE(kin_prop.has_position_group("j_group2"));
  EXPECT_TRUE(kin_prop.has_velocity_group("j_group1"));
  EXPECT_TRUE(kin_prop.has_velocity_group("j_group2"));
  EXPECT_FALSE(kin_prop.has_position_group("j_group33"));
  EXPECT_FALSE(kin_prop.has_velocity_group("j_group33"));

  EXPECT_TRUE(kin_prop.has_body_group("b_group1"));
  EXPECT_TRUE(kin_prop.has_body_group("b_group2"));
  EXPECT_TRUE(kin_prop.has_body_group("b_group3"));
  EXPECT_FALSE(kin_prop.has_body_group("b_group55"));

  EXPECT_EQ(kin_prop.get_position_group("j_group1").size(), 0);
  EXPECT_EQ(kin_prop.get_velocity_group("j_group1").size(), 0);

  EXPECT_EQ(kin_prop.get_body_group("b_group1").size(), 0);
  EXPECT_EQ(kin_prop.get_body_group("b_group2").size(), 2);
  EXPECT_EQ(kin_prop.get_body_group("b_group2")[0]->get_name(), "link1");
  EXPECT_EQ(kin_prop.get_body_group("b_group2")[1]->get_name(), "link3");
  EXPECT_EQ(kin_prop.get_body_group("b_group3").size(), 1);
  EXPECT_EQ(kin_prop.get_body_group("b_group3")[0]->get_name(), "world");

  EXPECT_EQ(kin_prop.get_joint_group("j_group1").size(), 0);
  EXPECT_EQ(kin_prop.get_joint_group("j_group2").size(), 2);
  EXPECT_EQ(kin_prop.get_joint_group("j_group2")[0]->get_name(), "base");
  EXPECT_EQ(kin_prop.get_joint_group("j_group2")[1]->get_name(), "joint1");

  const std::vector<int>& q_indices = kin_prop.get_position_group("j_group2");
  const std::vector<int>& v_indices = kin_prop.get_velocity_group("j_group2");
  switch (type) {
    case drake::multibody::joints::kQuaternion:
      EXPECT_EQ(q_indices.size(), 8);
      EXPECT_EQ(v_indices.size(), 7);
      EXPECT_EQ(robot->get_position_name(q_indices[0]), "base_x");
      EXPECT_EQ(robot->get_position_name(q_indices[1]), "base_y");
      EXPECT_EQ(robot->get_position_name(q_indices[2]), "base_z");
      EXPECT_EQ(robot->get_position_name(q_indices[3]), "base_qw");
      EXPECT_EQ(robot->get_position_name(q_indices[4]), "base_qx");
      EXPECT_EQ(robot->get_position_name(q_indices[5]), "base_qy");
      EXPECT_EQ(robot->get_position_name(q_indices[6]), "base_qz");

      EXPECT_EQ(robot->get_velocity_name(v_indices[0]), "base_wx");
      EXPECT_EQ(robot->get_velocity_name(v_indices[1]), "base_wy");
      EXPECT_EQ(robot->get_velocity_name(v_indices[2]), "base_wz");
      EXPECT_EQ(robot->get_velocity_name(v_indices[3]), "base_vx");
      EXPECT_EQ(robot->get_velocity_name(v_indices[4]), "base_vy");
      EXPECT_EQ(robot->get_velocity_name(v_indices[5]), "base_vz");
      break;
    case multibody::joints::kRollPitchYaw:
      EXPECT_EQ(q_indices.size(), 7);
      EXPECT_EQ(v_indices.size(), 7);
      EXPECT_EQ(robot->get_position_name(q_indices[0]), "base_x");
      EXPECT_EQ(robot->get_position_name(q_indices[1]), "base_y");
      EXPECT_EQ(robot->get_position_name(q_indices[2]), "base_z");
      EXPECT_EQ(robot->get_position_name(q_indices[3]), "base_roll");
      EXPECT_EQ(robot->get_position_name(q_indices[4]), "base_pitch");
      EXPECT_EQ(robot->get_position_name(q_indices[5]), "base_yaw");

      EXPECT_EQ(robot->get_velocity_name(v_indices[0]), "base_xdot");
      EXPECT_EQ(robot->get_velocity_name(v_indices[1]), "base_ydot");
      EXPECT_EQ(robot->get_velocity_name(v_indices[2]), "base_zdot");
      EXPECT_EQ(robot->get_velocity_name(v_indices[3]), "base_rolldot");
      EXPECT_EQ(robot->get_velocity_name(v_indices[4]), "base_pitchdot");
      EXPECT_EQ(robot->get_velocity_name(v_indices[5]), "base_yawdot");
      break;
    case multibody::joints::kFixed:
      EXPECT_EQ(q_indices.size(), 1);
      EXPECT_EQ(v_indices.size(), 1);
      break;
  }

  EXPECT_EQ(robot->get_position_name(q_indices.back()), "joint1");
  EXPECT_EQ(robot->get_velocity_name(v_indices.back()), "joint1dot");
}

GTEST_TEST(RigidBodyTreeYAMLParsingTest, TestJointGroup) {
  TestKinematicsProperty(multibody::joints::kRollPitchYaw);
  TestKinematicsProperty(multibody::joints::kQuaternion);
  TestKinematicsProperty(multibody::joints::kFixed);
}

}  // namespace parsers
}  // namespace drake
