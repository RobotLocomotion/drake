#include "drake/systems/ros_tf_publisher.h"

#include <memory>

#include <gtest/gtest.h>
#include "ros/ros.h"

#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/ros/parameter_server.h"

using std::make_unique;
using std::string;

namespace drake {

using multibody::joints::kFixed;
using parsers::urdf::AddModelInstanceFromUrdfString;
using ros::GetRosParameterOrThrow;

namespace systems {
namespace test {
namespace {

// Tests the RosTfPublisher by instantiating it, making it publish a transform,
// and verifying that the expected transform messages were transmitted.
GTEST_TEST(RosTfPublisherTest, TestRosTfPublisher) {
  string urdf_string;
  EXPECT_NO_THROW(urdf_string =
      GetRosParameterOrThrow<string>("model"));
  auto tree = make_unique<RigidBodyTree<double>>();
  AddModelInstanceFromUrdfString(urdf_string, "." /* root string */, kFixed,
      nullptr /* weld to frame */, tree.get());
  auto publisher = make_unique<RosTfPublisher>(*tree);
  auto context = publisher->CreateDefaultContext();
  auto input = make_unique<BasicVector<double>>(tree->get_num_positions() +
      tree->get_num_velocities());

  EXPECT_EQ(publisher->get_num_output_ports(), 0);
  EXPECT_EQ(input->size(), 4);
  EXPECT_EQ(publisher->get_input_port(0).size(), input->size());

  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(input->size());
  robot_position << 1.0, -0.5, 0, 0;

  input->get_mutable_value() << robot_position;

  context->FixInputPort(0, std::move(input));
  publisher->DoPublish(*context);

  const std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>&
      messages = publisher->get_transform_messages();

  // Checks the transform message for the first body.
  EXPECT_NE(messages.find("two_dof_robot0link1"), messages.end());
  const geometry_msgs::TransformStamped* body1_message =
      messages.at("two_dof_robot0link1").get();
  EXPECT_EQ(body1_message->header.frame_id, "world");
  EXPECT_EQ(body1_message->child_frame_id, "link1");
  EXPECT_EQ(body1_message->transform.translation.x, 0);
  EXPECT_EQ(body1_message->transform.translation.y, 0);
  EXPECT_EQ(body1_message->transform.translation.z, 0);
  EXPECT_EQ(body1_message->transform.rotation.w, 1);
  EXPECT_EQ(body1_message->transform.rotation.x, 0);
  EXPECT_EQ(body1_message->transform.rotation.y, 0);
  EXPECT_EQ(body1_message->transform.rotation.z, 0);

  // Checks the transform message for the second body.
  EXPECT_NE(messages.find("two_dof_robot0link2"), messages.end());
  const geometry_msgs::TransformStamped* body2_message =
      messages.at("two_dof_robot0link2").get();
  EXPECT_EQ(body2_message->header.frame_id, "link1");
  EXPECT_EQ(body2_message->child_frame_id, "link2");
  EXPECT_EQ(body2_message->transform.translation.x, 0);
  EXPECT_EQ(body2_message->transform.translation.y, 0);
  EXPECT_DOUBLE_EQ(body2_message->transform.translation.z, 0.6);
  EXPECT_DOUBLE_EQ(body2_message->transform.rotation.w, 0.87758256189037265);
  EXPECT_DOUBLE_EQ(body2_message->transform.rotation.x, 0.47942553860420295);
  EXPECT_EQ(body2_message->transform.rotation.y, 0);
  EXPECT_EQ(body2_message->transform.rotation.z, 0);

  // Checks the transform message for the third body.
  EXPECT_NE(messages.find("two_dof_robot0link3"), messages.end());
  const geometry_msgs::TransformStamped* body3_message =
      messages.at("two_dof_robot0link3").get();
  EXPECT_EQ(body3_message->header.frame_id, "link2");
  EXPECT_EQ(body3_message->child_frame_id, "link3");
  EXPECT_EQ(body3_message->transform.translation.x, 0);
  EXPECT_EQ(body3_message->transform.translation.y, 0);
  EXPECT_DOUBLE_EQ(body3_message->transform.translation.z, 0.6);
  EXPECT_DOUBLE_EQ(body3_message->transform.rotation.w, 0.96891242171064473);
  EXPECT_DOUBLE_EQ(body3_message->transform.rotation.x, -0.24740395925452291);
  EXPECT_EQ(body3_message->transform.rotation.y, 0);
  EXPECT_EQ(body3_message->transform.rotation.z, 0);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_ros_tf_publisher_test_node");
  return RUN_ALL_TESTS();
}
