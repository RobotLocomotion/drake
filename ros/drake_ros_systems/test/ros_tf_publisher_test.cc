#include <gtest/gtest.h>
#include <memory>

#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/ros_tf_publisher.h"
#include "ros/ros.h"

using std::make_unique;
using std::string;

namespace drake {

using multibody::joints::kFixed;
using parsers::urdf::AddModelInstanceFromUrdfString;
using ros::GetRosParameterOrThrow;

namespace systems {
namespace test {
namespace {

// Tests the RostfPublisher by instantiating it, making it publish a transform,
// and verifying that the expected transform messages were transmitted.
GTEST_TEST(RosTfPublisherTest, TestRosTfPublisher) {
  string urdf_string;
  EXPECT_NO_THROW(urdf_string =
      GetRosParameterOrThrow<string>("model"));
  auto tree = make_unique<RigidBodyTree<double>>();
  AddModelInstanceFromUrdfString(urdf_string, "." /* root string */, kFixed,
      nullptr /* weld to frame */, tree.get());
  auto publisher = make_unique<RosTfPublisher<double>>(*tree);
  auto context = publisher->CreateDefaultContext();
  auto input = make_unique<BasicVector<double>>(tree->get_num_positions() +
      tree->get_num_velocities());

  std::cout << "num positions: " << tree->get_num_positions() << std::endl;
  std::cout << "num velocities: " << tree->get_num_velocities() << std::endl;

  for (int i = 0; i < tree->get_num_positions(); ++i) {
    std::cout << "position " << i << ": " << tree->get_position_name(i) << std::endl;
  }
  for (int i = 0; i < tree->get_num_velocities(); ++i) {
    std::cout << "velocity " << i << ": " << tree->get_velocity_name(i) << std::endl;
  }

  EXPECT_EQ(input->size(), publisher->get_input_port(0).get_size());
  EXPECT_EQ(input->size(), 4);

  std::cout << "size of input: " << input->size() << std::endl;

  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(input->size());
  robot_position << 1.0, -0.5, 0, 0;

  input->get_mutable_value() << robot_position;
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_ros_test_node");
  return RUN_ALL_TESTS();
}
