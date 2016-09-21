#include <gtest/gtest.h>

#include "ros/ros.h"

#include "drake/ros/parameter_server.h"

namespace drake {
namespace ros {
namespace {

// Verifies that an exception is thrown if the requested parameter does not
// exist.
GTEST_TEST(DrakeRosParameterServerTest, TestGetNonExistentParameter) {
  EXPECT_THROW(GetROSParameter<double>("non_existent_parameter"),
    std::runtime_error);
}

// Verifies that a double can be obtained from the ROS parameter server.
GTEST_TEST(DrakeRosParameterServerTest, TestGetDouble) {
  double value = 0;
  value = GetROSParameter<double>("double_parameter_name");
  EXPECT_NO_THROW(
      value = GetROSParameter<double>("double_parameter_name"));
  EXPECT_EQ(3.14159265359, value);
}

// Verifies that a string can be obtained from the ROS parameter server.
GTEST_TEST(DrakeRosParameterServerTest, TestGetString) {
  std::string value{};
  EXPECT_NO_THROW(
      value = GetROSParameter<std::string>("string_parameter_name"));
  EXPECT_EQ("In teaching others we teach ourselves.", value);
}

// Verifies that an integer can be obtained from the ROS parameter server.
GTEST_TEST(DrakeRosParameterServerTest, TestGetInt) {
  int value{};
  EXPECT_NO_THROW(
      value = GetROSParameter<int>("int_parameter_name"));
  EXPECT_EQ(1729, value);
}

// Verifies that a boolean can be obtained from the ROS parameter server.
GTEST_TEST(DrakeRosParameterServerTest, TestGetBool) {
  bool true_value = false;
  bool false_value = true;
  EXPECT_NO_THROW(
      true_value = GetROSParameter<bool>("bool_parameter_name_true"));
  EXPECT_NO_THROW(
      false_value = GetROSParameter<bool>("bool_parameter_name_false"));
  EXPECT_TRUE(true_value);
  EXPECT_FALSE(false_value);
}

}  // namespace
}  // namespace ros
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_ros_parameter_server_test_node",
      ros::init_options::AnonymousName);
  ros::start();
  return RUN_ALL_TESTS();
}
