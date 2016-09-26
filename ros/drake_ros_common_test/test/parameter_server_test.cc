#include <gtest/gtest.h>

#include "ros/ros.h"

#include "drake/ros/parameter_server.h"

namespace drake {
namespace ros {
namespace {

double short_timeout = 0.1;

// Tests GetRosParameterOrThrow<double>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrThrowDouble) {
  double value{};

  // Verifies a double can be obtained from the ROS parameter server.
  EXPECT_NO_THROW(value =
      GetRosParameterOrThrow<double>("double_parameter_name"));
  EXPECT_EQ(3.14159265359, value);

  // Verifies an exception is thrown if the parameter does not exist.
  EXPECT_THROW(
      GetRosParameterOrThrow<double>("bad_parameter", short_timeout),
      std::runtime_error);

  // Verifies no exception is thrown if the parameter's type is an int since an
  // int can be coverted into a double.
  value = 0;
  EXPECT_NO_THROW(value = GetRosParameterOrThrow<double>("int_parameter_name"));
  EXPECT_EQ(1729.0, value);

  // Verifies an exception is thrown if the parameter's type is not convertible
  // into a `double`.
  EXPECT_THROW(
      GetRosParameterOrThrow<double>("string_parameter_name"),
      std::runtime_error);
}

// Tests GetRosParameterOrThrow<std::string>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrThrowString) {
  std::string value{};

  // Verifies a string can be obtained from the ROS parameter server.
  EXPECT_NO_THROW(value =
      GetRosParameterOrThrow<std::string>("string_parameter_name"));
  EXPECT_EQ("In teaching others we teach ourselves.", value);

  // Verifies an exception is thrown if the parameter does not exist.
  EXPECT_THROW(
      GetRosParameterOrThrow<std::string>("bad_parameter", short_timeout),
      std::runtime_error);

  // Verifies an exception is thrown if the parameter's type is not convertible
  // into a `std::string`.
  EXPECT_THROW(
      GetRosParameterOrThrow<std::string>("double_parameter_name"),
      std::runtime_error);
}

// Tests GetRosParameterOrThrow<int>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrThrowInt) {
  int value{};

  // Verifies an integer can be obtained from the ROS parameter server.
  EXPECT_NO_THROW(
      value = GetRosParameterOrThrow<int>("int_parameter_name"));
  EXPECT_EQ(1729, value);

  // Verifies an exception is thrown if the parameter does not exist.
  EXPECT_THROW(
      GetRosParameterOrThrow<int>("bad_parameter", short_timeout),
      std::runtime_error);

  // Verifies no exception is thrown if the parameter's type is a double.  The
  // parameter's value is 3.14159265359. When this is converted into an int, it
  // becomes 3.
  value = 0;
  EXPECT_NO_THROW(value = GetRosParameterOrThrow<int>("double_parameter_name"));
  EXPECT_EQ(3, value);

  // Verifies an exception is thrown if the parameter's type is not convertible
  // into an `int`.
  EXPECT_THROW(
      GetRosParameterOrThrow<int>("string_parameter_name"), std::runtime_error);
}

// Tests GetRosParameterOrThrow<bool>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrThrowBool) {
  bool true_value{false};
  bool false_value{true};

  // Note that an incorrect default value is used to ensure the correct value
  // was obtained from the ROS parameter server.
  EXPECT_NO_THROW(
      true_value = GetRosParameterOrThrow<bool>(
          "bool_parameter_name_true", false));
  EXPECT_NO_THROW(
      false_value = GetRosParameterOrThrow<bool>(
          "bool_parameter_name_false", true));
  EXPECT_TRUE(true_value);
  EXPECT_FALSE(false_value);

  // Verifies an exception is thrown if the parameter's type is not convertible
  // into a `bool`.
  EXPECT_THROW(GetRosParameterOrThrow<bool>("double_parameter_name"),
      std::runtime_error);
}

// Tests GetRosParameterOrDefault<double>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrDefaultDouble) {
  const int default_value = 1980;
  double value = 0;

  // Verifies a double can be obtained from the ROS parameter server.
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<double>("double_parameter_name", default_value));
  EXPECT_EQ(3.14159265359, value);

  // Verifies the default value is returned when the parameter does not exist.
  value = 0;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<double>("bad_parameter", default_value,
          short_timeout));
  EXPECT_EQ(default_value, value);

  // Verifies no exception is thrown if the parameter's type is an int since an
  // int can be coverted into a double.
  value = 0;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<double>("int_parameter_name", default_value));
  EXPECT_EQ(1729.0, value);

  // Verifies the default value is returned if the parameter's type is not
  // convertable into a `double`.
  value = 0;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<double>("string_parameter_name", default_value));
  EXPECT_EQ(default_value, value);
}

// Tests GetRosParameterOrDefault<std::string>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrDefaultString) {
  const std::string default_value =
      "All things in moderation,  including moderation.";
  std::string value{};

  // Verifies that a string can be obtained from the ROS parameter server.
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<std::string>(
          "string_parameter_name", default_value));
  EXPECT_EQ("In teaching others we teach ourselves.", value);

  // Verifies the default value is returned when the parameter does not exist.
  value = "";
  EXPECT_NO_THROW(
      value = GetRosParameterOrDefault<std::string>("bad_parameter",
          default_value, short_timeout));
  EXPECT_EQ(default_value, value);

  // Verifies the default value is returned if the parameter's type is not
  // convertable into a `string`.
  value = "";
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<std::string>("double_parameter_name",
          default_value));
  EXPECT_EQ(default_value, value);
}

// Tests GetRosParameterOrDefault<int>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrDefaultInt) {
  const int default_value = 617;
  int value{};

  // Verifies that an integer can be obtained from the ROS parameter server.
  EXPECT_NO_THROW(
      value = GetRosParameterOrDefault<int>("int_parameter_name",
          default_value));
  EXPECT_EQ(1729, value);

  // Verifies the default value is returned when the parameter does not exist.
  value = 0;
  EXPECT_NO_THROW(
      value = GetRosParameterOrDefault<int>("bad_parameter", default_value,
          short_timeout));
  EXPECT_EQ(default_value, value);

  // Verifies a `double` is cast into an `int` when accessed as an integer.
  value = 0;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<int>("double_parameter_name", default_value));
  EXPECT_EQ(3, value);

  // Verifies the default value is returned when the parameter is a type that
  // is not convertible into an `int`.
  value = 0;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<int>("string_parameter_name", default_value));
  EXPECT_EQ(default_value, value);
}

// Tests GetRosParameterOrDefault<bool>().
GTEST_TEST(DrakeRosParameterServerTest, TestGetOrDefaultBool) {
  bool true_value = false;
  bool false_value = true;

  // An incorrect default value is used to ensure the correct value was obtained
  // from the ROS parameter server.
  EXPECT_NO_THROW(
      true_value = GetRosParameterOrDefault<bool>(
          "bool_parameter_name_true", false));
  EXPECT_NO_THROW(
      false_value = GetRosParameterOrDefault<bool>(
          "bool_parameter_name_false", true));
  EXPECT_TRUE(true_value);
  EXPECT_FALSE(false_value);

  // Verifies the default value is returned when the parameter does not exist.
  const bool default_value = true;
  bool value = false;
  EXPECT_NO_THROW(
      value = GetRosParameterOrDefault<bool>("bad_parameter", default_value,
        short_timeout));
  EXPECT_EQ(default_value, value);

  // Verifies the default value is returned when the parameter's type is
  // not convertible into a `bool`.
  value = false;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<bool>("double_parameter_name", default_value));
  EXPECT_TRUE(value);

  value = false;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<bool>("double_zero", default_value));
  EXPECT_TRUE(value);

  value = false;
  EXPECT_NO_THROW(value =
      GetRosParameterOrDefault<bool>("int_zero", default_value));
  EXPECT_TRUE(value);
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
