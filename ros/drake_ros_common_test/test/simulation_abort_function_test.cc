#include <gtest/gtest.h>

#include "ros/ros.h"

#include "drake/ros/simulation_abort_function.h"

namespace drake {
namespace ros {
namespace {

// Verifies that AddAbortFunction() works.
GTEST_TEST(DrakeRosSimulateAbortFunctionTest, TestAbortFunction) {
  SimulationOptions options;
  EXPECT_NO_THROW(AddAbortFunction(&options));

  double sim_time = 1.0;
  EXPECT_FALSE(options.should_stop(sim_time));
}

}  // namespace
}  // namespace ros
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_ros_simulation_abort_parameter_test_node",
      ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
