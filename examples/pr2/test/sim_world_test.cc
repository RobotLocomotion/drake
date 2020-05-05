#include "drake/examples/pr2/sim_world.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace examples {
namespace pr2 {
namespace {

GTEST_TEST(SimWorldTest, ConstructionTest) {
  const std::string kRobotName = "pr2";
  SimWorld<double> sim_world(std::vector<std::string>{kRobotName});

  const auto& robot_plant = sim_world.get_robot_plant(kRobotName);

  EXPECT_EQ(robot_plant.num_actuators(), 28);
  EXPECT_EQ(robot_plant.num_positions(), 28);
  EXPECT_EQ(robot_plant.num_bodies(), 84);
}

}  // namespace
}  // namespace pr2
}  // namespace examples
}  // namespace drake
