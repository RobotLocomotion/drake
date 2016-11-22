#include "drake/examples/toyota_hsrb/demo1_common.h"

#include <gtest/gtest.h>
#include <memory>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "ros/ros.h"

namespace drake {

using lcm::DrakeLcm;
using systems::Diagram;
using systems::Simulator;

namespace examples {
namespace toyota_hsrb {
namespace {

GTEST_TEST(DrakeExamplesToyotaHsrbTest, TestSim) {
  // Parses the command line arguments.
  lcm::DrakeLcm lcm;
  std::unique_ptr<Diagram<double>> demo_diagram;

  auto simulator = CreateSimulation(&lcm, &demo_diagram);
  EXPECT_NE(simulator, nullptr);
  EXPECT_NE(demo_diagram, nullptr);

  const double kSimDuration = 0.1;
  simulator->StepTo(kSimDuration);
}

}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_examples_hsrb_demo1_test_node",
            ros::init_options::AnonymousName);
  ros::start();
  return RUN_ALL_TESTS();
}
