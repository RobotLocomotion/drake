#include "drake/examples/toyota_hsrb/passive_demo_common.h"

#include <gtest/gtest.h>
#include <memory>
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "ros/ros.h"

namespace drake {

using lcm::DrakeMockLcm;
using systems::Diagram;
using systems::Simulator;

namespace examples {
namespace toyota_hsrb {
namespace {

GTEST_TEST(DrakeExamplesToyotaHsrbTest, TestSim) {
  lcm::DrakeMockLcm lcm;
  std::unique_ptr<Diagram<double>> demo_diagram;

  std::unique_ptr<systems::Simulator<double>> simulator =
      CreateSimulation(&lcm, &demo_diagram);
  EXPECT_NE(simulator, nullptr);
  EXPECT_NE(demo_diagram, nullptr);
  lcm.StartReceiveThread();

  const double kSimDuration = 0.1;
  simulator->StepTo(kSimDuration);
}

}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_examples_toyota_hsrb_passive_demo_test_node",
            ros::init_options::AnonymousName);
  ros::start();
  return RUN_ALL_TESTS();
}
