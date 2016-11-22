#include "drake/examples/toyota_hsrb/demo1_common.h"

#include <gtest/gtest.h>
#include <memory>
// #include <string>

// #include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"
#include "drake/lcm/drake_lcm.h"
// #include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
// #include "drake/ros/parameter_server.h"
// #include "drake/systems/analysis/simulator.h"
// #include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
// #include "drake/systems/ros_tf_publisher.h"
#include "ros/ros.h"

namespace drake {

using lcm::DrakeLcm;
// using systems::Context;
using systems::Diagram;
// using systems::DiagramBuilder;
// using systems::RigidBodyPlant;
// using systems::RosTfPublisher;
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_examples_hsrb_demo1_test_node",
      ros::init_options::AnonymousName);
  ros::start();
  return RUN_ALL_TESTS();
}
