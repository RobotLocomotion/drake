#include <gtest/gtest.h>
#include <string>

#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
// #include "drake/systems/framework/diagram.h"
#include "ros/ros.h"

namespace drake {
namespace examples {
namespace toyota_hsrb {
namespace {

GTEST_TEST(DrakeExamplesToyotaHsrbTest, TestSim) {
  // Parses the command line arguments.
  lcm::DrakeLcm lcm;

  std::string urdf_string =
      GetRosParameterOrThrow<std::string>("/robot_description");
  double penetration_stiffness =
      GetRosParameterOrThrow<double>("penetration_stiffness");
  double penetration_damping =
      GetRosParameterOrThrow<double>("penetration_damping");
  double friction_coefficient =
      GetRosParameterOrThrow<double>("friction_coefficient");

  auto plant_diagram = CreateDemo1Diagram(
    urdf_string, penetration_stiffness,
    penetration_damping, friction_coefficient, &lcm);

  lcm.StartReceiveThread();

  Simulator<double> simulator(*plant_diagram);

  // TODO(liang.fok): Modify System 2.0 to not require the following
  // initialization.
  //
  // Zeros the rigid body plant's state. This is necessary because it is by
  // default initialized to a vector a NaN values.
  systems::Context<double>* plant_context =
      plant_diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), plant_diagram);

  EXPECT_NE(plant_context, nullptr);

  const RigidBodyPlant<double>* plant = GetRigidBodyPlant(plant_diagram);
  plant->SetZeroConfiguration(plant_context);

  simulator.Initialize();

  const double kSimDuration = 0.1;
  simulator.StepTo(kSimDuration);
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
