#include <gtest/gtest.h>
#include <string>

#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/ros_tf_publisher.h"
#include "ros/ros.h"

namespace drake {

using lcm::DrakeLcm;
using ros::GetRosParameterOrThrow;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::RigidBodyPlant;
using systems::RosTfPublisher;
using systems::Simulator;

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

  const Diagram<double>* plant_diagram{nullptr};
  RigidBodyPlant<double>* plant{nullptr};
  std::unique_ptr<Diagram<double>> plant_diagram_ptr = CreateHsrbPlantDiagram(
      urdf_string, penetration_stiffness, penetration_damping,
      friction_coefficient, &lcm, &plant);
  DRAKE_DEMAND(plant_diagram_ptr != nullptr);
  DRAKE_DEMAND(plant != nullptr);

  plant_diagram = plant_diagram_ptr.get();
  DRAKE_DEMAND(plant_diagram != nullptr);

  std::unique_ptr<Diagram<double>> input_diagram =
        CreateHsrbDemo1Diagram(*plant, std::move(plant_diagram_ptr));

  lcm.StartReceiveThread();

  Simulator<double> simulator(*input_diagram);

  // TODO(liang.fok): Modify System 2.0 to not require the following
  // initialization. See #4191.
  //
  // Zeros the rigid body plant's state. This is necessary because it is by
  // default initialized to a vector a NaN values.
  systems::Context<double>* plant_diagram_context =
      input_diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), plant_diagram);

  systems::Context<double>* plant_context =
      plant_diagram->GetMutableSubsystemContext(
          plant_diagram_context, plant);

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
