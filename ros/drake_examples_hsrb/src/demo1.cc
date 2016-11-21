/**
 * @file
 *
 * Implements a Drake + HSRb demo that is uncontrolled, i.e., the robot remains
 * stationary and its arm falls due to gravity. The purpose of this demo is to
 * illustrate the ability to load and simulate the robot in Drake.
 * README.md for instructions on how to run this demo.
 */
#include <algorithm>
#include <chrono>
#include <gflags/gflags.h>

#include "ros/ros.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/ros_tf_publisher.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using lcm::DrakeLcm;
// using multibody::AddFlatTerrainToWorld;
// using parsers::urdf::AddModelInstanceFromUrdfString;
using ros::GetRosParameterOrThrow;
// using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
// using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::RosTfPublisher;
using systems::Simulator;

namespace examples {
namespace toyota_hsrb {
namespace {

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

int exec(int argc, char* argv[]) {
  // Parses the command line arguments.
  ::ros::init(argc, argv, "drake_hsrb_demo_1");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  lcm::DrakeLcm lcm;
  // ::ros::NodeHandle ros_node;

  DiagramBuilder<double> builder;
  const Diagram<double>* plant_diagram{nullptr};
  const RigidBodyPlant<double>* plant{nullptr};

  {
    std::string urdf_string =
        GetRosParameterOrThrow<std::string>("/robot_description");
    double penetration_stiffness =
        GetRosParameterOrThrow<double>("penetration_stiffness");
    double penetration_damping =
        GetRosParameterOrThrow<double>("penetration_damping");
    double friction_coefficient =
        GetRosParameterOrThrow<double>("friction_coefficient");

    plant_diagram = builder.AddSystem(
        CreateDemo1Diagram(urdf_string, penetration_stiffness,
                           penetration_damping, friction_coefficient, &lcm));
    DRAKE_DEMAND(plant_diagram != nullptr);

    plant = GetRigidBodyPlant(plant_diagram);
    auto ros_tf_publisher = builder.AddSystem<RosTfPublisher<double>>(
        plant->get_rigid_body_tree());
    builder.Connect(plant_diagram->get_output_port(0),
        ros_tf_publisher->get_input_port(0));
  }

  auto demo_diagram = builder.Build();
  lcm.StartReceiveThread();

  Simulator<double> simulator(*demo_diagram);

  // TODO(liang.fok): Modify System 2.0 to not require the following
  // initialization.
  //
  // Zeros the rigid body plant's state. This is necessary because it is by
  // default initialized to a vector a NaN values.
  systems::Context<double>* plant_diagram_context =
      demo_diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), plant_diagram);

  systems::Context<double>* plant_context =
      plant_diagram->GetMutableSubsystemContext(
          plant_diagram_context, plant);
  plant->SetZeroConfiguration(plant_context);

  simulator.Initialize();

  // The amount of time to simulate between checking for ros::ok();
  const double kSimStepIncrement = 0.1;

  double current_time = std::min(kSimStepIncrement, FLAGS_simulation_sec);
  while (current_time <= FLAGS_simulation_sec && ::ros::ok()) {
    auto start = std::chrono::steady_clock::now();
    simulator.StepTo(current_time);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - start);
    std::cout << "Time: " << current_time << ", real-time factor: "
              << ((kSimStepIncrement * 1000) / duration.count()) << std::endl;
    current_time += kSimStepIncrement;
  }

  return 0;
}

}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::toyota_hsrb::exec(argc, argv);
}
