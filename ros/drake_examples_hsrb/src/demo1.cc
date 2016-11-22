/**
 * @file
 *
 * Implements a Drake + HSRb demo that is uncontrolled, i.e., the robot remains
 * stationary and its arm falls due to gravity. This demo illustrates the
 * ability to load and simulate the HSRb in Drake. See README.md for
 * instructions on how to run this demo.
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

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

int exec(int argc, char* argv[]) {
  ::ros::init(argc, argv, "hsrb_demo_1");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;
  const Diagram<double>* plant_diagram{nullptr};
  RigidBodyPlant<double>* plant{nullptr};
  const Diagram<double>* input_diagram{nullptr};

  {
    std::string urdf_string =
        GetRosParameterOrThrow<std::string>("/robot_description");
    double penetration_stiffness =
        GetRosParameterOrThrow<double>("penetration_stiffness");
    double penetration_damping =
        GetRosParameterOrThrow<double>("penetration_damping");
    double friction_coefficient =
        GetRosParameterOrThrow<double>("friction_coefficient");

    std::unique_ptr<Diagram<double>> plant_diagram_ptr = CreateHsrbPlantDiagram(
        urdf_string, penetration_stiffness, penetration_damping,
        friction_coefficient, &lcm, &plant);
    DRAKE_DEMAND(plant_diagram_ptr != nullptr);
    DRAKE_DEMAND(plant != nullptr);

    plant_diagram = plant_diagram_ptr.get();
    DRAKE_DEMAND(plant_diagram != nullptr);

    std::unique_ptr<Diagram<double>> input_diagram_ptr =
        CreateHsrbDemo1Diagram(*plant, std::move(plant_diagram_ptr));

    input_diagram =
        builder.AddSystem(std::move(input_diagram_ptr));
    DRAKE_DEMAND(input_diagram != nullptr);

    auto ros_tf_publisher = builder.AddSystem<RosTfPublisher<double>>(
        plant->get_rigid_body_tree());
    builder.Connect(input_diagram->get_output_port(0),
                    ros_tf_publisher->get_input_port(0));
  }

  auto demo_diagram = builder.Build();
  lcm.StartReceiveThread();

  Simulator<double> simulator(*demo_diagram);

  // TODO(liang.fok): Modify System 2.0 to not require the following
  // initialization. See #4191.
  //
  // Zeros the rigid body plant's state. This is necessary because it is by
  // default initialized to a vector a NaN values.
  systems::Context<double>* input_diagram_context =
      demo_diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), input_diagram);

  systems::Context<double>* plant_diagram_context =
      input_diagram->GetMutableSubsystemContext(
          input_diagram_context, plant_diagram);

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
