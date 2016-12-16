#include "drake/examples/toyota_hsrb/passive_demo_common.h"

#include "drake/common/drake_assert.h"
#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/ros_clock_publisher.h"
#include "drake/systems/ros_tf_publisher.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using ros::GetRosParameterOrThrow;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::RigidBodyPlant;
using systems::RosClockPublisher;
using systems::RosTfPublisher;
using systems::Simulator;

namespace examples {
namespace toyota_hsrb {

std::unique_ptr<Simulator<double>> CreateSimulation(lcm::DrakeLcmInterface* lcm,
    std::unique_ptr<Diagram<double>>* demo_diagram) {
  std::string urdf_string =
      GetRosParameterOrThrow<std::string>("/robot_description");
  double penetration_stiffness =
      GetRosParameterOrThrow<double>("penetration_stiffness");
  double penetration_damping =
      GetRosParameterOrThrow<double>("penetration_damping");
  double friction_coefficient =
      GetRosParameterOrThrow<double>("friction_coefficient");

  DiagramBuilder<double> builder;
  Diagram<double>* plant_diagram{nullptr};
  RigidBodyPlant<double>* plant{nullptr};
  Diagram<double>* input_diagram{nullptr};

  {
    std::unique_ptr<Diagram<double>> plant_diagram_ptr =
        BuildPlantAndVisualizerDiagram(
            urdf_string, penetration_stiffness, penetration_damping,
            friction_coefficient, lcm, &plant);
    DRAKE_DEMAND(plant_diagram_ptr != nullptr);
    DRAKE_DEMAND(plant != nullptr);

    plant_diagram = plant_diagram_ptr.get();
    DRAKE_DEMAND(plant_diagram != nullptr);

    std::unique_ptr<Diagram<double>> input_diagram_ptr =
        BuildConstantSourceToPlantDiagram(std::move(plant_diagram_ptr));
    input_diagram =
        builder.AddSystem(std::move(input_diagram_ptr));
    DRAKE_DEMAND(input_diagram != nullptr);

    auto ros_tf_publisher = builder.AddSystem<RosTfPublisher>(
        plant->get_rigid_body_tree());
    builder.Connect(input_diagram->get_output_port(0),
                    ros_tf_publisher->get_input_port(0));
    builder.AddSystem<RosClockPublisher>();
  }

  *demo_diagram = builder.Build();

  auto simulator = make_unique<Simulator<double>>(**demo_diagram);

  simulator->Initialize();
  return std::move(simulator);
}

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
