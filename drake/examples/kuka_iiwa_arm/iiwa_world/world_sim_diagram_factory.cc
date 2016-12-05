#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_diagram_factory.h"

#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/system.h"

namespace drake {

using systems::Diagram;
using systems::DiagramBuilder;
using systems::RigidBodyPlant;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Simulator;
using systems::System;
using lcm::DrakeLcmInterface;
using systems::DrakeVisualizer;

namespace examples {
namespace kuka_iiwa_arm {

std::unique_ptr<Diagram<double>> BuildPlantAndVisualizerDiagram(
    std::unique_ptr<RigidBodyTree<double>> rigid_body_tree,
    double penetration_stiffness, double penetration_damping,
    double friction_coefficient, DrakeLcmInterface* lcm,
    RigidBodyPlant<double>** plant_ptr) {
  std::unique_ptr<DiagramBuilder<double>> builder{
      std::make_unique<DiagramBuilder<double>>()};

  auto plant = builder->template AddSystem<RigidBodyPlant<double>>(
      std::move(rigid_body_tree));

  plant_ptr = &plant;

  DRAKE_DEMAND(plant != nullptr);
  plant->set_contact_parameters(penetration_stiffness, penetration_damping,
                                friction_coefficient);

  DRAKE_DEMAND(plant->get_num_actuators() > 0);

  // Creates and adds a DrakeVisualizer publisher.
  auto viz_publisher_ = builder->template AddSystem<DrakeVisualizer>(
      plant->get_rigid_body_tree(), lcm);

  // Connects the plant to the publisher for visualization.
  builder->Connect(plant->get_output_port(0),
                   viz_publisher_->get_input_port(0));

  // Exposes output and input ports of the Diagram.
  builder->ExportOutput(plant->get_output_port(0));
  builder->ExportInput(plant->get_input_port(0));

  drake::log()->debug("Plant Diagram built...");

  return builder->Build();
}

std::unique_ptr<systems::Diagram<double>> BuildConstantSourceToPlantDiagram(
    std::unique_ptr<systems::Diagram<double>> plant_visualizer_diagram) {
  // Sets up a builder for the demo.
  std::unique_ptr<drake::systems::DiagramBuilder<double>> source_plant_builder{
      std::make_unique<drake::systems::DiagramBuilder<double>>()};

  const int num_inputs = plant_visualizer_diagram->get_input_port(0).get_size();
  auto plant_visualizer_diagram_ptr = source_plant_builder->template AddSystem(
      std::move(plant_visualizer_diagram));

  // Instantiates a constant source that outputs a vector of zeros.
  VectorX<double> constant_value(num_inputs);
  constant_value.setZero();

  auto const_source_ =
      source_plant_builder
          ->template AddSystem<systems::ConstantVectorSource<double>>(
              constant_value);

  // Cascades the constant source to the iiwa plant diagram. This effectively
  // results in the robot being uncontrolled.
  source_plant_builder->Cascade(*const_source_, *plant_visualizer_diagram_ptr);

  return source_plant_builder->Build();
}

void SetZeroConfiguration(Simulator<double>* simulator,
                          const Diagram<double>* demo_diagram) {
  DRAKE_DEMAND(simulator != nullptr && demo_diagram != nullptr);

  std::vector<const systems::System<double>*> demo_systems =
      demo_diagram->GetSystems();

  const Diagram<double>* plant_and_visualizer_diagram =
      dynamic_cast<const systems::Diagram<double>*>(demo_systems.at(0));
  DRAKE_DEMAND(plant_and_visualizer_diagram != nullptr);

  std::vector<const System<double>*> plant_and_visualizer_subsystems =
      plant_and_visualizer_diagram->GetSystems();

  const RigidBodyPlant<double>* rigid_body_plant =
      dynamic_cast<const RigidBodyPlant<double>*>(
          plant_and_visualizer_subsystems.at(0));

  DRAKE_DEMAND(rigid_body_plant != nullptr);

  Context<double>* input_diagram_context =
      demo_diagram->GetMutableSubsystemContext(simulator->get_mutable_context(),
                                               plant_and_visualizer_diagram);
  DRAKE_DEMAND(plant_and_visualizer_diagram != nullptr);

  Context<double>* plant_context =
      plant_and_visualizer_diagram->GetMutableSubsystemContext(
          input_diagram_context, rigid_body_plant);

  rigid_body_plant->SetDefaultState(plant_context);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
