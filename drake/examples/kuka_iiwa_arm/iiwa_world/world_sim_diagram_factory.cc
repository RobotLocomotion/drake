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
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using lcm::DrakeLcmInterface;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::System;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
VisualizedPlant<T>::VisualizedPlant(
    std::unique_ptr<RigidBodyTree<T>> rigid_body_tree,
    double penetration_stiffness, double penetration_damping,
    double friction_coefficient, lcm::DrakeLcmInterface* lcm) {
  DiagramBuilder<T> builder;

  rigid_body_plant_ =
      builder.template AddSystem<RigidBodyPlant<T>>(std::move(rigid_body_tree));

  DRAKE_DEMAND(rigid_body_plant_ != nullptr);
  rigid_body_plant_->set_contact_parameters(
      penetration_stiffness, penetration_damping, friction_coefficient);

  DRAKE_DEMAND(rigid_body_plant_->get_num_actuators() > 0);

  // Creates and adds a DrakeVisualizer publisher.
  auto viz_publisher_ = builder.template AddSystem<DrakeVisualizer>(
      rigid_body_plant_->get_rigid_body_tree(), lcm);

  // Connects the plant to the publisher for visualization.
  builder.Connect(rigid_body_plant_->get_output_port(0),
                  viz_publisher_->get_input_port(0));

  // Exposes output and input ports of the Diagram.
  builder.ExportOutput(rigid_body_plant_->get_output_port(0));
  builder.ExportInput(rigid_body_plant_->get_input_port(0));

  drake::log()->debug("Plant and visualizer Diagram built...");

  builder.BuildInto(this);
}
template class VisualizedPlant<double>;

template <typename T>
PassiveVisualizedPlant<T>::PassiveVisualizedPlant(
    std::unique_ptr<VisualizedPlant<T>> visualized_plant) {
  // Sets up a builder for the demo.
  DiagramBuilder<T> builder;

  const int num_inputs = visualized_plant->get_input_port(0).get_size();
  visualized_plant_ = builder.template AddSystem<VisualizedPlant<T>>(
      std::move(visualized_plant));

  // Instantiates a constant source that outputs a vector of zeros.
  VectorX<double> constant_value(num_inputs);
  constant_value.setZero();

  constant_vector_source_ =
      builder.template AddSystem<systems::ConstantVectorSource<T>>(
          constant_value);

  // Cascades the constant source to the plant and visualizer diagram. This
  // effectively results in the robot being uncontrolled.
  builder.Cascade(*constant_vector_source_, *visualized_plant_);

  builder.BuildInto(this);
}
template class PassiveVisualizedPlant<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
