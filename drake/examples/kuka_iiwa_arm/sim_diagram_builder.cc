#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"

#include <utility>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
systems::DrakeVisualizer* SimDiagramBuilder<T>::AddVisualizer(
    drake::lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(visualizer_ == nullptr);
  DRAKE_DEMAND(plant_ != nullptr);

  visualizer_ = builder_.template AddSystem<systems::DrakeVisualizer>(
      plant_->get_rigid_body_tree(), lcm);

  return visualizer_;
}

template <typename T>
systems::RigidBodyPlant<T>* SimDiagramBuilder<T>::AddPlant(
    std::unique_ptr<systems::RigidBodyPlant<T>> plant) {
  DRAKE_DEMAND(plant_ == nullptr);
  plant_ =
      builder_.template AddSystem<systems::RigidBodyPlant<T>>(std::move(plant));
  return plant_;
}

template <typename T>
systems::RigidBodyPlant<T>* SimDiagramBuilder<T>::AddPlant(
    std::unique_ptr<RigidBodyTree<T>> world_tree) {
  return AddPlant(
      std::make_unique<systems::RigidBodyPlant<T>>(std::move(world_tree)));
}

template <typename T>
void SimDiagramBuilder<T>::ConnectControllersAndVisualizer() {
  DRAKE_DEMAND(plant_);

  // Wires plant state output to controller state input, and controller torque
  // output to plant torque input.
  for (const auto& pair : controllers_) {
    const int instance_id = pair.first;

    systems::StateFeedbackController<T>* controller = pair.second;

    // Connects the state port to the controller.
    const auto& instance_state_output_port =
        plant_->model_instance_state_output_port(instance_id);
    builder_.Connect(instance_state_output_port,
                     controller->get_input_port_estimated_state());

    // Connects the controller torque output to plant.
    const auto& instance_torque_input_port =
        plant_->model_instance_actuator_command_input_port(instance_id);
    builder_.Connect(controller->get_output_port_control(),
                     instance_torque_input_port);
  }

  if (visualizer_) {
    builder_.Connect(plant_->get_output_port(0),
                     visualizer_->get_input_port(0));
  }
}

template class SimDiagramBuilder<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
