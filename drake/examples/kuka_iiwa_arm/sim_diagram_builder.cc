#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
void SimDiagramBuilder<T>::Setup(
    std::unique_ptr<RigidBodyTree<T>> world_tree_ptr,
    std::unordered_map<int, std::unique_ptr<systems::StateFeedbackController<T>>>& robot_controllers) {

  // Makes a plant first
  plant_ = builder_.template AddSystem<systems::RigidBodyPlant<T>>(std::move(world_tree_ptr));
  // const RigidBodyTree<T>& world_tree = plant_->get_rigid_body_tree();

  // Adds the controllers.
  for (auto& pair : robot_controllers) {
    const int instance_id = pair.first;

    systems::StateFeedbackController<T>* controller =
        builder_.template AddSystem(std::move(pair.second));
    controllers_.emplace(instance_id, controller);

    // Connects the state port to the controller.
    const auto& instance_state_output_port = plant_->model_instance_state_output_port(instance_id);
    std::cout << instance_state_output_port.size() << ", " << controller->get_input_port_estimated_state().size() << std::endl;
    builder_.Connect(instance_state_output_port, controller->get_input_port_estimated_state());

    // Connects the controller torque output to plant.
    const auto& instance_torque_input_port = plant_->model_instance_actuator_command_input_port(instance_id);
    builder_.Connect(controller->get_output_port_control(), instance_torque_input_port);

    // Expose unconnected controller input ports, since they need to wired up by the caller.
    std::vector<InputPortIdLookup> input_lookup; // vector of pair(input id of the controller, input id of the resulting diagram)
    for (int original_id = 0; original_id < controller->get_num_input_ports(); ++original_id) {
      // Skips state input, since that has already been connected.
      if (original_id == controller->get_input_port_estimated_state().get_index())
        continue;

      // Expose controller input.
      builder_.ExportInput(controller->get_input_port(original_id));

      input_lookup.push_back(InputPortIdLookup(original_id, num_exposed_input_ports_));
      num_exposed_input_ports_++;
    }

    exposed_inputs_controller_.emplace(instance_id, input_lookup);
  }
}

template <typename T>
void SimDiagramBuilder<T>::ExposePlantOutputPortState(int instance_id) {
  builder_.ExportOutput(plant_->model_instance_state_output_port(instance_id));
  exposed_outputs_state_.emplace(instance_id, num_exposed_output_ports_);
  num_exposed_output_ports_++;
}

template <typename T>
void SimDiagramBuilder<T>::ExposePlantOutputPortFullState() {
  builder_.ExportOutput(plant_->get_output_port(0));
  exposed_outputs_state_.emplace(-1, num_exposed_output_ports_);
  num_exposed_output_ports_++;
}

template class SimDiagramBuilder<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
