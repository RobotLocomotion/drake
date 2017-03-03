#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
systems::RigidBodyPlant<T>* SimDiagramBuilder<T>::AddPlant(std::unique_ptr<systems::RigidBodyPlant<T>> plant) {
  DRAKE_DEMAND(plant_ == nullptr);
  plant_ = this->template AddSystem<systems::RigidBodyPlant<T>>(std::move(plant));
  return plant_;
}

template <typename T>
systems::StateFeedbackController<T>* SimDiagramBuilder<T>::AddController(
    const int instance_id, std::unique_ptr<systems::StateFeedbackController<T>> controller) {
  DRAKE_DEMAND(controllers_.find(instance_id) == controllers_.end());

  systems::StateFeedbackController<T>* controller_ptr = this->template AddSystem(std::move(controller));
  controllers_.emplace(instance_id, controller_ptr);

  return controller_ptr;
}

template <typename T>
void SimDiagramBuilder<T>::AddControllers(std::unordered_map<int, std::unique_ptr<systems::StateFeedbackController<T>>>& controllers) {
  for (auto& pair : controllers) {
    AddController(pair.first, std::move(pair.second));
  }
}

template <typename T>
void SimDiagramBuilder<T>::WireThingsTogether() {
  DRAKE_DEMAND(plant_);

  // Adds the controllers.
  for (const auto& pair : controllers_) {
    const int instance_id = pair.first;

    systems::StateFeedbackController<T>* controller = pair.second;

    // Connects the state port to the controller.
    const auto& instance_state_output_port = plant_->model_instance_state_output_port(instance_id);
    std::cout << instance_state_output_port.size() << ", " << controller->get_input_port_estimated_state().size() << std::endl;
    this->Connect(instance_state_output_port, controller->get_input_port_estimated_state());

    // Connects the controller torque output to plant.
    const auto& instance_torque_input_port = plant_->model_instance_actuator_command_input_port(instance_id);
    this->Connect(controller->get_output_port_control(), instance_torque_input_port);

    /*
    // Expose unconnected controller input ports, since they need to wired up by the caller.
    std::vector<InputPortIdLookup> input_lookup;
    for (int original_id = 0; original_id < controller->get_num_input_ports(); ++original_id) {
      // Skips state input, since that has already been connected.
      if (original_id == controller->get_input_port_estimated_state().get_index())
        continue;

      std::cout << "wiring: " << original_id << "\n";
      // Expose controller input.
      this->ExportInput(controller->get_input_port(original_id));

      // input_lookup.push_back(InputPortIdLookup(original_id, num_exposed_input_ports_));
      // num_exposed_input_ports_++;
    }
    */

    // exposed_inputs_controller_.emplace(instance_id, input_lookup);
  }
}

/*
template <typename T>
void SimDiagramBuilder<T>::ExposePlantOutputPortState(int instance_id) {
  this->ExportOutput(plant_->model_instance_state_output_port(instance_id));
  exposed_outputs_state_.emplace(instance_id, num_exposed_output_ports_);
  num_exposed_output_ports_++;
}

template <typename T>
void SimDiagramBuilder<T>::ExposePlantOutputPortFullState() {
  this->ExportOutput(plant_->get_output_port(0));
  exposed_outputs_state_.emplace(-1, num_exposed_output_ports_);
  num_exposed_output_ports_++;
}
*/

template class SimDiagramBuilder<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
