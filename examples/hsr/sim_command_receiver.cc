#include "drake/examples/hsr/sim_command_receiver.h"

namespace drake {
namespace examples {
namespace hsr {

SimCommandReceiver::SimCommandReceiver(
    const multibody::MultibodyPlant<double>* robot_plant)
    : robot_plant_(robot_plant) {
  DRAKE_DEMAND(robot_plant != nullptr);
  DeclareAbstractInputPort("sim_command_message",
                           Value<lcmt_hsr_sim_command>());

  const int state_size =
      robot_plant_->num_velocities() + robot_plant_->num_positions();
  // Commanded state.
  output_port_desired_state_ = &this->DeclareVectorOutputPort(
      "desired_state", systems::BasicVector<double>(state_size),
      &SimCommandReceiver::CalcDesiredStateOutput);
}

void SimCommandReceiver::CalcDesiredStateOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const auto& command =
      this->get_sim_command_input_port().Eval<lcmt_hsr_sim_command>(context);

  const int joint_command_size = command.num_joints;
  DRAKE_DEMAND(joint_command_size == robot_plant_->num_actuators());

  const int num_positions = robot_plant_->num_positions();
  for (int i = 0; i < joint_command_size; ++i) {
    const auto& joint_name = command.joint_name[i];
    const auto& joint_it = robot_plant_->GetJointByName(joint_name);
    (*output)[joint_it.position_start()] = command.joint_position[i];
    (*output)[num_positions + joint_it.velocity_start()] =
        command.joint_velocity[i];
  }
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake
