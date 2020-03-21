#include "drake/examples/hsr/sim_command_sender.h"

namespace drake {
namespace examples {
namespace hsr {

using multibody::JointActuatorIndex;

SimCommandSender::SimCommandSender(
    const drake::multibody::MultibodyPlant<double>* robot_plant)
    : robot_plant_(robot_plant) {
  DRAKE_DEMAND(robot_plant != nullptr);
  const int state_size =
      robot_plant_->num_positions() + robot_plant_->num_velocities();
  // Desired state.
  this->DeclareInputPort("desired_state", drake::systems::kVectorValued,
                         state_size);

  this->DeclareAbstractOutputPort("sim_command_message",
                                  &SimCommandSender::MakeOutput,
                                  &SimCommandSender::CalcOutput);
}

lcmt_hsr_sim_command SimCommandSender::MakeOutput() const {
  lcmt_hsr_sim_command command_message{};

  const int num_actuators = robot_plant_->num_actuators();
  command_message.num_joints = num_actuators;
  command_message.joint_name.resize(num_actuators);
  command_message.joint_position.resize(num_actuators);
  command_message.joint_velocity.resize(num_actuators);

  return command_message;
}

void SimCommandSender::CalcOutput(const systems::Context<double>& context,
                                  lcmt_hsr_sim_command* output) const {
  DRAKE_DEMAND(output != nullptr);
  lcmt_hsr_sim_command& sim_command = *output;

  const auto& input = this->get_desired_state_input_port().Eval(context);

  sim_command.utime = static_cast<int>(1e6 * context.get_time());
  const int num_positions = robot_plant_->num_positions();
  for (int i = 0; i < robot_plant_->num_actuators(); ++i) {
    const auto& joint_i =
        robot_plant_->get_joint_actuator(JointActuatorIndex(i)).joint();
    sim_command.joint_name[i] = joint_i.name();
    sim_command.joint_position[i] = input[joint_i.position_start()];
    sim_command.joint_velocity[i] =
        input[joint_i.velocity_start() + num_positions];
  }
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake
