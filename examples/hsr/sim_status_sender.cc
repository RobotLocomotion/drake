#include "drake/examples/hsr/sim_status_sender.h"

namespace drake {
namespace examples {
namespace hsr {

using multibody::JointActuatorIndex;

SimStatusSender::SimStatusSender(
    const drake::multibody::MultibodyPlant<double>* robot_plant)
    : robot_plant_(robot_plant) {
  DRAKE_DEMAND(robot_plant != nullptr);
  const int state_size =
      robot_plant_->num_positions() + robot_plant_->num_velocities();
  // Commanded state.
  this->DeclareInputPort("estimated_state", drake::systems::kVectorValued,
                         state_size);

  this->DeclareAbstractOutputPort("sim_status_message",
                                  &SimStatusSender::MakeOutput,
                                  &SimStatusSender::CalcOutput);
}

lcmt_hsr_sim_status SimStatusSender::MakeOutput() const {
  lcmt_hsr_sim_status status_message{};

  const int num_actuators = robot_plant_->num_actuators();
  status_message.num_joints = num_actuators;
  status_message.joint_name.resize(num_actuators);
  status_message.joint_position.resize(num_actuators);
  status_message.joint_velocity.resize(num_actuators);
  status_message.joint_torque.resize(num_actuators);

  return status_message;
}

void SimStatusSender::CalcOutput(const systems::Context<double>& context,
                                 lcmt_hsr_sim_status* output) const {
  DRAKE_DEMAND(output != nullptr);
  lcmt_hsr_sim_status& sim_status = *output;

  const auto& input = this->get_estimated_state_input_port().Eval(context);

  sim_status.utime = static_cast<int>(1e6 * context.get_time());
  const int num_positions = robot_plant_->num_positions();
  for (int i = 0; i < robot_plant_->num_actuators(); ++i) {
    const auto& joint_i =
        robot_plant_->get_joint_actuator(JointActuatorIndex(i)).joint();
    sim_status.joint_name[i] = joint_i.name();
    sim_status.joint_position[i] = input[joint_i.position_start()];
    sim_status.joint_velocity[i] =
        input[joint_i.velocity_start() + num_positions];
    // Set the torque status to 0 for now.
    sim_status.joint_torque[i] = 0.0;
  }
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake
