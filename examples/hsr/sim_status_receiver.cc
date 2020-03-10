#include "drake/examples/hsr/sim_status_receiver.h"

namespace drake {
namespace examples {
namespace hsr {

SimStatusReceiver::SimStatusReceiver(
    const multibody::MultibodyPlant<double>* robot_plant)
    : robot_plant_(robot_plant) {
  DRAKE_DEMAND(robot_plant != nullptr);
  DeclareAbstractInputPort("sim_status_message", Value<lcmt_hsr_sim_status>());

  const int state_size =
      robot_plant_->num_velocities() + robot_plant_->num_positions();
  // Estimated state.
  output_port_estimated_state_ = &this->DeclareVectorOutputPort(
      "estimated_state", systems::BasicVector<double>(state_size),
      &SimStatusReceiver::CalcEstimatedStateOutput);

  // Estimated torque
  output_port_torque_ = &this->DeclareVectorOutputPort(
      "torque", systems::BasicVector<double>(robot_plant_->num_actuators()),
      &SimStatusReceiver::CalcTorqueOutput);
}

void SimStatusReceiver::CalcEstimatedStateOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const auto& status =
      this->get_sim_status_input_port().Eval<lcmt_hsr_sim_status>(context);

  const int joint_status_size = status.num_joints;
  DRAKE_DEMAND(joint_status_size == robot_plant_->num_actuators());

  const int num_positions = robot_plant_->num_positions();
  for (int i = 0; i < joint_status_size; ++i) {
    const auto& joint_name = status.joint_name[i];
    const auto& joint_it = robot_plant_->GetJointByName(joint_name);
    (*output)[joint_it.position_start()] = status.joint_position[i];
    (*output)[num_positions + joint_it.velocity_start()] =
        status.joint_velocity[i];
  }
}

void SimStatusReceiver::CalcTorqueOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const auto& status =
      this->get_sim_status_input_port().Eval<lcmt_hsr_sim_status>(context);
  if (status.utime != 0) {
    const int joint_status_size = status.num_joints;
    DRAKE_DEMAND(joint_status_size == robot_plant_->num_actuators());

    for (int i = 0; i < joint_status_size; ++i) {
      const auto& joint_actuator_name = status.joint_name[i] + "_actuator";
      const auto& joint_actuator =
          robot_plant_->GetJointActuatorByName(joint_actuator_name);
      (*output)[joint_actuator.index()] = status.joint_torque[i];
    }
  } else {
    output->SetZero();
    (*output)[0] = 1.0;  // To make a valid quaternion representation.
  }
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake
