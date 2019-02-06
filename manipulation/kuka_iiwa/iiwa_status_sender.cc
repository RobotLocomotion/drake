#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using systems::BasicVector;
using systems::Context;

IiwaStatusSender::IiwaStatusSender(int num_joints) : num_joints_(num_joints) {
  // Commanded state.
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  // Measured state.
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  // Commanded torque.
  this->DeclareInputPort(systems::kVectorValued, num_joints_);
  // Measured torque.
  this->DeclareInputPort(systems::kVectorValued, num_joints_);
  // Measured external torque.
  this->DeclareInputPort(systems::kVectorValued, num_joints_);

  this->DeclareAbstractOutputPort(&IiwaStatusSender::MakeOutputStatus,
                                  &IiwaStatusSender::OutputStatus);
}

lcmt_iiwa_status IiwaStatusSender::MakeOutputStatus() const {
  lcmt_iiwa_status msg{};
  msg.num_joints = num_joints_;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_velocity_estimated.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);
  return msg;
}

void IiwaStatusSender::OutputStatus(const Context<double>& context,
                                    lcmt_iiwa_status* output) const {
  lcmt_iiwa_status& status = *output;

  status.utime = context.get_time() * 1e6;
  const BasicVector<double>* command = this->EvalVectorInput(context, 0);
  const BasicVector<double>* state = this->EvalVectorInput(context, 1);
  const BasicVector<double>* commanded_torque =
      this->EvalVectorInput(context, 2);
  const BasicVector<double>* measured_torque =
      this->EvalVectorInput(context, 3);
  const BasicVector<double>* external_torque =
      this->EvalVectorInput(context, 4);

  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_velocity_estimated[i] = state->GetAtIndex(i + num_joints_);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
    status.joint_torque_commanded[i] = commanded_torque->GetAtIndex(i);

    if (external_torque) {
      status.joint_torque_external[i] = external_torque->GetAtIndex(i);
    }
    if (measured_torque) {
      status.joint_torque_measured[i] = measured_torque->GetAtIndex(i);
    } else {
      // TODO(rcory) Update joint_torque_measured to report actual measured
      // torque once RigidBodyPlant supports it. For now, assume
      // joint_torque_measured == joint_torque_commanded.
      status.joint_torque_measured[i] = commanded_torque->GetAtIndex(i);
    }
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
