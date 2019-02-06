#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using systems::BasicVector;
using systems::Context;

IiwaCommandSender::IiwaCommandSender(int num_joints)
    : num_joints_(num_joints),
      position_input_port_(
          this->DeclareInputPort(systems::kVectorValued, num_joints_)
              .get_index()),
      torque_input_port_(
          this->DeclareInputPort(systems::kVectorValued, num_joints_)
              .get_index()) {
  this->DeclareAbstractOutputPort(&IiwaCommandSender::OutputCommand);
}

void IiwaCommandSender::OutputCommand(const Context<double>& context,
                                      lcmt_iiwa_command* output) const {
  lcmt_iiwa_command& command = *output;

  command.utime = context.get_time() * 1e6;
  const BasicVector<double>* positions = this->EvalVectorInput(context, 0);

  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = positions->GetAtIndex(i);
  }

  const BasicVector<double>* torques = this->EvalVectorInput(context, 1);
  if (torques == nullptr) {
    command.num_torques = 0;
    command.joint_torque.clear();
  } else {
    command.num_torques = num_joints_;
    command.joint_torque.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i) {
      command.joint_torque[i] = torques->GetAtIndex(i);
    }
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
