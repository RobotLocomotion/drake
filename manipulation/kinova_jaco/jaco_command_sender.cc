#include "drake/manipulation/kinova_jaco/jaco_command_sender.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

using drake::systems::kVectorValued;

JacoCommandSender::JacoCommandSender(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  state_input_ = &DeclareInputPort(
      "state", kVectorValued, (num_joints_ + num_fingers_) * 2);
  position_input_ = &DeclareInputPort(
      "position", kVectorValued, num_joints_ + num_fingers_);
  velocity_input_ = &DeclareInputPort(
      "velocity", kVectorValued, num_joints_ + num_fingers_);

  this->DeclareAbstractOutputPort(
      "lcmt_jaco_command", &JacoCommandSender::CalcOutput);
}

void JacoCommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_jaco_command* output) const {

  output->utime = context.get_time() * 1e6;
  output->num_joints = num_joints_;
  output->joint_position.resize(num_joints_);
  output->joint_velocity.resize(num_joints_);

  output->num_fingers = num_fingers_;
  output->finger_position.resize(num_fingers_);
  output->finger_velocity.resize(num_fingers_);

  if (state_input_->HasValue(context)) {
    DRAKE_DEMAND(!position_input_->HasValue(context));
    DRAKE_DEMAND(!velocity_input_->HasValue(context));

    const auto& state = state_input_->Eval(context);
    for (int i = 0; i < num_joints_; ++i) {
      output->joint_position[i] = state(i);
      output->joint_velocity[i] = state(i + num_joints_ + num_fingers_);
    }

    for (int i = 0; i < num_fingers_; ++i) {
      output->finger_position[i] = state(i + num_joints_) * kFingerUrdfToSdk;
      output->finger_velocity[i] =
          state.tail(num_fingers_)(i) * kFingerUrdfToSdk;
    }
    return;
  }

  const auto& position = position_input_->Eval(context);
  const auto& velocity = velocity_input_->Eval(context);
  for (int i = 0; i < num_joints_; ++i) {
    output->joint_position[i] = position(i);
    output->joint_velocity[i] = velocity(i);
  }

  for (int i = 0; i < num_fingers_; ++i) {
    output->finger_position[i] =
        position(num_joints_ + i) * kFingerUrdfToSdk;
    output->finger_velocity[i] =
        velocity(num_joints_ + i) * kFingerUrdfToSdk;
  }
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
