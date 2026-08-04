#include "drake/manipulation/kinova_jaco/jaco_status_sender.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

using drake::systems::kVectorValued;

JacoStatusSender::JacoStatusSender(int num_joints, int num_fingers)
    : num_joints_(num_joints), num_fingers_(num_fingers) {
  position_input_ =
      &DeclareInputPort("position", kVectorValued, num_joints_ + num_fingers_);
  velocity_input_ =
      &DeclareInputPort("velocity", kVectorValued, num_joints_ + num_fingers_);
  torque_input_ =
      &DeclareInputPort("torque", kVectorValued, num_joints_ + num_fingers_);
  torque_external_input_ = &DeclareInputPort("torque_external", kVectorValued,
                                             num_joints_ + num_fingers_);
  current_input_ =
      &DeclareInputPort("current", kVectorValued, num_joints_ + num_fingers_);
  time_measured_input_ = &DeclareInputPort("time_measured", kVectorValued, 1);
  DeclareAbstractOutputPort("lcmt_jaco_status", &JacoStatusSender::CalcOutput);
}

void JacoStatusSender::CalcOutput(const systems::Context<double>& context,
                                  lcmt_jaco_status* output) const {
  const double time_measured =
      get_time_measured_input_port().HasValue(context)
          ? get_time_measured_input_port().Eval(context)[0]
          : context.get_time();

  const Eigen::VectorXd zero_state =
      Eigen::VectorXd::Zero(num_joints_ + num_fingers_);
  const auto& torque = get_torque_input_port().HasValue(context)
                           ? get_torque_input_port().Eval(context)
                           : zero_state;
  const auto& torque_external =
      get_torque_external_input_port().HasValue(context)
          ? get_torque_external_input_port().Eval(context)
          : zero_state;
  const auto& current = get_current_input_port().HasValue(context)
                            ? get_current_input_port().Eval(context)
                            : zero_state;

  lcmt_jaco_status& status = *output;
  status.utime = time_measured * 1e6;

  status.num_joints = num_joints_;
  status.joint_position.resize(num_joints_, 0);
  status.joint_velocity.resize(num_joints_, 0);
  status.joint_torque.resize(num_joints_, 0);
  status.joint_torque_external.resize(num_joints_, 0);
  status.joint_current.resize(num_joints_, 0);

  status.num_fingers = num_fingers_;
  status.finger_position.resize(num_fingers_, 0);
  status.finger_velocity.resize(num_fingers_, 0);
  status.finger_torque.resize(num_fingers_, 0);
  status.finger_torque_external.resize(num_fingers_, 0);
  status.finger_current.resize(num_fingers_, 0);

  const auto& position = position_input_->Eval(context);
  const auto& velocity = velocity_input_->Eval(context);

  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position[i] = position(i);
    // It seems like the Jaco (at least for some models/firmwares) reports
    // half of the actual angular velocity.  Fix that up here.  Note
    // bug-for-bug compatibility implemented in JacoStatusReceiver.
    status.joint_velocity[i] = velocity(i) / 2;
    status.joint_torque[i] = torque(i);
    status.joint_torque_external[i] = torque_external(i);
    status.joint_current[i] = current(i);
  }

  for (int i = 0; i < num_fingers_; ++i) {
    status.finger_position[i] = position(num_joints_ + i) * kFingerUrdfToSdk;
    status.finger_velocity[i] = velocity(num_joints_ + i) * kFingerUrdfToSdk;
    status.finger_torque[i] = torque(num_joints_ + i);
    status.finger_torque_external[i] = torque_external(num_joints_ + i);
    status.finger_current[i] = current(num_joints_ + i);
  }
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
