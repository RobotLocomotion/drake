#include "drake/manipulation/kinova_jaco/jaco_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

using systems::BasicVector;
using systems::Context;

JacoStatusReceiver::JacoStatusReceiver(int num_joints, int num_fingers)
    : num_joints_(num_joints), num_fingers_(num_fingers) {
  message_input_ =
      &DeclareAbstractInputPort("lcmt_jaco_status", Value<lcmt_jaco_status>{});
  position_measured_output_ = &DeclareVectorOutputPort(
      "position_measured", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcJointOutput<&lcmt_jaco_status::joint_position,
                                           &lcmt_jaco_status::finger_position,
                                           1>);
  velocity_measured_output_ = &DeclareVectorOutputPort(
      "velocity_measured", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcJointOutput<&lcmt_jaco_status::joint_velocity,
                                           &lcmt_jaco_status::finger_velocity,
                                           1>);
  torque_measured_output_ = &DeclareVectorOutputPort(
      "torque_measured", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcJointOutput<&lcmt_jaco_status::joint_torque,
                                           &lcmt_jaco_status::finger_torque,
                                           0>);
  torque_external_output_ = &DeclareVectorOutputPort(
      "torque_external", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcJointOutput<
          &lcmt_jaco_status::joint_torque_external,
          &lcmt_jaco_status::finger_torque_external, 0>);
  current_output_ = &DeclareVectorOutputPort(
      "current", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcJointOutput<&lcmt_jaco_status::joint_current,
                                           &lcmt_jaco_status::finger_current,
                                           0>);
  // Declared after the previous points to preserve port numbers (even though
  // this isn't a guaranteed stable part of the API).
  time_measured_output_ = &DeclareVectorOutputPort(
      "time_measured", 1, &JacoStatusReceiver::CalcTimeOutput);
}

template <std::vector<double> drake::lcmt_jaco_status::* arm_ptr,
          std::vector<double> drake::lcmt_jaco_status::* finger_ptr,
          int finger_scale>
void JacoStatusReceiver::CalcJointOutput(const Context<double>& context,
                                         BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_jaco_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
    return;
  }

  Eigen::VectorXd output_vec(num_joints_ + num_fingers_);
  const auto& arm_field = status.*arm_ptr;
  output_vec.head(num_joints_) =
      Eigen::Map<const Eigen::VectorXd>(arm_field.data(), arm_field.size());
  if (num_fingers_) {
    constexpr double scale_factor = finger_scale ? kFingerSdkToUrdf : 1;
    const auto& finger_field = status.*finger_ptr;
    output_vec.tail(num_fingers_) =
        Eigen::Map<const Eigen::VectorXd>(finger_field.data(),
                                          finger_field.size()) *
        scale_factor;
  }
  output->SetFromVector(output_vec);
}

void JacoStatusReceiver::CalcTimeOutput(const Context<double>& context,
                                        BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_jaco_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    (*output)[0] = static_cast<double>(status.utime) / 1e6;
  }
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
