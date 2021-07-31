#include "drake/manipulation/kinova_jaco/jaco_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

using systems::BasicVector;
using systems::Context;

JacoStatusReceiver::JacoStatusReceiver(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareAbstractInputPort(
      "lcmt_jaco_status", Value<lcmt_jaco_status>{});
  this->DeclareVectorOutputPort("state", (num_joints_ + num_fingers_) * 2,
                                &JacoStatusReceiver::CalcStateOutput);
  this->DeclareVectorOutputPort(
      "torque", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcLcmOutput<&lcmt_jaco_status::joint_torque,
                                         &lcmt_jaco_status::finger_torque>);
  this->DeclareVectorOutputPort("torque_external", num_joints_ + num_fingers_,
                                &JacoStatusReceiver::CalcLcmOutput<
                                    &lcmt_jaco_status::joint_torque_external,
                                    &lcmt_jaco_status::finger_torque_external>);
  this->DeclareVectorOutputPort(
      "current", num_joints_ + num_fingers_,
      &JacoStatusReceiver::CalcLcmOutput<&lcmt_jaco_status::joint_current,
                                         &lcmt_jaco_status::finger_current>);
}

using OutPort = systems::OutputPort<double>;
const OutPort& JacoStatusReceiver::get_state_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& JacoStatusReceiver::get_torque_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}
const OutPort& JacoStatusReceiver::get_torque_external_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}
const OutPort& JacoStatusReceiver::get_current_output_port() const {
  return LeafSystem<double>::get_output_port(3);
}

void JacoStatusReceiver::CalcStateOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_jaco_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
    return;
  }

  Eigen::VectorXd state((num_joints_ + num_fingers_) * 2);
  for (int i = 0; i < status.num_joints; ++i) {
    state(i) = status.joint_position[i];
    // It seems like the Jaco reports half of the actual angular
    // velocity.  Fix that up here.  Note bug-for-bug compatibility
    // implemented in JacoStatusSender.
    state.segment(num_joints_ + num_fingers_, num_joints_)(i) =
        status.joint_velocity[i] * 2;
  }

  for (int i = 0; i < status.num_fingers; ++i) {
    state(i + num_joints_) = status.finger_position[i] * kFingerSdkToUrdf;
    // The reported finger velocities are completely bogus.  I
    // (sam.creasey) am not sure that passing them on here is even
    // useful.
    state.tail(num_fingers_)(i) =
        status.finger_velocity[i] * kFingerSdkToUrdf;
  }
  output->get_mutable_value() = state;
}

template <std::vector<double> drake::lcmt_jaco_status::* arm_ptr,
          std::vector<double> drake::lcmt_jaco_status::* finger_ptr>
void JacoStatusReceiver::CalcLcmOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_jaco_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
    return;
  }

  Eigen::VectorXd state(num_joints_ + num_fingers_);
  const auto& arm_field = status.*arm_ptr;
  state.head(num_joints_) = Eigen::Map<const Eigen::VectorXd>(
      arm_field.data(), arm_field.size());

  const auto& finger_field = status.*finger_ptr;
  state.tail(num_fingers_) = Eigen::Map<const Eigen::VectorXd>(
      finger_field.data(), finger_field.size());
  output->get_mutable_value() = state;
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
