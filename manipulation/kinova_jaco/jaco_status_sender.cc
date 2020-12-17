#include "drake/manipulation/kinova_jaco/jaco_status_sender.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

JacoStatusSender::JacoStatusSender(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareInputPort(
      "state", systems::kVectorValued, (num_joints_ + num_fingers_) * 2);
  this->DeclareInputPort(
      "torque", systems::kVectorValued, num_joints_ + num_fingers_);
  this->DeclareInputPort(
      "torque_external", systems::kVectorValued, num_joints_ + num_fingers_);
  this->DeclareInputPort(
      "current", systems::kVectorValued, num_joints_ + num_fingers_);
  this->DeclareAbstractOutputPort(
      "lcmt_jaco_status", &JacoStatusSender::CalcOutput);
}

using InPort = systems::InputPort<double>;
const InPort& JacoStatusSender::get_state_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& JacoStatusSender::get_torque_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
const InPort& JacoStatusSender::get_torque_external_input_port() const {
  return LeafSystem<double>::get_input_port(2);
}
const InPort& JacoStatusSender::get_current_input_port() const {
  return LeafSystem<double>::get_input_port(3);
}

void JacoStatusSender::CalcOutput(
    const systems::Context<double>& context, lcmt_jaco_status* output) const {
  const Eigen::VectorXd zero_state =
      Eigen::VectorXd::Zero(num_joints_ + num_fingers_);
  const auto& state =
      get_state_input_port().Eval(context);
  const auto& torque =
      get_torque_input_port().HasValue(context) ?
      get_torque_input_port().Eval(context) :
      zero_state;
  const auto& torque_external =
      get_torque_external_input_port().HasValue(context) ?
      get_torque_external_input_port().Eval(context) :
      zero_state;
  const auto& current =
      get_current_input_port().HasValue(context) ?
      get_current_input_port().Eval(context) :
      zero_state;

  lcmt_jaco_status& status = *output;
  status.utime = context.get_time() * 1e6;

  status.num_joints = num_joints_;
  status.joint_position.resize(num_joints_, 0);
  status.joint_velocity.resize(num_joints_, 0);
  status.joint_torque.resize(num_joints_, 0);
  status.joint_torque_external.resize(num_joints_, 0);
  status.joint_current.resize(num_joints_, 0);
  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position[i] = state(i);
    // It seems like the Jaco reports half of the actual angular
    // velocity.  Fix that up here.  Note bug-for-bug compatibility
    // implemented in JacoStatusReceiver.
    status.joint_velocity[i] = state(i + num_joints_ + num_fingers_) / 2;
    status.joint_torque[i] = torque(i);
    status.joint_torque_external[i] = torque_external(i);
    status.joint_current[i] = current(i);
  }

  status.num_fingers = num_fingers_;
  status.finger_position.resize(num_fingers_, 0);
  status.finger_velocity.resize(num_fingers_, 0);
  status.finger_torque.resize(num_fingers_, 0);
  status.finger_torque_external.resize(num_fingers_, 0);
  status.finger_current.resize(num_fingers_, 0);
  for (int i = 0; i < num_fingers_; ++i) {
    status.finger_position[i] = state(i + num_joints_) * kFingerUrdfToSdk;
    status.finger_velocity[i] = state.tail(num_fingers_)(i) * kFingerUrdfToSdk;
    status.finger_torque[i] = torque(i + num_joints_);
    status.finger_torque_external[i] = torque_external(i + num_joints_);
    status.finger_current[i] = current(i + num_joints_);
  }
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
