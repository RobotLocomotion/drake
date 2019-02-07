#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

IiwaStatusSender::IiwaStatusSender(int num_joints)
    : num_joints_(num_joints),
      zero_vector_(Eigen::VectorXd::Zero(num_joints)) {
  this->DeclareInputPort(
      "position_commanded", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "position_measured", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "velocity_estimated", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque_commanded", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque_measured", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque_external", systems::kVectorValued, num_joints_);
  // TODO(jwnimmer-tri) Remove these two state inputs on 2019-05-01.
  this->DeclareInputPort(
      "state_commanded", systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort(
      "state_measured", systems::kVectorValued, num_joints_ * 2);
  this->DeclareAbstractOutputPort(
      "lcmt_iiwa_status", &IiwaStatusSender::CalcOutput);
}

using InPort = systems::InputPort<double>;
const InPort& IiwaStatusSender::get_position_commanded_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& IiwaStatusSender::get_position_measured_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
const InPort& IiwaStatusSender::get_velocity_estimated_input_port() const {
  return LeafSystem<double>::get_input_port(2);
}
const InPort& IiwaStatusSender::get_torque_commanded_input_port() const {
  return LeafSystem<double>::get_input_port(3);
}
const InPort& IiwaStatusSender::get_torque_measured_input_port() const {
  return LeafSystem<double>::get_input_port(4);
}
const InPort& IiwaStatusSender::get_torque_external_input_port() const {
  return LeafSystem<double>::get_input_port(5);
}
const InPort& IiwaStatusSender::get_command_input_port() const {
  return LeafSystem<double>::get_input_port(6);
}
const InPort& IiwaStatusSender::get_state_input_port() const {
  return LeafSystem<double>::get_input_port(7);
}
const systems::OutputPort<double>& IiwaStatusSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void IiwaStatusSender::CalcOutput(
    const systems::Context<double>& context, lcmt_iiwa_status* output) const {
  const auto* const position_commanded_in = this->EvalVectorInput(context, 0);
  const auto* const position_measured_in = this->EvalVectorInput(context, 1);
  const auto* const velocity_estimated_in = this->EvalVectorInput(context, 2);
  const auto* const torque_commanded_in = this->EvalVectorInput(context, 3);
  const auto* const torque_measured_in = this->EvalVectorInput(context, 4);
  const auto* const torque_external_in = this->EvalVectorInput(context, 5);
  const auto* const state_commanded_in = this->EvalVectorInput(context, 6);
  const auto* const state_measured_in = this->EvalVectorInput(context, 7);

  // Check that required inputs are present.
  DRAKE_THROW_UNLESS(position_commanded_in || state_commanded_in);
  DRAKE_THROW_UNLESS(position_measured_in || state_measured_in);
  DRAKE_THROW_UNLESS(torque_commanded_in);
  // Check that one-or-the-other inputs are not doubly-connected.
  DRAKE_THROW_UNLESS(!(position_commanded_in && state_commanded_in));
  DRAKE_THROW_UNLESS(!(position_measured_in && state_measured_in));
  DRAKE_THROW_UNLESS(!(velocity_estimated_in && state_measured_in));
  // For each output quantity, choose which input data to use.
  const auto& position_commanded =
      position_commanded_in ? *position_commanded_in : *state_commanded_in;
  const auto& position_measured =
      position_measured_in ? *position_measured_in : *state_measured_in;
  const auto& velocity_estimated =
      velocity_estimated_in ? velocity_estimated_in->CopyToVector() :
      state_measured_in ? Eigen::VectorXd(
          state_measured_in->CopyToVector().tail(num_joints_)) :
      zero_vector_.CopyToVector();
  const auto& torque_commanded =
      *torque_commanded_in;
  const auto& torque_measured =
      torque_measured_in ? *torque_measured_in : torque_commanded;
  const auto& torque_external =
      torque_external_in ? *torque_external_in : zero_vector_;

  lcmt_iiwa_status& status = *output;
  status.utime = context.get_time() * 1e6;
  status.num_joints = num_joints_;
  status.joint_position_measured.resize(num_joints_, 0);
  status.joint_velocity_estimated.resize(num_joints_, 0);
  status.joint_position_commanded.resize(num_joints_, 0);
  status.joint_position_ipo.resize(num_joints_, 0);
  status.joint_torque_measured.resize(num_joints_, 0);
  status.joint_torque_commanded.resize(num_joints_, 0);
  status.joint_torque_external.resize(num_joints_, 0);
  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = position_measured[i];
    status.joint_velocity_estimated[i] = velocity_estimated[i];
    status.joint_position_commanded[i] = position_commanded[i];
    status.joint_torque_commanded[i] = torque_commanded[i];
    status.joint_torque_measured[i] = torque_measured[i];
    status.joint_torque_external[i] = torque_external[i];
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
