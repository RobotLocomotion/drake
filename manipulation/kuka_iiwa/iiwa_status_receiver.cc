#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using systems::BasicVector;
using systems::Context;

IiwaStatusReceiver::IiwaStatusReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort(
      "lcmt_iiwa_status", Value<lcmt_iiwa_status>{});
  this->DeclareVectorOutputPort(
      "position_commanded", num_joints_,
      &IiwaStatusReceiver::CalcLcmOutput<
          &lcmt_iiwa_status::joint_position_commanded>);
  this->DeclareVectorOutputPort(
      "position_measured", num_joints_,
      &IiwaStatusReceiver::CalcLcmOutput<
          &lcmt_iiwa_status::joint_position_measured>);
  this->DeclareVectorOutputPort(
      "velocity_estimated", num_joints_,
      &IiwaStatusReceiver::CalcLcmOutput<
          &lcmt_iiwa_status::joint_velocity_estimated>);
  this->DeclareVectorOutputPort("torque_commanded", num_joints_,
                                &IiwaStatusReceiver::CalcLcmOutput<
                                    &lcmt_iiwa_status::joint_torque_commanded>);
  this->DeclareVectorOutputPort("torque_measured", num_joints_,
                                &IiwaStatusReceiver::CalcLcmOutput<
                                    &lcmt_iiwa_status::joint_torque_measured>);
  this->DeclareVectorOutputPort("torque_external", num_joints_,
                                &IiwaStatusReceiver::CalcLcmOutput<
                                    &lcmt_iiwa_status::joint_torque_external>);
}

IiwaStatusReceiver::~IiwaStatusReceiver() = default;

using OutPort = systems::OutputPort<double>;
const OutPort& IiwaStatusReceiver::get_position_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& IiwaStatusReceiver::get_position_measured_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}
const OutPort& IiwaStatusReceiver::get_velocity_estimated_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}
const OutPort& IiwaStatusReceiver::get_torque_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(3);
}
const OutPort& IiwaStatusReceiver::get_torque_measured_output_port() const {
  return LeafSystem<double>::get_output_port(4);
}
const OutPort& IiwaStatusReceiver::get_torque_external_output_port() const {
  return LeafSystem<double>::get_output_port(5);
}

template <std::vector<double> drake::lcmt_iiwa_status::* field_ptr>
void IiwaStatusReceiver::CalcLcmOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_iiwa_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    const auto& status_field = status.*field_ptr;
    DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
    DRAKE_THROW_UNLESS(static_cast<int>(status_field.size()) == num_joints_);
    output->get_mutable_value() = Eigen::Map<const Eigen::VectorXd>(
        status_field.data(), num_joints_);
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
