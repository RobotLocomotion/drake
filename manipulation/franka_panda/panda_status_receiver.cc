#include "drake/manipulation/franka_panda/panda_status_receiver.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::lcmt_panda_status;
using drake::Value;
using drake::systems::BasicVector;
using drake::systems::Context;

PandaStatusReceiver::PandaStatusReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort("lcmt_panda_status",
                                 Value<lcmt_panda_status>{});
  this->DeclareVectorOutputPort(
      "position_commanded", BasicVector<double>(num_joints_),
      &PandaStatusReceiver::CalcLcmOutput<
          &lcmt_panda_status::joint_position_desired>);
  this->DeclareVectorOutputPort(
      "position_measured", BasicVector<double>(num_joints_),
      &PandaStatusReceiver::CalcLcmOutput<&lcmt_panda_status::joint_position>);
  this->DeclareVectorOutputPort(
      "velocity_commanded", BasicVector<double>(num_joints_),
      &PandaStatusReceiver::CalcLcmOutput<
          &lcmt_panda_status::joint_velocity_desired>);
  this->DeclareVectorOutputPort(
      "velocity_measured", BasicVector<double>(num_joints_),
      &PandaStatusReceiver::CalcLcmOutput<&lcmt_panda_status::joint_velocity>);
  this->DeclareVectorOutputPort(
      "acceleration_commanded", BasicVector<double>(num_joints_),
      &PandaStatusReceiver::CalcLcmOutput<
          &lcmt_panda_status::joint_acceleration_desired>);
  this->DeclareVectorOutputPort("torque_commanded",
                                BasicVector<double>(num_joints_),
                                &PandaStatusReceiver::CalcLcmOutput<
                                    &lcmt_panda_status::joint_torque_desired>);
  this->DeclareVectorOutputPort(
      "torque_measured", BasicVector<double>(num_joints_),
      &PandaStatusReceiver::CalcLcmOutput<&lcmt_panda_status::joint_torque>);
  this->DeclareVectorOutputPort("torque_external",
                                BasicVector<double>(num_joints_),
                                &PandaStatusReceiver::CalcLcmOutput<
                                    &lcmt_panda_status::joint_torque_external>);
}

PandaStatusReceiver::~PandaStatusReceiver() = default;

using OutPort = drake::systems::OutputPort<double>;
const OutPort& PandaStatusReceiver::get_position_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& PandaStatusReceiver::get_position_measured_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}
const OutPort& PandaStatusReceiver::get_velocity_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}
const OutPort& PandaStatusReceiver::get_velocity_measured_output_port() const {
  return LeafSystem<double>::get_output_port(3);
}
const OutPort& PandaStatusReceiver::get_acceleration_commanded_output_port()
    const {
  return LeafSystem<double>::get_output_port(4);
}
const OutPort& PandaStatusReceiver::get_torque_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(5);
}
const OutPort& PandaStatusReceiver::get_torque_measured_output_port() const {
  return LeafSystem<double>::get_output_port(6);
}
const OutPort& PandaStatusReceiver::get_torque_external_output_port() const {
  return LeafSystem<double>::get_output_port(7);
}

template <std::vector<double> drake::lcmt_panda_status::* field_ptr>
void PandaStatusReceiver::CalcLcmOutput(const Context<double>& context,
                                        BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_panda_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    const auto& status_field = status.*field_ptr;
    DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
    DRAKE_THROW_UNLESS(static_cast<int>(status_field.size()) == num_joints_);
    output->get_mutable_value() =
        Eigen::Map<const Eigen::VectorXd>(status_field.data(), num_joints_);
  }
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
