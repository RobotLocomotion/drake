#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using systems::BasicVector;
using systems::Context;

IiwaStatusReceiver::IiwaStatusReceiver(int num_joints)
    : num_joints_(num_joints),
      position_commanded_output_port_(
          this->DeclareVectorOutputPort(
                  "position_commanded",
                  BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_position_commanded>)
              .get_index()),
      position_measured_output_port_(
          this->DeclareVectorOutputPort(
                  "position_measured",
                  BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_position_measured>)
              .get_index()),
      velocity_estimated_output_port_(
          this->DeclareVectorOutputPort(
                  "velocity_estimated",
                  BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_velocity_estimated>)
              .get_index()),
      torque_commanded_output_port_(
          this->DeclareVectorOutputPort(
                  "torque_commanded", BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_torque_commanded>)
              .get_index()),
      torque_measured_output_port_(
          this->DeclareVectorOutputPort(
                  "torque_measured", BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_torque_measured>)
              .get_index()),
      torque_external_output_port_(
          this->DeclareVectorOutputPort(
                  "torque_external", BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::CopyLcmVectorOut<
                      &lcmt_iiwa_status::joint_torque_external>)
              .get_index()),
      state_output_port_(
          this->DeclareVectorOutputPort(
                  "state",
                  BasicVector<double>(num_joints_ * 2),
                  &IiwaStatusReceiver::OutputState)
              .get_index()) {
  this->DeclareAbstractInputPort(
      "lcmt_iiwa_status",
      Value<lcmt_iiwa_status>{});
}

template <std::vector<double> drake::lcmt_iiwa_status::* field>
void IiwaStatusReceiver::CopyLcmVectorOut(
    const Context<double>& context,
    BasicVector<double>* output) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    Eigen::VectorBlock<VectorX<double>> output_vec =
    output->get_mutable_value();
    for (int i = 0; i < num_joints_; ++i) {
      output_vec(i) = (status.*field)[i];
    }
  }
}

void IiwaStatusReceiver::OutputState(
    const Context<double>& context,
    BasicVector<double>* output) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    Eigen::VectorBlock<VectorX<double>> output_vec =
        output->get_mutable_value();
    for (int i = 0; i < num_joints_; ++i) {
      output_vec(i) = status.joint_position_measured[i];
      output_vec(i + num_joints_) = status.joint_velocity_estimated[i];
    }
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
