#include "drake/examples/planar_gripper/planar_gripper_lcm.h"

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_planar_gripper_command.hpp"
#include "drake/lcmt_planar_gripper_status.hpp"

namespace drake {
namespace examples {
namespace planar_gripper {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

GripperCommandDecoder::GripperCommandDecoder(int num_fingers) :
      num_fingers_(num_fingers), num_joints_(num_fingers * 2) {
  this->DeclareAbstractInputPort(
      "lcmt_planar_gripper_command",
      Value<lcmt_planar_gripper_command>{});
  state_output_port_ = &this->DeclareVectorOutputPort(
      "state", num_joints_ * 2, &GripperCommandDecoder::OutputStateCommand);
  torques_output_port_ = &this->DeclareVectorOutputPort(
      "torques", num_joints_, &GripperCommandDecoder::OutputTorqueCommand);
  this->DeclarePeriodicDiscreteUpdateEvent(
      kGripperLcmStatusPeriod, 0., &GripperCommandDecoder::UpdateDiscreteState);
  // Register a forced discrete state update event. It is added for unit test,
  // or for potential users who require forced updates.
  this->DeclareForcedDiscreteUpdateEvent(
      &GripperCommandDecoder::UpdateDiscreteState);
  // Discrete state holds pos, vel, torque.
  this->DeclareDiscreteState(num_joints_ * 3);
}

void GripperCommandDecoder::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>> pos) const {
  // The Discrete state consists of positions, velocities, torques.
  auto state_value =
      context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(pos.size() == num_joints_);
  // Set the initial positions.
  state_value.head(num_joints_) = pos;
  // Set the initial velocities and torques to zero.
  state_value.tail(num_joints_ * 2) = VectorX<double>::Zero(num_joints_ * 2);
}

systems::EventStatus GripperCommandDecoder::UpdateDiscreteState(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->get_value<lcmt_planar_gripper_command>();

  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  auto state_value = discrete_state->get_mutable_value(0);
  auto positions = state_value.head(num_joints_);
  auto velocities = state_value.segment(num_joints_, num_joints_);
  auto torques = state_value.tail(num_joints_);

  DRAKE_DEMAND(command.num_fingers == 0 || command.num_fingers == num_fingers_);

  for (int i = 0; i < command.num_fingers; ++i) {
    const lcmt_planar_gripper_finger_command& fcommand =
        command.finger_command[i];
    const int st_index = 2 * i;
    positions(st_index) = fcommand.joint_position[0];
    positions(st_index + 1) = fcommand.joint_position[1];
    velocities(st_index) = fcommand.joint_velocity[0];
    velocities(st_index + 1) = fcommand.joint_velocity[1];
    torques(st_index) = fcommand.joint_torque[0];
    torques(st_index + 1) = fcommand.joint_torque[1];
  }

  return systems::EventStatus::Succeeded();
}

void GripperCommandDecoder::OutputStateCommand(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0).value().head(num_joints_ * 2);
}

void GripperCommandDecoder::OutputTorqueCommand(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value().tail(num_joints_);
}

GripperCommandEncoder::GripperCommandEncoder(int num_fingers) :
      num_fingers_(num_fingers), num_joints_(num_fingers * 2) {
  state_input_port_ =
      &this->DeclareInputPort("state", systems::kVectorValued, num_joints_ * 2);
  torques_input_port_ =
      &this->DeclareInputPort("torque", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort("lcmt_gripper_command",
                                  &GripperCommandEncoder::OutputCommand);
}

void GripperCommandEncoder::OutputCommand(
    const Context<double>& context,
    lcmt_planar_gripper_command* command) const {
  command->utime = static_cast<int64_t>(context.get_time() * 1e6);
  const systems::BasicVector<double>* state_input =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* torque_input =
      this->EvalVectorInput(context, 1);

  command->num_fingers = num_fingers_;
  command->finger_command.resize(num_fingers_);

  for (int i = 0; i < num_fingers_; ++i) {
    const int st_index = 2 * i;
    lcmt_planar_gripper_finger_command& fcommand =
        command->finger_command[i];
    fcommand.joint_position[0] = state_input->GetAtIndex(st_index);
    fcommand.joint_position[1] = state_input->GetAtIndex(st_index + 1);
    fcommand.joint_velocity[0] =
        state_input->GetAtIndex(num_joints_ + st_index);
    fcommand.joint_velocity[1] =
        state_input->GetAtIndex(num_joints_ + st_index + 1);
    fcommand.joint_torque[0] = torque_input->GetAtIndex(st_index);
    fcommand.joint_torque[1] = torque_input->GetAtIndex(st_index + 1);
  }
}

GripperStatusDecoder::GripperStatusDecoder(int num_fingers)
    : num_fingers_(num_fingers),
      num_joints_(num_fingers * 2),
      num_tip_forces_(num_fingers * 2) {
  state_output_port_ = &this->DeclareVectorOutputPort(
      "state", num_joints_ * 2, &GripperStatusDecoder::OutputStateStatus);
  force_output_port_ =
      &this->DeclareVectorOutputPort("fingertip_force", num_tip_forces_,
                                     &GripperStatusDecoder::OutputForceStatus);
  this->DeclareAbstractInputPort("lcmt_planar_gripper_status",
                                 Value<lcmt_planar_gripper_status>{});
  // Discrete state includes: {state, fingertip_force(y,z)}.
  this->DeclareDiscreteState((num_joints_* 2) + (num_fingers_ * 2));

  this->DeclarePeriodicDiscreteUpdateEvent(
      kGripperLcmStatusPeriod, 0., &GripperStatusDecoder::UpdateDiscreteState);
  // Register a forced discrete state update event. It is added for unit test,
  // or for potential users who require forced updates.
  this->DeclareForcedDiscreteUpdateEvent(
      &GripperStatusDecoder::UpdateDiscreteState);
}

systems::EventStatus GripperStatusDecoder::UpdateDiscreteState(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->get_value<lcmt_planar_gripper_status>();

  auto state_value = discrete_state->get_mutable_value(0);
  auto positions = state_value.head(num_joints_);
  auto velocities = state_value.segment(num_joints_, num_joints_);
  auto tip_forces = state_value.tail(num_fingers_ * 2);

  DRAKE_DEMAND(status.num_fingers == 0 || status.num_fingers == num_fingers_);

  for (int i = 0; i < status.num_fingers; ++i) {
    const int st_index = 2 * i;
    const lcmt_planar_gripper_finger_status& fstatus = status.finger_status[i];
    positions(st_index) = fstatus.joint_position[0];
    positions(st_index + 1) = fstatus.joint_position[1];
    velocities(st_index) = fstatus.joint_velocity[0];
    velocities(st_index + 1) = fstatus.joint_velocity[1];
    tip_forces(st_index) = fstatus.fingertip_force.fy;
    tip_forces(st_index + 1) = fstatus.fingertip_force.fz;
  }

  return systems::EventStatus::Succeeded();
}

void GripperStatusDecoder::OutputStateStatus(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0).value().head(num_joints_ * 2);
}

void GripperStatusDecoder::OutputForceStatus(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0).value().tail(num_fingers_ * 2);
}

GripperStatusEncoder::GripperStatusEncoder(int num_fingers)
    : num_fingers_(num_fingers),
      num_joints_(num_fingers * 2),
      num_tip_forces_(num_fingers * 2) {
  state_input_port_ =
      &this->DeclareInputPort("state", systems::kVectorValued, num_joints_ * 2);
  force_input_port_ = &this->DeclareInputPort(
      "fingertip_force", systems::kVectorValued, num_tip_forces_);
  this->DeclareAbstractOutputPort("lcmt_gripper_status",
                                  &GripperStatusEncoder::OutputStatus);
}

void GripperStatusEncoder::OutputStatus(
    const Context<double>& context, lcmt_planar_gripper_status* status) const {
  status->utime = static_cast<int64_t>(context.get_time() * 1e6);
  const systems::BasicVector<double>* state_input =
      this->EvalVectorInput(context, 0);
  const VectorX<double>& state_value = state_input->value();
  const systems::BasicVector<double>* force_input =
      this->EvalVectorInput(context, 1);
  const VectorX<double>& force_value = force_input->value();

  status->num_fingers = num_fingers_;
  status->finger_status.resize(num_fingers_);
  for (int i = 0; i < num_fingers_; ++i) {
    const int st_index = 2 * i;
    lcmt_planar_gripper_finger_status& fstatus = status->finger_status[i];
    fstatus.joint_position[0] = state_value(st_index);
    fstatus.joint_position[1] = state_value(st_index + 1);
    fstatus.joint_velocity[0] =
        state_value(num_joints_ + st_index);
    fstatus.joint_velocity[1] =
        state_value(num_joints_ + st_index + 1);
    fstatus.fingertip_force.timestamp =
        static_cast<int64_t>(context.get_time() * 1e3);
    fstatus.fingertip_force.fy = force_value(st_index);
    fstatus.fingertip_force.fz = force_value(st_index + 1);

    // For the planar gripper, these are all zero.
    fstatus.fingertip_force.fx = 0;
    fstatus.fingertip_force.tx = 0;
    fstatus.fingertip_force.ty = 0;
    fstatus.fingertip_force.tz = 0;
  }
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
