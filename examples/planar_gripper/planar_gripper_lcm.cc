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
      systems::kUseDefaultName,
      Value<lcmt_planar_gripper_command>{});
  this->DeclareVectorOutputPort("state",
      systems::BasicVector<double>(num_joints_ * 2),
      &GripperCommandDecoder::OutputStateCommand);
  this->DeclareVectorOutputPort("torques",
      systems::BasicVector<double>(num_joints_),
      &GripperCommandDecoder::OutputTorqueCommand);
  this->DeclarePeriodicDiscreteUpdateEvent(
      kGripperLcmStatusPeriod, 0., &GripperCommandDecoder::UpdateDiscreteState);
  // Register a forced discrete state update event. It is added for unit test,
  // or for potential users who require forced updates.
  this->DeclareForcedDiscreteUpdateEvent(
      &GripperCommandDecoder::UpdateDiscreteState);
  // discrete state holds pos, vel, torque.
  this->DeclareDiscreteState(num_joints_ * 3);
}

void GripperCommandDecoder::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value =
      context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_ * 3);  // pos, vel, torque.
  state_value.head(num_joints_) = x;
  state_value.segment(num_joints_, num_joints_) =
      VectorX<double>::Zero(num_joints_);
  state_value.tail(num_joints_) = VectorX<double>::Zero(num_joints_);
}

systems::EventStatus GripperCommandDecoder::UpdateDiscreteState(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->get_value<lcmt_planar_gripper_command>();

  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  auto positions = state_value.head(num_joints_);
  auto velocities = state_value.segment(num_joints_, num_joints_);
  auto torques = state_value.tail(num_joints_);

  DRAKE_DEMAND(command.num_fingers == 0 || command.num_fingers == num_fingers_);

  for (int i = 0; i < command.num_fingers; ++i) {
    auto st_index = 2 * i;
    positions(st_index) = command.finger_command[i].joint_position[0];
    positions(st_index + 1) = command.finger_command[i].joint_position[1];
    velocities(st_index) = command.finger_command[i].joint_velocity[0];
    velocities(st_index + 1) = command.finger_command[i].joint_velocity[1];
    torques(st_index) = command.finger_command[i].joint_torque[0];
    torques(st_index + 1) = command.finger_command[i].joint_torque[1];
  }

  return systems::EventStatus::Succeeded();
}

void GripperCommandDecoder::OutputStateCommand(const Context<double>& context,
                                        BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value().head(num_joints_ * 2);
}

void GripperCommandDecoder::OutputTorqueCommand(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value().tail(num_joints_);
}

GripperCommandEncoder::GripperCommandEncoder(int num_fingers) :
      num_fingers_(num_fingers), num_joints_(num_fingers * 2) {
  this->DeclareInputPort("state", systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort("torque", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(&GripperCommandEncoder::OutputCommand);
}

void GripperCommandEncoder::OutputCommand(
    const Context<double>& context, lcmt_planar_gripper_command* output) const {

  output->utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* state_input =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* torque_input =
      this->EvalVectorInput(context, 1);

  output->num_fingers = num_fingers_;
  output->finger_command.resize(num_fingers_);

  for (int i = 0; i < num_fingers_; ++i) {
    auto st_indx = 2 * i;
    output->finger_command[i].joint_position[0] =
        state_input->GetAtIndex(st_indx);
    output->finger_command[i].joint_position[1] =
        state_input->GetAtIndex(st_indx + 1);
    output->finger_command[i].joint_velocity[0] =
        state_input->GetAtIndex(num_joints_ + st_indx);
    output->finger_command[i].joint_velocity[1] =
        state_input->GetAtIndex(num_joints_ + st_indx + 1);
    output->finger_command[i].joint_torque[0] =
        torque_input->GetAtIndex(st_indx);
    output->finger_command[i].joint_torque[1] =
        torque_input->GetAtIndex(st_indx + 1);
  }
}

GripperStatusDecoder::GripperStatusDecoder(int num_fingers) :
      num_fingers_(num_fingers), num_joints_(num_fingers * 2) {
  this->DeclareVectorOutputPort("state",
      systems::BasicVector<double>(num_joints_ * 2),
      &GripperStatusDecoder::OutputStateStatus);
  this->DeclareVectorOutputPort("fingertip_force",
      systems::BasicVector<double>(num_fingers_ * 2),
      &GripperStatusDecoder::OutputForceStatus);
  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<lcmt_planar_gripper_status>{});
  // discrete state includes: {state, fingertip_force(y,z)}
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

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);

  auto state_value = state.get_mutable_value();
  auto positions = state_value.head(num_joints_);
  auto velocities = state_value.segment(num_joints_, num_joints_);
  auto tip_forces = state_value.tail(num_fingers_ * 2);

  DRAKE_DEMAND(status.num_fingers == 0 || status.num_fingers == num_fingers_);

  for (int i = 0; i < status.num_fingers; ++i) {
    auto st_index = 2 * i;
    positions(st_index) = status.finger_status[i].joint_position[0];
    positions(st_index + 1) = status.finger_status[i].joint_position[1];
    velocities(st_index) = status.finger_status[i].joint_velocity[0];
    velocities(st_index + 1) = status.finger_status[i].joint_velocity[1];
    tip_forces(st_index) = status.finger_status[i].fingertip_force.fy;
    tip_forces(st_index + 1) = status.finger_status[i].fingertip_force.fz;
  }

  return systems::EventStatus::Succeeded();
}

void GripperStatusDecoder::OutputStateStatus(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value().head(num_joints_ * 2);
}

void GripperStatusDecoder::OutputForceStatus(
    const Context<double>& context, BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value().tail(num_fingers_ * 2);
}

GripperStatusEncoder::GripperStatusEncoder(int num_fingers)
    : num_fingers_(num_fingers), num_joints_(num_fingers * 2) {
  this->DeclareInputPort("state", systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort("fingertip_force", systems::kVectorValued,
                         num_fingers_ * 2);
  this->DeclareAbstractOutputPort(&GripperStatusEncoder::MakeOutputStatus,
                                  &GripperStatusEncoder::OutputStatus);
}

lcmt_planar_gripper_status GripperStatusEncoder::MakeOutputStatus() const {
  lcmt_planar_gripper_status msg{};
  msg.utime = 0;
  msg.num_fingers = num_fingers_;
  msg.finger_status.resize(num_fingers_);
  return msg;
}

void GripperStatusEncoder::OutputStatus(
    const Context<double>& context, lcmt_planar_gripper_status* output) const {
  output->utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* state_input =
      this->EvalVectorInput(context, 0);
  auto state_value = state_input->get_value();
  const systems::BasicVector<double>* force_input =
      this->EvalVectorInput(context, 1);
  auto force_value = force_input->get_value();

  output->num_fingers = num_fingers_;
  output->finger_status.resize(num_fingers_);
  for (int i = 0; i < num_fingers_; ++i) {
    auto st_index = 2 * i;
    output->finger_status[i].joint_position[0] = state_value(st_index);
    output->finger_status[i].joint_position[1] = state_value(st_index + 1);
    output->finger_status[i].joint_velocity[0] =
        state_value(num_joints_ + st_index);
    output->finger_status[i].joint_velocity[1] =
        state_value(num_joints_ + st_index + 1);
    output->finger_status[i].fingertip_force.fy = force_value(st_index);
    output->finger_status[i].fingertip_force.fz = force_value(st_index + 1);

    // For the planar gripper, these are all zero.
    output->finger_status[i].fingertip_force.fx = 0;
    output->finger_status[i].fingertip_force.tx = 0;
    output->finger_status[i].fingertip_force.ty = 0;
    output->finger_status[i].fingertip_force.tz = 0;
  }
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
