#include "drake/examples/kinova_jaco_arm/jaco_lcm.h"

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

namespace {
// Kinova's JACO urdf's have a 0-114 degree (0-2 radian) range for the
// finger joints, but the SDK's position values range from 0-6800
// degrees (0-118.68 radian), which I suspect is the speed of the
// motor driving the finger actuator (it's certainly not the angle of
// the finger joint).  Convert these as appropriate here.  I
// (sam.creasey) still don't think we wind up with the correct
// simulated position vs. where the actual fingers are at the same
// commanded value, but it's a start.
constexpr double kFingerSdkToUrdf = 2. / 118.68;
constexpr double kFingerUrdfToSdk = 118.68 / 2.;
}  // namespace

JacoCommandReceiver::JacoCommandReceiver(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(
      systems::BasicVector<double>((num_joints_ + num_fingers_) * 2),
      &JacoCommandReceiver::OutputCommand);
  this->DeclarePeriodicDiscreteUpdate(kJacoLcmStatusPeriod);
  this->DeclareDiscreteState((num_joints_ + num_fingers_) * 2);
}

void JacoCommandReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value =
      context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_ + num_fingers_);
  state_value.head(num_joints_ + num_fingers_) = x;
  state_value.tail(num_joints_ + num_fingers_) =
      VectorX<double>::Zero(num_joints_ + num_fingers_);
}

void JacoCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_jaco_command>();

  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  auto velocities = state_value.tail(num_joints_ + num_fingers_);

  DRAKE_DEMAND(command.num_joints == 0 || command.num_joints == num_joints_);
  DRAKE_DEMAND(command.num_fingers == 0 || command.num_fingers == num_fingers_);

  for (int i = 0; i < command.num_joints; ++i) {
    state_value(i) = command.joint_position[i];
    velocities(i) = command.joint_velocity[i];
  }

  for (int i = 0; i < command.num_fingers; ++i) {
    state_value(i + num_joints_) =
        command.finger_position[i] * kFingerSdkToUrdf;
    velocities(i + num_joints_) =
        command.finger_velocity[i] * kFingerSdkToUrdf;
  }
}

void JacoCommandReceiver::OutputCommand(const Context<double>& context,
                                        BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value();
}

JacoCommandSender::JacoCommandSender(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareInputPort(systems::kVectorValued,
                         (num_joints_ + num_fingers_) * 2);
  this->DeclareAbstractOutputPort(&JacoCommandSender::OutputCommand);
}

void JacoCommandSender::OutputCommand(
    const Context<double>& context, lcmt_jaco_command* output) const {

  output->utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* input =
      this->EvalVectorInput(context, 0);

  output->num_joints = num_joints_;
  output->joint_position.resize(num_joints_);
  output->joint_velocity.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    output->joint_position[i] = input->GetAtIndex(i);
    output->joint_velocity[i] = input->GetAtIndex(
        i + num_joints_ + num_fingers_);
  }

  output->num_fingers = num_fingers_;
  output->finger_position.resize(num_fingers_);
  output->finger_velocity.resize(num_fingers_);
  for (int i = 0; i < num_fingers_; ++i) {
    output->finger_position[i] =
        input->GetAtIndex(i + num_joints_) * kFingerUrdfToSdk;
    output->finger_velocity[i] = input->GetAtIndex(
        i + num_joints_ * 2 + num_fingers_)* kFingerUrdfToSdk;
  }
}

JacoStatusReceiver::JacoStatusReceiver(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareVectorOutputPort(
      systems::BasicVector<double>((num_joints_ + num_fingers_) * 2),
      &JacoStatusReceiver::OutputStatus);
  this->DeclareAbstractInputPort();
  this->DeclareDiscreteState((num_joints_ + num_fingers_)* 2);
  this->DeclarePeriodicDiscreteUpdate(kJacoLcmStatusPeriod);
}

void JacoStatusReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_jaco_status>();

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  auto velocities = state_value.tail(num_joints_ + num_fingers_);

  DRAKE_DEMAND(status.num_joints == 0 || status.num_joints == num_joints_);
  DRAKE_DEMAND(status.num_fingers == 0 || status.num_fingers == num_fingers_);

  for (int i = 0; i < status.num_joints; ++i) {
    state_value(i) = status.joint_position[i];
    // It seems like the Jaco reports half of the actual angular
    // velocity.  Fix that up here.  Note bug-for-bug compatibility
    // implemented in JacoStatusSender.
    velocities(i) = status.joint_velocity[i] * 2;
  }

  for (int i = 0; i < status.num_fingers; ++i) {
    state_value(i + num_joints_) =
        status.finger_position[i] * kFingerSdkToUrdf;
    // The reported finger velocities are completely bogus.  I
    // (sam.creasey) am not sure that passing them on here is even
    // useful.
    velocities(i + num_joints_) =
        status.finger_velocity[i] * kFingerSdkToUrdf;
  }
}

void JacoStatusReceiver::OutputStatus(const Context<double>& context,
                                      BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value();
}

JacoStatusSender::JacoStatusSender(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareInputPort(systems::kVectorValued,
                         (num_joints_ + num_fingers_) * 2);
  this->DeclareAbstractOutputPort(&JacoStatusSender::MakeOutputStatus,
                                  &JacoStatusSender::OutputStatus);
}

lcmt_jaco_status JacoStatusSender::MakeOutputStatus() const {
  lcmt_jaco_status msg{};
  msg.utime = 0;
  msg.num_joints = num_joints_;
  msg.num_fingers = num_fingers_;
  msg.joint_position.resize(msg.num_joints, 0);
  msg.joint_velocity.resize(msg.num_joints, 0);
  msg.joint_torque.resize(msg.num_joints, 0);
  msg.joint_current.resize(msg.num_joints, 0);
  msg.finger_position.resize(msg.num_fingers, 0);
  msg.finger_velocity.resize(msg.num_fingers, 0);
  msg.finger_torque.resize(msg.num_fingers, 0);
  msg.finger_current.resize(msg.num_fingers, 0);
  return msg;
}

void JacoStatusSender::OutputStatus(
    const Context<double>& context, lcmt_jaco_status* output) const {
  output->utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* input =
      this->EvalVectorInput(context, 0);

  output->num_joints = num_joints_;
  output->joint_position.resize(num_joints_);
  output->joint_velocity.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    output->joint_position[i] = input->GetAtIndex(i);
    // It seems like the Jaco reports half of the actual angular
    // velocity.  Simulate that here.  Note this is also handled in
    // JacoStatusReceiver.
    output->joint_velocity[i] =
        input->GetAtIndex(i + num_joints_ + num_fingers_) / 2;
  }

  output->num_fingers = num_fingers_;
  output->finger_position.resize(num_fingers_);
  output->finger_velocity.resize(num_fingers_);
  for (int i = 0; i < num_fingers_; ++i) {
    output->finger_position[i] =
        input->GetAtIndex(i + num_joints_) * kFingerUrdfToSdk;
    output->finger_velocity[i] = input->GetAtIndex(
        i + num_joints_ * 2 + num_fingers_) * kFingerUrdfToSdk;
  }
}

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
