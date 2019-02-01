#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/internal_iiwa_command_translator.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using internal::IiwaCommand;
using internal::IiwaCommandTranslator;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints)
    : num_joints_(num_joints) {
  lcmt_iiwa_command uninitialized_message{};
  uninitialized_message.utime =
      static_cast<uint64_t>(IiwaCommand<double>::kUnitializedTime);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // TODO(jwnimmer-tri) Remove this input port after 2019-03-01.
  this->DeclareVectorInputPort("command_vector",
                               IiwaCommand<double>(num_joints));
#pragma GCC diagnostic pop
  this->DeclareAbstractInputPort(
      "command_message",
      Value<lcmt_iiwa_command>(uninitialized_message));

  this->DeclareVectorOutputPort(
      "state", BasicVector<double>(num_joints_ * 2),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 0, num_joints_ * 2, o);
      });
  this->DeclareVectorOutputPort(
      "feedforward_torque", BasicVector<double>(num_joints_),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, num_joints_ * 2, num_joints_, o);
      });
  this->DeclarePeriodicDiscreteUpdate(kIiwaLcmStatusPeriod);
  // State + torque
  this->DeclareDiscreteState(num_joints_ * 3);
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.setZero();
  state_value.head(num_joints_) = x;
}

void IiwaCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const IiwaCommand<double>* command =
      this->template EvalVectorInput<IiwaCommand>(context, 0);

  // If the vector input port is not wired, try the abstract value one.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  IiwaCommand<double> decoded_command(num_joints_);
#pragma GCC diagnostic pop
  if (!command) {
    const AbstractValue* input = this->EvalAbstractInput(context, 1);
    DRAKE_THROW_UNLESS(input != nullptr);
    const auto& command_msg = input->GetValue<lcmt_iiwa_command>();
    std::vector<uint8_t> bytes(command_msg.getEncodedSize());
    const int encoded_size = command_msg.encode(bytes.data(), 0, bytes.size());
    DRAKE_DEMAND(encoded_size == static_cast<int>(bytes.size()));
    if (command_msg.num_joints == 0) {
      // This can happen if we haven't received a message yet.
      return;
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const IiwaCommandTranslator translator(num_joints_);
#pragma GCC diagnostic pop
    translator.Deserialize(bytes.data(), bytes.size(), &decoded_command);
    command = &decoded_command;
  }
  DRAKE_THROW_UNLESS(command);

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  // Haven't received a legit message yet.
  if (command->utime() == IiwaCommand<double>::kUnitializedTime) {
    return;
  }

  state_value.segment(num_joints_, num_joints_) =
      (command->joint_position() - state_value.head(num_joints_)) /
      kIiwaLcmStatusPeriod;
  state_value.head(num_joints_) = command->joint_position();
  state_value.tail(num_joints_) = command->joint_torque();
}

void IiwaCommandReceiver::CopyStateToOutput(const Context<double>& context,
                                            int start_idx, int length,
                                            BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec =
      context.get_discrete_state(0).get_value().segment(start_idx, length);
}

std::unique_ptr<systems::lcm::LcmSubscriberSystem>
MakeIiwaCommandLcmSubscriberSystem(
    int num_joints, const std::string& channel,
    drake::lcm::DrakeLcmInterface* lcm) {
  drake::lcmt_iiwa_command message_size_exemplar;
  message_size_exemplar.num_joints = num_joints;
  message_size_exemplar.joint_position.resize(num_joints);
  message_size_exemplar.num_torques = num_joints;
  message_size_exemplar.joint_torque.resize(num_joints);
  return systems::lcm::LcmSubscriberSystem::MakeFixedSize(
      message_size_exemplar, channel, lcm);
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
