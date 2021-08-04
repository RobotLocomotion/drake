#include "drake/examples/allegro_hand/allegro_lcm.h"

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace allegro_hand {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

AllegroCommandReceiver::AllegroCommandReceiver(int num_joints,
                                               double lcm_period)
    : num_joints_(num_joints), lcm_period_(lcm_period) {
  DRAKE_THROW_UNLESS(lcm_period > 0);
  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<lcmt_allegro_command>{});
  state_output_port_ =
      this->DeclareVectorOutputPort(
              systems::kUseDefaultName, num_joints_ * 2,
              [this](const Context<double>& c, BasicVector<double>* o) {
                this->CopyStateToOutput(c, 0, num_joints_ * 2, o);
              })
          .get_index();
  torque_output_port_ =
      this->DeclareVectorOutputPort(
              systems::kUseDefaultName, num_joints_,
              [this](const Context<double>& c, BasicVector<double>* o) {
                this->CopyStateToOutput(c, num_joints_ * 2, num_joints_, o);
              })
          .get_index();
  this->DeclarePeriodicDiscreteUpdate(lcm_period_);
  // State + torque
  this->DeclareDiscreteState(num_joints_ * 3);
}

void AllegroCommandReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>>& x) const {
  auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.setZero();
  state_value.head(num_joints_) = x;
}

void AllegroCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->get_value<lcmt_allegro_command>();

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (command.num_joints != 0) {
    DRAKE_DEMAND(command.num_joints == num_joints_);
    VectorX<double> new_positions(num_joints_);
    for (int i = 0; i < command.num_joints; ++i) {
      new_positions(i) = command.joint_position[i];
    }

    state_value.segment(num_joints_, num_joints_).setZero();
    state_value.head(num_joints_) = new_positions;
  }

  // If the message does not contain torque commands, set torque command to
  // zeros.
  if (command.num_torques == 0) {
    state_value.tail(num_joints_).setZero();
  } else {
    DRAKE_DEMAND(command.num_torques == num_joints_);
    for (int i = 0; i < num_joints_; i++)
      state_value[2 * num_joints_ + i] = command.joint_torque[i];
  }
}

void AllegroCommandReceiver::CopyStateToOutput(
    const Context<double>& context, int start_idx, int length,
    BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec =
      context.get_discrete_state(0).get_value().segment(start_idx, length);
}

AllegroStatusSender::AllegroStatusSender(int num_joints)
    : num_joints_(num_joints) {
  // Commanded state.
  command_input_port_ = this->DeclareInputPort(
      systems::kUseDefaultName, systems::kVectorValued, num_joints_ * 2)
          .get_index();
  // Measured state.
  state_input_port_ = this->DeclareInputPort(
      systems::kUseDefaultName, systems::kVectorValued, num_joints_ * 2)
          .get_index();
  // Commanded torque.
  command_torque_input_port_ = this->DeclareInputPort(
      systems::kUseDefaultName, systems::kVectorValued, num_joints_)
          .get_index();

  this->DeclareAbstractOutputPort(systems::kUseDefaultName,
                                  &AllegroStatusSender::OutputStatus);
}

void AllegroStatusSender::OutputStatus(const Context<double>& context,
                                       lcmt_allegro_status* output) const {
  lcmt_allegro_status& status = *output;

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* state = this->EvalVectorInput(context, 1);
  const systems::BasicVector<double>* commanded_torque =
      this->EvalVectorInput(context, 2);

  status.num_joints = num_joints_;
  status.joint_position_measured.resize(num_joints_, 0);
  status.joint_velocity_estimated.resize(num_joints_, 0);
  status.joint_position_commanded.resize(num_joints_, 0);
  status.joint_torque_commanded.resize(num_joints_, 0);
  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_velocity_estimated[i] = state->GetAtIndex(i + num_joints_);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
    status.joint_torque_commanded[i] = commanded_torque->GetAtIndex(i);
  }
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
