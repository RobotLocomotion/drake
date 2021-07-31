#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/lcm_messages.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using systems::BasicVector;
using systems::Context;

SchunkWsgCommandReceiver::SchunkWsgCommandReceiver(double initial_position,
                                                   double initial_force)
    : initial_position_(initial_position), initial_force_(initial_force) {
  this->DeclareVectorOutputPort("position", 1,
                                &SchunkWsgCommandReceiver::CalcPositionOutput);
  this->DeclareVectorOutputPort(
      "force_limit", 1, &SchunkWsgCommandReceiver::CalcForceLimitOutput);

  lcmt_schunk_wsg_command uninitialized_message{};
  this->DeclareAbstractInputPort(
      "command_message",
      Value<lcmt_schunk_wsg_command>(uninitialized_message));
}

void SchunkWsgCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message =
      this->get_input_port(0).Eval<lcmt_schunk_wsg_command>(context);

  double target_position = initial_position_;
  // N.B. This works due to lcm::Serializer<>::CreateDefaultValue() using
  // value-initialization.
  if (!lcm::AreLcmMessagesEqual(message, lcmt_schunk_wsg_command{})) {
    target_position = message.target_position_mm / 1e3;
    if (std::isnan(target_position)) {
      target_position = 0;
    }
  }

  output->SetAtIndex(0, target_position);
}

void SchunkWsgCommandReceiver::CalcForceLimitOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message =
      this->get_input_port(0).Eval<lcmt_schunk_wsg_command>(context);

  double force_limit = initial_force_;
  // N.B. This works due to lcm::Serializer<>::CreateDefaultValue() using
  // value-initialization.
  if (!lcm::AreLcmMessagesEqual(message, lcmt_schunk_wsg_command{})) {
    force_limit = message.force;
  }

  output->SetAtIndex(0, force_limit);
}

SchunkWsgCommandSender::SchunkWsgCommandSender(double default_force_limit)
    : position_input_port_(
          this->DeclareVectorInputPort("position", 1).get_index()),
      force_limit_input_port_(
          this->DeclareVectorInputPort("force_limit", 1).get_index()),
      default_force_limit_(default_force_limit) {
  this->DeclareAbstractOutputPort("lcmt_schunk_wsg_command",
                                  &SchunkWsgCommandSender::CalcCommandOutput);
}

void SchunkWsgCommandSender::CalcCommandOutput(
    const drake::systems::Context<double>& context,
    drake::lcmt_schunk_wsg_command* output) const {
  lcmt_schunk_wsg_command& command = *output;

  command.utime = context.get_time() * 1e6;
  command.target_position_mm = get_position_input_port().Eval(context)[0] * 1e3;
  if (get_force_limit_input_port().HasValue(context)) {
    command.force = get_force_limit_input_port().Eval(context)[0];
  } else {
    command.force = default_force_limit_;
  }
}

SchunkWsgStatusReceiver::SchunkWsgStatusReceiver()
    : state_output_port_(
          this->DeclareVectorOutputPort("state", 2,
                                        &SchunkWsgStatusReceiver::CopyStateOut)
              .get_index()),
      force_output_port_(
          this->DeclareVectorOutputPort("force", 1,
                                        &SchunkWsgStatusReceiver::CopyForceOut)
              .get_index()) {
  this->DeclareAbstractInputPort("lcmt_schunk_wsg_status",
                                 Value<lcmt_schunk_wsg_status>());
}

void SchunkWsgStatusReceiver::CopyStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_schunk_wsg_status>(context);
  output->SetAtIndex(0, status.actual_position_mm / 1e3);
  output->SetAtIndex(1, status.actual_speed_mm_per_s / 1e3);
}

void SchunkWsgStatusReceiver::CopyForceOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& status =
      get_status_input_port().Eval<lcmt_schunk_wsg_status>(context);
  output->SetAtIndex(0, status.actual_force);
}

SchunkWsgStatusSender::SchunkWsgStatusSender() {
  state_input_port_ = this->DeclareInputPort(
      systems::kUseDefaultName, systems::kVectorValued, 2).get_index();
  force_input_port_ = this->DeclareInputPort(
      systems::kUseDefaultName, systems::kVectorValued, 1).get_index();
  this->DeclareAbstractOutputPort(
      systems::kUseDefaultName, &SchunkWsgStatusSender::OutputStatus);
}

void SchunkWsgStatusSender::OutputStatus(const Context<double>& context,
                                         lcmt_schunk_wsg_status* output) const {
  lcmt_schunk_wsg_status& status = *output;
  status.utime = context.get_time() * 1e6;

  const auto& state = get_state_input_port().Eval(context);
  // The position and speed reported in this message are between the
  // two fingers rather than the position/speed of a single finger
  // (so effectively doubled).
  status.actual_position_mm = state[0] * 1e3;
  status.actual_speed_mm_per_s = state[1] * 1e3;

  if (get_force_input_port().HasValue(context)) {
    // In drake-schunk-driver, the driver attempts to apply a sign to the
    // force value based on the last reported direction of gripper movement
    // (the gripper always reports positive values).  This does not work very
    // well, and as a result the sign is almost always negative (when
    // non-zero) regardless of which direction the gripper was moving in when
    // motion was blocked.  As it's not a reliable source of information in
    // the driver (and should be removed there at some point), we don't try to
    // replicate it here and instead report positive forces.
    using std::abs;
    status.actual_force = abs(get_force_input_port().Eval(context)[0]);
  } else {
    status.actual_force = 0;
  }
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
