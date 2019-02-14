#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using systems::BasicVector;
using systems::Context;

void SchunkWsgCommandTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  auto command = dynamic_cast<SchunkWsgCommand<double>*>(vector_base);
  DRAKE_THROW_UNLESS(command);

  lcmt_schunk_wsg_command msg{};
  const int length = msg.decode(lcm_message_bytes, 0, lcm_message_length);
  DRAKE_THROW_UNLESS(length == lcm_message_length);

  command->set_utime(msg.utime);
  command->set_target_position_mm(msg.target_position_mm);
  command->set_force(msg.force);
}

void SchunkWsgCommandTranslator::Serialize(double,
                                           const systems::VectorBase<double>&,
                                           std::vector<uint8_t>*) const {
  throw std::runtime_error("Not implemented");
}

std::unique_ptr<systems::BasicVector<double>>
SchunkWsgCommandTranslator::AllocateOutputVector() const {
  return std::make_unique<SchunkWsgCommand<double>>();
}

SchunkWsgCommandReceiver::SchunkWsgCommandReceiver(double initial_position,
                                                   double initial_force)
    : initial_position_(initial_position), initial_force_(initial_force) {
  this->DeclareVectorOutputPort("position", BasicVector<double>(1),
                                &SchunkWsgCommandReceiver::CalcPositionOutput);
  this->DeclareVectorOutputPort(
      "force_limit", BasicVector<double>(1),
      &SchunkWsgCommandReceiver::CalcForceLimitOutput);

  // TODO(jwnimmer-tri) Remove this input port after 2019-03-01.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  SchunkWsgCommand<double> uninitialized_vector;
#pragma GCC diagnostic pop
  this->DeclareVectorInputPort("command_vector", uninitialized_vector);

  lcmt_schunk_wsg_command uninitialized_message{};
  this->DeclareAbstractInputPort(
      "command_message",
      Value<lcmt_schunk_wsg_command>(uninitialized_message));
}

void SchunkWsgCommandReceiver::EvalInput(
    const Context<double>& context, SchunkWsgCommand<double>* result) const {
  // Try the vector input port first.
  if (this->get_input_port(0).HasValue(context)) {
    *result = this->get_input_port(0).Eval<SchunkWsgCommand<double>>(context);
  } else {
    const auto& message =
        this->get_input_port(1).Eval<lcmt_schunk_wsg_command>(context);
    std::vector<uint8_t> bytes(message.getEncodedSize());
    message.encode(bytes.data(), 0, bytes.size());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const SchunkWsgCommandTranslator translator;
#pragma GCC diagnostic pop
    translator.Deserialize(bytes.data(), bytes.size(), result);
  }
}

void SchunkWsgCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const SchunkWsgCommand<double> default_command;
  SchunkWsgCommand<double> wsg_command;
#pragma GCC diagnostic pop
  EvalInput(context, &wsg_command);

  double target_position = initial_position_;
  if (wsg_command.utime() != default_command.utime()) {
    target_position = wsg_command.target_position_mm() / 1e3;
    if (std::isnan(target_position)) {
      target_position = 0;
    }
  }

  output->SetAtIndex(0, target_position);
}

void SchunkWsgCommandReceiver::CalcForceLimitOutput(
    const Context<double>& context, BasicVector<double>* output) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const SchunkWsgCommand<double> default_command;
  SchunkWsgCommand<double> wsg_command;
#pragma GCC diagnostic pop
  EvalInput(context, &wsg_command);

  double force_limit = initial_force_;
  if (wsg_command.utime() != default_command.utime()) {
    force_limit = wsg_command.force();
  }

  output->SetAtIndex(0, force_limit);
}

SchunkWsgCommandSender::SchunkWsgCommandSender()
    : position_input_port_(this->DeclareVectorInputPort(
                                   "position", systems::BasicVector<double>(1))
                               .get_index()),
      force_limit_input_port_(
          this->DeclareVectorInputPort("force_limit",
                                       systems::BasicVector<double>(1))
              .get_index()) {
  this->DeclareAbstractOutputPort("lcmt_schunk_wsg_command",
                                  &SchunkWsgCommandSender::CalcCommandOutput);
}

void SchunkWsgCommandSender::CalcCommandOutput(
    const drake::systems::Context<double>& context,
    drake::lcmt_schunk_wsg_command* output) const {
  lcmt_schunk_wsg_command& command = *output;

  command.utime = context.get_time() * 1e6;
  command.target_position_mm = get_position_input_port().Eval(context)[0] * 1e3;
  command.force = get_force_limit_input_port().Eval(context)[0];
}

SchunkWsgStatusReceiver::SchunkWsgStatusReceiver()
    : state_output_port_(this->DeclareVectorOutputPort(
                                 "state", systems::BasicVector<double>(2),
                                 &SchunkWsgStatusReceiver::CopyStateOut)
                             .get_index()),
      force_output_port_(this->DeclareVectorOutputPort(
                                 "force", systems::BasicVector<double>(1),
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

SchunkWsgStatusSender::SchunkWsgStatusSender(int input_state_size,
                                             int input_torque_size,
                                             int position_index,
                                             int velocity_index)
    : position_index_(position_index), velocity_index_(velocity_index) {
  // Note: Using DRAKE_DEPRECATED on the constructor was not throwing a
  // compile time warning.
  drake::log()->warn(
      "This constructor is deprecated.  Use the default constructor "
      "and just wire in the two-dimensional state input.  Note that "
      "the *sign* of the expected input has also changed -- it is the positive "
      "distance between fingers.  Use MakeMultibodyStateToWsgStateSystem() and "
      "MakeMultibodyForceToWsgForceSystem() to create the transforms.");

  input_port_wsg_state_ =
      this->DeclareInputPort(systems::kVectorValued, input_state_size)
          .get_index();
  // Note: Keeping this behavior for backwards compatibility (but it is
  // deprecated).  The existing code had a bug where only the first element
  // of the force input was every used, even if input_torque_size > 1.
  force_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, input_torque_size)
          .get_index();
  this->DeclareAbstractOutputPort(&SchunkWsgStatusSender::OutputStatus);
}

SchunkWsgStatusSender::SchunkWsgStatusSender() {
  state_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 2).get_index();
  force_input_port_ =
      this->DeclareInputPort(systems::kVectorValued, 1).get_index();
  this->DeclareAbstractOutputPort(&SchunkWsgStatusSender::OutputStatus);
}

void SchunkWsgStatusSender::OutputStatus(const Context<double>& context,
                                         lcmt_schunk_wsg_status* output) const {
  lcmt_schunk_wsg_status& status = *output;
  status.utime = context.get_time() * 1e6;

  // Maintain the deprecated mode for now.
  if (input_port_wsg_state_.is_valid()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto& state = get_input_port_wsg_state().Eval(context);
#pragma GCC diagnostic pop
    status.actual_position_mm = -2 * state[position_index_] * 1e3;
    status.actual_speed_mm_per_s = -2 * state[velocity_index_] * 1e3;
  } else {
    const auto& state = get_state_input_port().Eval(context);
    // The position and speed reported in this message are between the
    // two fingers rather than the position/speed of a single finger
    // (so effectively doubled).
    status.actual_position_mm = state[0] * 1e3;
    status.actual_speed_mm_per_s = state[1] * 1e3;
  }

  if (get_force_input_port().HasValue(context)) {
    // In drake-schunk-driver, the driver attempts to apply a sign to the
    // force value based on the last reported direction of gripper movement
    // (the gripper always reports positive values).  This does not work very
    // well, and as a result the sign is almost always negative (when
    // non-zero) regardless of which direction the gripper was moving in when
    // motion was blocked.  As it's not a reliable source of information in
    // the driver (and should be removed there at some point), we don't try to
    // replicate it here and instead report positive forces.

    // TODO(sammy-tri) once the deprecated constructor/ports are removed, this
    // can just use Eval(context)[0] here.
    status.actual_force = get_force_input_port().Eval(context).cwiseAbs().sum();
  } else {
    status.actual_force = 0;
  }
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
