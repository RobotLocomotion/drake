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

SchunkWsgCommandReceiver::SchunkWsgCommandReceiver(
    double initial_position, double initial_force)
    : initial_position_(initial_position),
      initial_force_(initial_force),
      commanded_position_output_port_(
          this->DeclareVectorOutputPort(
              "commanded_position", BasicVector<double>(1),
              &SchunkWsgCommandReceiver::CalcCommandedPositionOutput)
          .get_index()),
      force_limit_output_port_(
          this->DeclareVectorOutputPort(
              "force_limit", BasicVector<double>(1),
              &SchunkWsgCommandReceiver::CalcForceLimitOutput)
          .get_index()) {
  this->DeclareAbstractInputPort("wsg_command",
                                 systems::Value<lcmt_schunk_wsg_command>());
}

void SchunkWsgCommandReceiver::CalcCommandedPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_schunk_wsg_command>();

  double target_position = initial_position_;
  if (command.utime != 0) {
    // The target_position_mm field represents the distance between the two
    // fingers in millimeters. This class generates a desired position output
    // for the distance for a single finger to the center.
    target_position = 0.5 * command.target_position_mm / 1e3;
    if (std::isnan(target_position)) {
      target_position = 0;
    }
  }

  output->SetAtIndex(0, target_position);
}


void SchunkWsgCommandReceiver::CalcForceLimitOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_schunk_wsg_command>();

  double force_limit = initial_force_;
  if (command.utime != 0) {
    force_limit = command.force;
  }

  output->SetAtIndex(0, force_limit);
}

SchunkWsgStatusSender::SchunkWsgStatusSender(int input_state_size,
                                             int input_torque_size,
                                             int position_index,
                                             int velocity_index)
    : position_index_(position_index), velocity_index_(velocity_index) {
  input_port_wsg_state_ =
      this->DeclareInputPort(systems::kVectorValued, input_state_size)
          .get_index();
  input_port_measured_torque_ =
      this->DeclareInputPort(systems::kVectorValued, input_torque_size)
          .get_index();
  this->DeclareAbstractOutputPort(&SchunkWsgStatusSender::OutputStatus);
}

void SchunkWsgStatusSender::OutputStatus(const Context<double>& context,
                                         lcmt_schunk_wsg_status* output) const {
  lcmt_schunk_wsg_status& status = *output;

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, input_port_wsg_state_);
  const systems::BasicVector<double>* force =
      this->EvalVectorInput(context, input_port_measured_torque_);
  // The position and speed reported in this message are between the
  // two fingers rather than the position/speed of a single finger
  // (so effectively doubled).
  status.actual_position_mm = -2 * state->GetAtIndex(position_index_) * 1e3;
  status.actual_speed_mm_per_s = -2 * state->GetAtIndex(velocity_index_) * 1e3;

  if (force) {
    status.actual_force = -force->GetAtIndex(0);
  } else {
    status.actual_force = 0;
  }
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
