#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/gripper_action.h"

#include <vector>

#include "external/robotlocomotion_lcmtypes/lcmtypes/robotlocomotion/robot_plan_t.hpp"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitive_base.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"

namespace drake {

namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
const double kMinWidthInMm{0};
const double kMaxWidthInMm{110};

struct GripperAction::InternalState {
  InternalState() {
    plan = lcmt_schunk_wsg_command();
    plan.force = 0.0;
    plan.utime = 0.0;
    plan.target_position_mm = kMaxWidthInMm; /* Opening */
  }
  ~InternalState() {}

  bool is_valid{false};
  lcmt_schunk_wsg_command plan;
  GripperActionInput previous_input{GripperActionInput::CLOSE};
  lcmt_schunk_wsg_command previous_valid_plan;
  double start_time{0.0};
  double plan_duration{0.0};
};

GripperAction::GripperAction(double desired_update_interval)
    : ActionPrimitive(desired_update_interval,
                      1 /* action_primitive_state_index */ ),
      internal_state_index_(0),
      plan_output_port(this->DeclareAbstractOutputPort().get_index()),
      input_port_primitive_input_(this->DeclareAbstractInputPort().get_index())
      {}

std::vector<std::unique_ptr<systems::AbstractValue>>
GripperAction::AllocateExtendedAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> return_value;
  return_value.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(InternalState())));

  return return_value;
}
std::unique_ptr<systems::AbstractValue>
GripperAction::ExtendedAllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  std::unique_ptr<systems::AbstractValue> return_value;
  if (descriptor.get_index() == plan_output_port) {
    lcmt_schunk_wsg_command default_command;
    default_command.target_position_mm = kMaxWidthInMm;
    default_command.force = 0.0;
    return_value =
        systems::AbstractValue::Make<lcmt_schunk_wsg_command>(default_command);
  }
  return return_value;
}

void GripperAction::SetExtendedDefaultState(
    const systems::Context<double> &context,
    systems::State<double> *state) const {
  InternalState& wsg_action_state =
      state->get_mutable_abstract_state<InternalState>(
          internal_state_index_ /* index of iiwastate */);
  wsg_action_state.is_valid = false;
  wsg_action_state.plan = lcmt_schunk_wsg_command(); // empty plan
  wsg_action_state.plan.target_position_mm = kMaxWidthInMm;
  wsg_action_state.previous_valid_plan = wsg_action_state.plan;
  wsg_action_state.start_time = 0;
  wsg_action_state.plan_duration = 0;
  wsg_action_state.previous_input = GripperActionInput::UNDEFINED;
}
void GripperAction::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the current input, check current state, overwrite state if state time
  // exceeded.

  InternalState& wsg_action_state =
      state->get_mutable_abstract_state<InternalState>(
          internal_state_index_);
  ActionPrimitiveState& primitive_state =
      state->get_mutable_abstract_state<ActionPrimitiveState>(
          action_primitive_state_index_);

  double time = context.get_time();
  const GripperActionInput& input_plan =
      this->EvalAbstractInput(context, input_port_primitive_input_)
          ->GetValue<GripperActionInput>();

  // check current state
  switch (primitive_state) {
    case ActionPrimitiveState::RUNNING:
      if (time - wsg_action_state.start_time > wsg_action_state.plan_duration) {
        wsg_action_state.is_valid = false;
        primitive_state = ActionPrimitiveState::WAITING;
        wsg_action_state.previous_valid_plan = wsg_action_state.plan;
      }
      break;
    case ActionPrimitiveState::WAITING:

      if (wsg_action_state.previous_input != input_plan) {
        // Then change state and start acting on the new input.
        primitive_state = ActionPrimitiveState::RUNNING;
        lcmt_schunk_wsg_command new_plan;
        new_plan.utime = static_cast<int64_t>(time * 1e6);
        if (input_plan == GripperActionInput::CLOSE) {
          new_plan.target_position_mm = kMinWidthInMm;
        } else {
          new_plan.target_position_mm = kMaxWidthInMm;
        }
        wsg_action_state.plan = new_plan;

        wsg_action_state.start_time = time;
        wsg_action_state.plan_duration = 0.5; /* wsg action duration */
        wsg_action_state.is_valid = true;
        wsg_action_state.previous_input = input_plan;
      }
      break;
    case ActionPrimitiveState::ABORTED:
      // Do nothing for now (reserved for the future).
      break;
  }
}

void GripperAction::DoExtendedCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  lcmt_schunk_wsg_command& wsg_plan_output =
      output->GetMutableData(plan_output_port)
          ->GetMutableValue<lcmt_schunk_wsg_command>();

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);

  if (internal_state.is_valid) {
    wsg_plan_output = internal_state.plan;
  }
  else {
    wsg_plan_output = internal_state.previous_valid_plan;
  }
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake