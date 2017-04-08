#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/iiwa_move.h"

#include <vector>
#include "robotlocomotion/robot_plan_t.hpp"

#include "external/robotlocomotion_lcmtypes/lcmtypes/robotlocomotion/robot_plan_t.hpp"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitive_base.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

using robotlocomotion::robot_plan_t;

namespace drake {
using systems::LeafSystem;

namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

struct IiwaMove::InternalState {
  InternalState() {}
  ~InternalState() {}

  IiwaActionInput last_input;
  bool is_valid{false};
  robot_plan_t plan;
  double start_time{0.0};
  double plan_duration{0.0};
};

IiwaMove::IiwaMove(const RigidBodyTree<double>& iiwa,
                   double desired_update_interval)
    : ActionPrimitive(desired_update_interval),
      input_port_primitive_(this->DeclareAbstractInputPort().get_index()),
      output_port_plan_(this->DeclareAbstractOutputPort().get_index()),
      iiwa_tree_(iiwa) {}

std::vector<std::unique_ptr<systems::AbstractValue>>
IiwaMove::AllocateExtendedAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> return_value;
  return_value.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(InternalState())));
  return return_value;
}

std::unique_ptr<systems::AbstractValue>
IiwaMove::ExtendedAllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  std::unique_ptr<systems::AbstractValue> return_value;
  if (descriptor.get_index() == output_port_plan_) {
    return_value = systems::AbstractValue::Make<robot_plan_t>(robot_plan_t());
  }
  return (return_value);
}

void IiwaMove::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Get the current input, check current state, overwrite state if state
  // time_now exceeded.

  InternalState& iiwa_action_state =
      state->get_mutable_abstract_state<InternalState>(
          0 /* index of iiwastate */);
  ActionPrimitiveState& primitive_state =
      state->get_mutable_abstract_state<ActionPrimitiveState>(
          1 /* index of action primitive state */);

  robot_plan_t current_plan = iiwa_action_state.plan;

  const IiwaActionInput& input_plan =
      this->EvalAbstractInput(context, input_port_primitive_)
          ->GetValue<IiwaActionInput>();

  DRAKE_DEMAND(input_plan.q.size() == input_plan.time.size());

  double time_now = context.get_time();

  // check current state
  switch (primitive_state) {
    case ActionPrimitiveState::RUNNING:
      // check time_now and change state if needed.

      if (time_now - iiwa_action_state.start_time >=
          iiwa_action_state.plan_duration) {
        primitive_state = ActionPrimitiveState::WAITING;
        iiwa_action_state.is_valid = false;
      }
      break;
    case ActionPrimitiveState::WAITING:
      // check input and change state if needed.
      // A new plan is considered to be input if it has a size greater than 0
      if (input_plan.is_valid && input_plan.time.size() > 0 &&
          (input_plan.q != iiwa_action_state.last_input.q ||
           input_plan.time != iiwa_action_state.last_input.time)) {
        const unsigned long plan_num_points = input_plan.time.size();

        std::vector<int> info(plan_num_points, 1);
        MatrixX<double> q_mat(input_plan.q.front().size(), plan_num_points);
        for (size_t i = 0; i < plan_num_points; ++i)
          q_mat.col(i) = input_plan.q[i];
        iiwa_action_state.plan =
            EncodeKeyFrames(iiwa_tree_, input_plan.time, info, q_mat);
        // Revist next line for "long" term plans.
        iiwa_action_state.plan_duration =
            input_plan.time.back() - input_plan.time.front();

        primitive_state = ActionPrimitiveState::RUNNING;
        iiwa_action_state.is_valid = true;
        iiwa_action_state.start_time = time_now;
        iiwa_action_state.last_input = input_plan;
      }
      break;
    case ActionPrimitiveState::ABORTED:
      drake::log()->info("IiwaPrimitive State ABORTED");
      break;
  }
}

void IiwaMove::DoExtendedCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Do the output copy.
  robot_plan_t& robot_plan_output = output->GetMutableData(output_port_plan_)
                                        ->GetMutableValue<robot_plan_t>();

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);

  if (internal_state.is_valid) {
    robot_plan_output = internal_state.plan;
  }
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake