#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/iiwa_move.h"

#include <vector>
#include "external/robotlocomotion_lcmtypes/lcmtypes/robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitive_base.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/rigid_body_tree.h"

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

struct IiwaMove::InternalState {
  InternalState() {}
  ~InternalState() {}

  IiwaActionInput last_input;
  robot_plan_t current_plan;
  double start_time{0.0};
  double plan_duration{0.0};
};

IiwaMove::IiwaMove(const RigidBodyTree<double>& iiwa,
                   double desired_update_interval)
    : ActionPrimitive(desired_update_interval,
                      1 /* action_primitive_state_index */),
      internal_state_index_(0),
      input_port_primitive_input_(this->DeclareAbstractInputPort().get_index()),
      output_port_plan_(
          this->DeclareAbstractOutputPort(systems::Value<robot_plan_t>())
              .get_index()),
      iiwa_tree_(iiwa) {}

std::vector<std::unique_ptr<systems::AbstractValue>>
IiwaMove::AllocateExtendedAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> return_value;
  return_value.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(InternalState())));
  return return_value;
}

void IiwaMove::SetExtendedDefaultState(const systems::Context<double>& context,
                                       systems::State<double>* state) const {
  InternalState& iiwa_action_state =
      state->get_mutable_abstract_state<InternalState>(
          internal_state_index_ /* index of iiwastate */);
  iiwa_action_state.last_input.is_valid = false;
  iiwa_action_state.current_plan = robot_plan_t();
  iiwa_action_state.last_input.time.clear();
  iiwa_action_state.last_input.q.clear();
  iiwa_action_state.start_time = 0;
  iiwa_action_state.plan_duration = 0;
}

void IiwaMove::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Get the current input, check current state, overwrite state if state
  // time_now exceeded.

  InternalState& iiwa_action_state =
      state->get_mutable_abstract_state<InternalState>(
          internal_state_index_);
  ActionPrimitiveState& primitive_state =
      state->get_mutable_abstract_state<ActionPrimitiveState>(
          get_action_primitive_state_index());

  const IiwaActionInput& input_plan =
      this->EvalAbstractInput(context, input_port_primitive_input_)
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
      }
      break;
    case ActionPrimitiveState::WAITING:
      // check input and change state if needed.
      // A new plan is considered to be input if it has a size greater than 0
      if (input_plan.is_valid && input_plan.time.size() > 0 &&
          (input_plan.q != iiwa_action_state.last_input.q ||
           input_plan.time != iiwa_action_state.last_input.time)) {
        const uint64_t plan_num_points = input_plan.time.size();

        std::vector<int> info(plan_num_points, 1);
        MatrixX<double> q_mat(input_plan.q.front().size(), plan_num_points);
        for (size_t i = 0; i < plan_num_points; ++i)
          q_mat.col(i) = input_plan.q[i];
        iiwa_action_state.current_plan =
            EncodeKeyFrames(iiwa_tree_, input_plan.time, info, q_mat);
        // TODO(naveenoid) : Revisit next line for "long" term plans.
        iiwa_action_state.plan_duration =
            input_plan.time.back() - input_plan.time.front();

        primitive_state = ActionPrimitiveState::RUNNING;
        iiwa_action_state.start_time = time_now;
        iiwa_action_state.last_input = input_plan;
      } else {
        iiwa_action_state.current_plan = robot_plan_t();
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

  robot_plan_output = internal_state.current_plan;
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
