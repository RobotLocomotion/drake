#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/dev/humanoid_manipulation_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

HumanoidPlanEvalSystem::HumanoidPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt) {
  set_name("HumanoidPlanEval");

  input_port_index_manip_plan_msg_ = DeclareAbstractInputPort().get_index();

  auto plan_as_value = systems::AbstractValue::Make<GenericPlan<double>>(
      HumanoidManipulationPlan<double>());
  abs_state_index_plan_ = DeclareAbstractState(std::move(plan_as_value));
}

void HumanoidPlanEvalSystem::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  GenericPlan<double>& plan = get_mutable_abstract_value<GenericPlan<double>>(
      state, abs_state_index_plan_);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_humanoid_status().get_index());

  // Gets the plan message fron input.
  const systems::AbstractValue* msg_as_value =
      EvalAbstractInput(context, input_port_index_manip_plan_msg_);
  DRAKE_DEMAND(msg_as_value != nullptr);

  // Handles the plan.
  plan.HandlePlan(*robot_status, get_paramset(), get_alias_groups(),
                  *msg_as_value);

  // Runs the controller.
  plan.ModifyPlan(*robot_status, get_paramset(), get_alias_groups());

  // Updates the QpInput in AbstractState.
  QpInput& qp_input = get_mutable_qp_input(state);
  plan.UpdateQpInput(*robot_status, get_paramset(), get_alias_groups(),
                     &qp_input);
}

void HumanoidPlanEvalSystem::Initialize(const HumanoidStatus& current_status,
                                        systems::State<double>* state) const {
  // Initializes the plan.
  GenericPlan<double>& plan = get_mutable_abstract_value<GenericPlan<double>>(
      state, abs_state_index_plan_);
  plan.Initialize(current_status, get_paramset(), get_alias_groups());

  // Uses the plan to initialize the first QpInput. This is important because
  // on the first tick, CalcOutput will be called before unrestricted update,
  // where outputs are "really" computed. So we need to compute it here first.
  QpInput& qp_input = get_mutable_qp_input(state);
  plan.UpdateQpInput(current_status, get_paramset(), get_alias_groups(),
                     &qp_input);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
