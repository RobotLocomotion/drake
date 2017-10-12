#include "drake/systems/controllers/plan_eval/plan_eval_base_system.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"
#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

using systems::controllers::qp_inverse_dynamics::GetDofNames;
using systems::controllers::qp_inverse_dynamics::QpInput;
using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

PlanEvalBaseSystem::PlanEvalBaseSystem(
    const RigidBodyTree<double>* robot,
    std::unique_ptr<RigidBodyTreeAliasGroups<double>>* alias_groups,
    std::unique_ptr<qp_inverse_dynamics::ParamSet>* paramset,
    double dt)
    : robot_(*robot), control_dt_(dt) {
  DRAKE_DEMAND(control_dt_ > 0);

  alias_groups_ = std::move(*alias_groups);
  paramset_ = std::move(*paramset);

  input_port_index_kinematic_state_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ =
      DeclareAbstractOutputPort(QpInput(GetDofNames(robot_)),
                                &PlanEvalBaseSystem::CopyOutQpInput)
          .get_index();

  // Declares discrete time update.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  set_name("PlanEval");

  abs_state_index_qp_input_ =
      DeclareAbstractState(systems::AbstractValue::Make<QpInput>(QpInput()));
}

void PlanEvalBaseSystem::CopyOutQpInput(const systems::Context<double>& context,
                                        QpInput* qp_input) const {
  // Copies QpInput from AbstractState.
  *qp_input = context.get_abstract_state<QpInput>(abs_state_index_qp_input_);
}

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
