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
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : robot_(*robot), control_dt_(dt), alias_groups_(robot) {
  DRAKE_DEMAND(control_dt_ > 0);

  input_port_index_kinematic_state_ = DeclareAbstractInputPort(
      kUseDefaultName,
      Value<RobotKinematicState<double>>{robot}).get_index();
  output_port_index_qp_input_ =
      DeclareAbstractOutputPort(QpInput(GetDofNames(robot_)),
                                &PlanEvalBaseSystem::CopyOutQpInput)
          .get_index();

  // Declares discrete time update.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  // Loads configuration files.
  alias_groups_.LoadFromFile(alias_groups_file_name);
  paramset_.LoadFromFile(param_file_name, alias_groups_);

  set_name("PlanEval");

  abs_state_index_qp_input_ =
      DeclareAbstractState(AbstractValue::Make<QpInput>(QpInput()));
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
