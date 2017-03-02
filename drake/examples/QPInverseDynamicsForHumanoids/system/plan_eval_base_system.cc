#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_base_system.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

PlanEvalBaseSystem::PlanEvalBaseSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : robot_(robot),
      kControlDt(dt),
      alias_groups_(robot),
      kAbsStateIdxQpInput(0),
      kAbsStateIdxPlan(1) {
  DRAKE_DEMAND(kControlDt > 0);

  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

  // Declares discrete time update.
  DeclarePeriodicUnrestrictedUpdate(kControlDt, 0);

  // Loads configuration files.
  alias_groups_.LoadFromFile(alias_groups_file_name);
  paramset_.LoadFromFile(param_file_name, alias_groups_);

  set_name("PlanEval");
}

void PlanEvalBaseSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Copies QpInput from AbstractState.
  QpInput& qp_input = output->GetMutableData(output_port_index_qp_input_)
                          ->GetMutableValue<QpInput>();
  qp_input = context.get_abstract_state<QpInput>(kAbsStateIdxQpInput);
}

std::unique_ptr<systems::AbstractValue>
PlanEvalBaseSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_qp_input_);
  return systems::AbstractValue::Make<QpInput>(QpInput(GetDofNames(robot_)));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
