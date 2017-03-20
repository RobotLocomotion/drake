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
    : robot_(robot), control_dt_(dt), alias_groups_(robot) {
  DRAKE_DEMAND(control_dt_ > 0);

  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

  // Declares discrete time update.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

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

  int abs_state_index_qp_input = context.get_abstract_state()->size() - 1;

  qp_input = context.get_abstract_state<QpInput>(abs_state_index_qp_input);

  // Does extended CalcOutput.
  DoExtendedCalcOutput(context, output);
}

std::unique_ptr<systems::AbstractValues>
PlanEvalBaseSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals =
      ExtendedAllocateAbstractState();

  abstract_vals.push_back(systems::AbstractValue::Make<QpInput>(
      QpInput()));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue>
PlanEvalBaseSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  if (descriptor.get_index() == output_port_index_qp_input_) {
    return systems::AbstractValue::Make<QpInput>(QpInput(GetDofNames(robot_)));
  } else {
    return ExtendedAllocateOutputAbstract(descriptor);
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
