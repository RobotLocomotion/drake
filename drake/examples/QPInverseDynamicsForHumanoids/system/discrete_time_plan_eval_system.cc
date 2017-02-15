#include "drake/examples/QPInverseDynamicsForHumanoids/system/discrete_time_plan_eval_system.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

DiscreteTimePlanEvalSystem::DiscreteTimePlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : robot_(robot),
      control_dt_(dt),
      alias_groups_(robot),
      abstract_state_index_qp_input_(0),
      abstract_state_index_plan_(1) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();
  // Declare discrete time update.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  set_name("plan_eval");

  // KinematicsProperty
  alias_groups_.LoadFromFile(alias_groups_file_name);

  paramset_.LoadFromFile(param_file_name, alias_groups_);

  abstract_state_index_qp_input_ = 0;
  abstract_state_index_plan_ = 1;
}

void DiscreteTimePlanEvalSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Copies QpInput from AbstractState.
  QpInput& qp_input = output->GetMutableData(output_port_index_qp_input_)
                          ->GetMutableValue<QpInput>();
  qp_input =
      context.get_abstract_state<QpInput>(abstract_state_index_qp_input_);
}

std::unique_ptr<systems::AbstractValue>
DiscreteTimePlanEvalSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_qp_input_);
  return systems::AbstractValue::Make<QpInput>(QpInput(GetDofNames(robot_)));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
