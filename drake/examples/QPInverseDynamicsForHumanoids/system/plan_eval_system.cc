#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"

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

PlanEvalSystem::PlanEvalSystem(const RigidBodyTree<double>& robot,
                               const std::string& alias_groups_file_name,
                               const std::string& param_file_name)
    : robot_(robot),
      alias_groups_(robot),
      abstract_state_qp_input_index_(0),
      abstract_state_plan_index_(1) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();
  // Declare discrete time controller.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  set_name("plan_eval");

  // KinematicsProperty
  alias_groups_.LoadFromYAMLFile(YAML::LoadFile(alias_groups_file_name));

  // Controller config
  paramset_.LoadFromYAMLConfigFile(YAML::LoadFile(param_file_name),
                                   alias_groups_);

  abstract_state_qp_input_index_ = 0;
  abstract_state_plan_index_ = 1;
}

void PlanEvalSystem::DoCalcOutput(const systems::Context<double>& context,
                                  systems::SystemOutput<double>* output) const {
  // Output:
  QpInput& qp_input = output->GetMutableData(output_port_index_qp_input_)
                          ->GetMutableValue<QpInput>();

  // Gets QpInput from AbstractState.
  qp_input =
      context.get_abstract_state<QpInput>(abstract_state_qp_input_index_);
}

std::unique_ptr<systems::AbstractValue> PlanEvalSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_qp_input_);
  return systems::AbstractValue::Make<QpInput>(QpInput(GetDofNames(robot_)));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
