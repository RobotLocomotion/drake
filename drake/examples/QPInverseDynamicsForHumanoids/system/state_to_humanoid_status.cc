#include "drake/examples/QPInverseDynamicsForHumanoids/system/state_to_humanoid_status.h"

#include <string>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

StateToHumanoidStatus::StateToHumanoidStatus(const RigidBodyTree<double>& robot,
                                       const std::string& path)
    : robot_(robot), alias_group_path_(path) {
  input_port_index_state_ =
      DeclareInputPort(systems::kVectorValued,
                       robot.get_num_positions() + robot.get_num_velocities())
          .get_index();
  output_port_index_humanoid_status_ = DeclareAbstractOutputPort().get_index();
}

void StateToHumanoidStatus::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Input:
  VectorX<double> x = EvalEigenVectorInput(context, input_port_index_state_);

  // Output:
  HumanoidStatus& hum_status =
      output->GetMutableData(output_port_index_humanoid_status_)
          ->GetMutableValue<HumanoidStatus>();

  hum_status.Update(context.get_time(), x.head(robot_.get_num_positions()),
                    x.tail(robot_.get_num_velocities()),
                    VectorX<double>::Zero(robot_.actuators.size()),
                    Vector6<double>::Zero(), Vector6<double>::Zero());
}

std::unique_ptr<systems::SystemOutput<double>>
StateToHumanoidStatus::AllocateOutput(
    const systems::Context<double>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<double>> output(
      new systems::LeafSystemOutput<double>);

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(robot_);
  alias_groups.LoadFromFile(alias_group_path_);

  HumanoidStatus rs(robot_, alias_groups);
  output->add_port(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<HumanoidStatus>(rs)));
  return std::move(output);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
