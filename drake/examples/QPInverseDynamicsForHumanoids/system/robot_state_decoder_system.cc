#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_state_decoder_system.h"

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <memory>
#include <string>
#include <utility>

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

RobotStateDecoderSystem::RobotStateDecoderSystem(
    const RigidBodyTree<double>& robot)
    : robot_(robot) {
  input_port_index_lcm_msg_ = DeclareAbstractInputPort().get_index();
  output_port_index_humanoid_status_ = DeclareAbstractOutputPort().get_index();
}

void RobotStateDecoderSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Input:
  const bot_core::robot_state_t* msg = EvalInputValue<bot_core::robot_state_t>(
      context, input_port_index_lcm_msg_);

  // Output:
  HumanoidStatus& hum_status =
      output->GetMutableData(output_port_index_humanoid_status_)
          ->GetMutableValue<HumanoidStatus>();

  VectorX<double> pos(hum_status.position().size());
  VectorX<double> vel(hum_status.velocity().size());
  VectorX<double> joint_torque(hum_status.joint_torque().size());
  Vector6<double> l_foot_wrench, r_foot_wrench;
  double time;

  DecodeRobotStateLcmMsg(*msg, hum_status.name_to_position_index(), &time, &pos,
                         &vel, &joint_torque, &l_foot_wrench, &r_foot_wrench);

  hum_status.Update(time, pos, vel, joint_torque, l_foot_wrench, r_foot_wrench);
}

std::unique_ptr<systems::AbstractValue>
RobotStateDecoderSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_humanoid_status_);

  std::string alias_groups_config =
      drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
      "config/valkyrie.alias_groups";
  // KinematicsProperty
  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(robot_);
  alias_groups.LoadFromFile(alias_groups_config);

  return systems::AbstractValue::Make<HumanoidStatus>(
      HumanoidStatus(robot_, alias_groups));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
