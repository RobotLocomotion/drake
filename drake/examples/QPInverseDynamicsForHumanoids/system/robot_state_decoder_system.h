#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::SystemPortDescriptor;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::Value;

/**
 * A translator from bot_core::robot_state_t to HumanoidStatus
 *
 * Input: lcm message bot_core::robot_state_t
 * Output: HumanoidStatus
 */
class RobotStateDecoderSystem : public systems::LeafSystem<double> {
 public:
  explicit RobotStateDecoderSystem(const RigidBodyTree<double>& robot)
      : robot_(robot) {
    input_port_index_lcm_msg_ = DeclareAbstractInputPort().get_index();
    output_port_index_humanoid_status_ =
        DeclareAbstractOutputPort().get_index();
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    // Input:
    const bot_core::robot_state_t* msg =
        EvalInputValue<bot_core::robot_state_t>(context,
                                                input_port_index_lcm_msg_);

    // Output:
    HumanoidStatus& hum_status =
        output->GetMutableData(output_port_index_humanoid_status_)
            ->GetMutableValue<HumanoidStatus>();

    VectorX<double> pos(hum_status.position().size());
    VectorX<double> vel(hum_status.velocity().size());
    VectorX<double> joint_torque(hum_status.joint_torque().size());
    Vector6<double> l_foot_wrench, r_foot_wrench;
    double time;

    DecodeRobotStateLcmMsg(*msg, hum_status.name_to_position_index(), &time,
                           &pos, &vel, &joint_torque, &l_foot_wrench,
                           &r_foot_wrench);

    hum_status.Update(time, pos, vel, joint_torque, l_foot_wrench,
                      r_foot_wrench);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    HumanoidStatus rs(robot_);
    output->add_port(
        std::unique_ptr<AbstractValue>(new Value<HumanoidStatus>(rs)));
    return std::move(output);
  }

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const SystemPortDescriptor<double>& get_input_port_robot_state_msg()
      const {
    return get_input_port(input_port_index_lcm_msg_);
  }

  /**
   * @return Port for the output: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_output_port_humanoid_status()
      const {
    return get_output_port(output_port_index_humanoid_status_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int input_port_index_lcm_msg_;
  int output_port_index_humanoid_status_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
