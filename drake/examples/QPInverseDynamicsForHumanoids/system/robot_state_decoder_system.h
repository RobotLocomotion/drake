#pragma once

#include <memory>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A translator from bot_core::robot_state_t to HumanoidStatus
 *
 * Input: lcm message bot_core::robot_state_t
 * Output: HumanoidStatus
 */
class RobotStateDecoderSystem : public systems::LeafSystem<double> {
 public:
  explicit RobotStateDecoderSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_robot_state_msg() const {
    return get_input_port(input_port_index_lcm_msg_);
  }

  /**
   * @return Port for the output: HumanoidStatus.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_humanoid_status() const {
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
