#pragma once

#include <memory>
#include <utility>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class RobotStatusWrapper : public systems::LeafSystem<double> {
 public:
  explicit RobotStatusWrapper(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_state()
      const {
    return get_input_port(input_port_index_state_);
  }

  /**
   * @return Port for the output: HumanoidStatus.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_humanoid_status()
      const {
    return get_output_port(output_port_index_humanoid_status_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int input_port_index_state_;
  int output_port_index_humanoid_status_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
