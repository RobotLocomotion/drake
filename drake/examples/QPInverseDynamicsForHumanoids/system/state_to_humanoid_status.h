#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A translator class from state vector to HumanoidStatus.
 */
class StateToHumanoidStatus : public systems::LeafSystem<double> {
 public:
  StateToHumanoidStatus(const RigidBodyTree<double>& robot,
                        const std::string& path);

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
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_humanoid_status() const {
    return get_output_port(output_port_index_humanoid_status_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  const std::string alias_group_path_;

  int input_port_index_state_{0};
  int output_port_index_humanoid_status_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
