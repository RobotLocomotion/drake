#pragma once

#include <memory>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class JointLevelControllerSystem : public systems::LeafSystem<double> {
 public:
  explicit JointLevelControllerSystem(const RigidBodyTree<double>& robot);

  virtual void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_qp_output()
      const {
    return get_input_port(input_port_index_qp_output_);
  }

  /**
   * @return Port for the output: HumanoidStatus.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_torque()
      const {
    return get_output_port(output_port_index_torque_);
  }

 protected:
  const RigidBodyTree<double>& robot_;

  int input_port_index_qp_output_;
  int output_port_index_torque_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
