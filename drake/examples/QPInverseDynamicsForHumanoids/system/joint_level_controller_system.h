#pragma once

#include <memory>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A stub for a more complex interface for joint level controllers.
 * The idea is to separate all joint level control from the higher level
 * full state feedback controller, e.g. qp inverse dynamics controller.
 * This can also run at a higher rate than the full state feedback controller.
 *
 * Possible things to be implemented here:
 *  joint level set points, gains, integrators, filters,
 *  disturbance observers, etc.
 *
 * Input: HumanoidStatus
 * Input: QpOutput
 * Output: lcm message bot_core::atlas_command_t in channel "ROBOT_COMMAND"
 */
class JointLevelControllerSystem : public systems::LeafSystem<double> {
 public:
  explicit JointLevelControllerSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(in_port_idx_humanoid_status_);
  }

  /**
   * @return Port for the input: QpOutput.
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_qp_output()
      const {
    return get_input_port(in_port_idx_qp_output_);
  }

  /**
   * @return Port for the output: bot_core::atlas_command_t message
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_atlas_command() const {
    return get_output_port(out_port_index_atlas_cmd_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int in_port_idx_qp_output_;
  int in_port_idx_humanoid_status_;
  int out_port_index_atlas_cmd_;

  // Joint level gains, these are in actuator order.
  VectorX<double> k_q_p_;
  VectorX<double> k_q_i_;
  VectorX<double> k_qd_p_;
  VectorX<double> k_f_p_;
  VectorX<double> ff_qd_;
  VectorX<double> ff_qd_d_;
  VectorX<double> ff_f_d_;
  VectorX<double> ff_const_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
