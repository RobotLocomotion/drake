#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class AtlasJointLevelControllerSystem : public JointLevelControllerSystem {
 public:
  explicit AtlasJointLevelControllerSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  virtual std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  /**
   * @return Port for the output: bot_core::atlas_command_t message
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_atlas_command() const {
    return get_output_port(output_port_index_atlas_cmd_);
  }

 private:
  int output_port_index_atlas_cmd_;

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
