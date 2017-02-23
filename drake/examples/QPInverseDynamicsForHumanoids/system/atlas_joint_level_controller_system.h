#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A class that extends JointLevelControllerSystem and output an additional
 * bot_core::atlas_command_t in addition to the BasicVector of torques.
 */
class AtlasJointLevelControllerSystem : public JointLevelControllerSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AtlasJointLevelControllerSystem)

  /**
   * Constructor for AtlasJointLevelControllerSystem.
   * @param robot Reference to a RigidBodyTree. An internal alias is saved,
   * so the lifespan of @p robot must be longer than this object.
   */
  explicit AtlasJointLevelControllerSystem(const RigidBodyTree<double>& robot);

  /**
   * Calls JointLevelControllerSystem::DoCalcOutput() first, then constructs
   * a bot_core::atlas_command_t.
   */
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  /**
   * @return Port for the output: bot_core::atlas_command_t message
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_atlas_command() const {
    return get_output_port(output_port_index_atlas_cmd_);
  }

 private:
  int output_port_index_atlas_cmd_{0};

  // Joint level gains, these are in the actuator order.
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
