#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A class that translates QpOutput to vector of torque commands in the
 * actuator order.
 */
class JointLevelControllerSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointLevelControllerSystem)

  /**
   * Constructor for JointLevelControllerSystem.
   * @param robot Reference to a RigidBodyTree. An internal alias is saved,
   * so the lifespan of @p robot must be longer than this object.
   */
  explicit JointLevelControllerSystem(const RigidBodyTree<double>& robot);

  /**
   * Extracts the torques from a QpOutput to a BasicVector.
   * More specifically, the output tau_act = B^T * qp_output.dof_torques, where
   * B is a selection matrix that maps the RigidBodyTree's actuators to its
   * generalized acceleration.
   */
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  /**
   * @return Port for the input: QpOutput
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_qp_output()
      const {
    return get_input_port(input_port_index_qp_output_);
  }

  /**
   * @return Port for the output: torques as a BasicVector
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_torque()
      const {
    return get_output_port(output_port_index_torque_);
  }

 protected:
  const RigidBodyTree<double>& robot_;

  int input_port_index_qp_output_{0};
  int output_port_index_torque_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
