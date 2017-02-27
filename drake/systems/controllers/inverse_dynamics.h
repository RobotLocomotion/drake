#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
class InverseDynamics : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamics)

  /// Constructs a gravity compensator for the given @tree. The provided @p tree
  /// must be fully actuated, i.e., the number of actuators must equal the
  /// number of positions in the RigidBodyTree. Otherwise, the process will
  /// abort.
  InverseDynamics(const RigidBodyTree<T>& tree, bool pure_gravity_compensation);

  /**
   * Returns the input port for the estimated state.
   */
  const InputPortDescriptor<T>& get_input_port_estimated_state() const {
    return this->get_input_port(input_port_index_state_);
  }

  /**
   * Returns the input port for the estimated state.
   */
  const InputPortDescriptor<T>& get_input_port_desired_acceleration() const {
    DRAKE_DEMAND(!only_gravity_);
    return this->get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for the estimated state.
   */
  const OutputPortDescriptor<T>& get_output_port_torque() const {
    return this->get_output_port(output_port_index_torque_);
  }

 private:
  // Sets the output port value to the generalized gravity forces
  // corresponding to a joint configuration as specified in the input.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  const RigidBodyTree<T>& tree_;
  const bool only_gravity_{false};

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_torque_{0};

  const int q_dim_{0};
  const int v_dim_{0};
  const int act_dim_{0};
};

}  // namespace systems
}  // namespace drake
