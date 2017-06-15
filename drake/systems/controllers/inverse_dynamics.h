#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
 * Solves inverse dynamics with no consideration for external wrenches,
 * under actuation, joint torque limits or closed kinematic chains. The torque
 * is `H(q) * vd_d + c(q, v) + g(q)`, where `H` is the inertia matrix, `c` is
 * the coriolis and centrifugal, `g` is the gravity term, `q` is the generalized
 * position, `v` is the generalized velocity and `vd_d` is the desired
 * generalized acceleration. There is also a pure gravity compensation mode,
 * in which torque is computed as `g(q)`. This system always has an BasicVector
 * input port for the state `(q, v)` and an BasicVector output port for the
 * computed torque. There is an additional BasicVector input port for desired
 * acceleration when configured to be not in pure gravity compensation mode.
 *
 * InverseDynamicsController uses a PID controller to generate desired
 * acceleration and uses this class to compute torque. This class should be used
 * directly if desired acceleration is computed differently.
 */
template <typename T>
class InverseDynamics : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamics)

  /**
   * Computes inverse dynamics for @p tree.
   * @param tree Reference to the model. The life span of @p tree must be longer
   * than this instance.
   * @param pure_gravity_compensation If set to true, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   */
  InverseDynamics(const RigidBodyTree<T>& tree, bool pure_gravity_compensation);

  /**
   * Returns the input port for the estimated state.
   */
  const InputPortDescriptor<T>& get_input_port_estimated_state() const {
    return this->get_input_port(input_port_index_state_);
  }

  /**
   * Returns the input port for the desired acceleration.
   */
  const InputPortDescriptor<T>& get_input_port_desired_acceleration() const {
    DRAKE_DEMAND(!pure_gravity_compensation_);
    return this->get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for the actuation torques.
   */
  const OutputPort<T>& get_output_port_torque() const {
    return this->get_output_port(output_port_index_torque_);
  }

  bool is_pure_gravity_compenstation() const {
    return pure_gravity_compensation_;
  }

 private:
  // This is the calculator method for the output port.
  void CalcOutputTorque(const Context<T>& context,
                        BasicVector<T>* torque) const;

  const RigidBodyTree<T>& tree_;
  const bool pure_gravity_compensation_{false};

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_torque_{0};

  const int q_dim_{0};
  const int v_dim_{0};
  const int act_dim_{0};
};

}  // namespace systems
}  // namespace drake
