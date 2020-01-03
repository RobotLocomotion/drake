#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

// Forward declaration keeps us from including RBT headers that significantly
// slow compilation.
template <class T>
class RigidBodyTree;

namespace drake {
namespace systems {
namespace controllers {
namespace rbt {   // Extra namespace to de-conflict vs the non-attic classname.

/**
 * Solves inverse dynamics with no consideration for joint actuator force
 * limits. The system also provides a pure gravity compensation mode. This
 * system provides a BasicVector input port for the state `(q, v)`, where `q`
 * is the generalized position and `v` is the generalized velocity, and a
 * BasicVector output port for the computed generalized forces. There is an
 * additional BasicVector input port for desired acceleration when configured
 * to be **not** in pure gravity compensation mode.
 *
 * InverseDynamicsController uses a PID controller to generate desired
 * acceleration and uses this class to compute generalized forces. This class
 * should be used directly if desired acceleration is computed differently.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 * @see Constructors for descriptions of how (and which) forces are incorporated
 *      into the inverse dynamics computation.
 *
 * Instantiated templates for the following kinds of T's are provided:
 *
 * - double
 */
template <typename T>
class InverseDynamics : public LeafSystem<T> {
 public:
  enum InverseDynamicsMode {
    /// Full inverse computation mode.
    kInverseDynamics,

    /// Purely gravity compensation mode.
    kGravityCompensation
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamics)

  /**
   * Computes inverse dynamics for `tree`, where the computed force `tau_id`
   * is: <pre>
   *   tau_id = `M(q)vd_d + C(q, v)v - tau_g(q) - tau_s(q) + tau_d(v)`
   * </pre>
   * where `M(q)` is the mass matrix, `C(q, v)v` is the Coriolis term,
   * `tau_g(q)` is the gravity term, `q` is the generalized position, `v` is the
   * generalized velocity, `vd_d` is the desired generalized acceleration,
   * `tau_s` is computed via `RigidBodyTree::CalcGeneralizedSpringForces()` and
   * `tau_d` is computed via `RigidBodyTree::frictionTorques()`.
   * In gravity compensation mode, the generalized force only includes the
   * gravity term, that is, `tau_id = -tau_g(q)`.
   * @param tree Pointer to the model. The life span of @p tree must be longer
   * than this instance.
   * @param mode If set to kGravityCompensation, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   */
  InverseDynamics(const RigidBodyTree<T>* tree, InverseDynamicsMode mode);

  ~InverseDynamics() override;

  /**
   * Returns the input port for the estimated state.
   */
  const InputPort<T>& get_input_port_estimated_state() const {
    return this->get_input_port(input_port_index_state_);
  }

  /**
   * Returns the input port for the desired acceleration.
   */
  const InputPort<T>& get_input_port_desired_acceleration() const {
    DRAKE_DEMAND(!this->is_pure_gravity_compensation());
    return this->get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for the generalized forces that realize the desired
   * acceleration. The dimension of that force vector will be identical to the
   * dimensionality of the generalized velocities.
   */
  const OutputPort<T>& get_output_port_force() const {
    return this->get_output_port(output_port_index_force_);
  }

  bool is_pure_gravity_compensation() const {
    return mode_ == InverseDynamicsMode::kGravityCompensation;
  }

 private:
  // This is the calculator method for the output port.
  void CalcOutputForce(const Context<T>& context,
                       BasicVector<T>* force) const;

  const RigidBodyTree<T>* rigid_body_tree_{nullptr};

  // Mode dictates whether to do inverse dynamics or just gravity compensation.
  const InverseDynamicsMode mode_;

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_force_{0};

  const int q_dim_{0};
  const int v_dim_{0};
};

}  // namespace rbt
}  // namespace controllers
}  // namespace systems
}  // namespace drake
