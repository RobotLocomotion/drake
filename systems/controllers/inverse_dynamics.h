#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

// Forward declaration keeps us from including RBT headers that significantly
// slow compilation.
template <class T>
class RigidBodyTree;

namespace drake {
namespace systems {
namespace controllers {

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

  DRAKE_DEPRECATED("Please use constructor with InverseDynamicsType.")
  InverseDynamics(const RigidBodyTree<T>* tree, bool pure_gravity_compensation);

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

  DRAKE_DEPRECATED("Please use constructor with InverseDynamicsType.")
  InverseDynamics(const multibody::multibody_plant::MultibodyPlant<T>* plant,
                  bool pure_gravity_compensation);

  // @TODO(edrumwri) Find a cleaner way of approaching the consideration of
  // external forces. I like to imagine a dichotomy of approaches for
  // construction of this system: incorporating *no* external forces or all
  // forces on the plant. The current approach does neither: it only pledges to
  // account for exactly the forces that MultibodyTree does.
  /**
   * Computes the generalized force `tau_id` that needs to be applied so that
   * the multibody system undergoes a desired acceleration `vd_d`. That is,
   * `tau_id` is the result of an inverse dynamics computation according to:
   * <pre>
   *   tau_id = M(q)vd_d + C(q, v)v - tau_g(q) - tau_app
   * </pre>
   * where `M(q)` is the mass matrix, `C(q, v)v` is the bias term containing
   * Coriolis and gyroscopic effects, `tau_g(q)` is the vector of generalized
   * forces due to gravity and `tau_app` contains applied forces from force
   * elements added to the multibody model (this can include damping, springs,
   * etc. See MultibodyTree::CalcForceElementsContribution()).
   *
   * @param plant Pointer to the multibody plant model. The life span of @p
   * plant must be longer than that of this instance.
   * @param mode If set to kGravityCompensation, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`).
   */
  InverseDynamics(const multibody::multibody_plant::MultibodyPlant<T>* plant,
                  InverseDynamicsMode mode);

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
   * Returns the output port for the actuation torques.
   */
  DRAKE_DEPRECATED("Please use get_output_port_force().")
  const OutputPort<T>& get_output_port_torque() const {
    return this->get_output_port(output_port_index_force_);
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
  const multibody::multibody_plant::MultibodyPlant<T>* multibody_plant_{
      nullptr};

  // Mode dictates whether to do inverse dynamics or just gravity compensation.
  const InverseDynamicsMode mode_;

  // This context is used solely for setting generalized positions and
  // velocities in multibody_plant_.
  std::unique_ptr<Context<T>> multibody_plant_context_;

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_force_{0};

  const int q_dim_{0};
  const int v_dim_{0};
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
