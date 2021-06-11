#pragma once

#include <memory>
#include <stdexcept>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

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
 * @see Constructors for descriptions of how (and which) forces are incorporated
 *      into the inverse dynamics computation.
 *
 * @ingroup control_systems
 * @tparam_double_only
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

  // @TODO(edrumwri) Find a cleaner way of approaching the consideration of
  // external forces. I like to imagine a dichotomy of approaches for
  // construction of this system: incorporating *no* external forces or all
  // forces on the plant. The current approach does neither: it only pledges to
  // account for exactly the forces that MultibodyPlant does.
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
   * etc. See MultibodyPlant::CalcForceElementsContribution()).
   *
   * @param plant Pointer to the multibody plant model. The life span of @p
   * plant must be longer than that of this instance.
   * @param mode If set to kGravityCompensation, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`).
   */
  InverseDynamics(const multibody::MultibodyPlant<T>* plant,
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

  // Methods for updating cache entries.
  void SetMultibodyContext(const Context<T>&, Context<T>*) const;
  void CalcMultibodyForces(const Context<T>&,
                           multibody::MultibodyForces<T>*) const;

  const multibody::MultibodyPlant<T>* const multibody_plant_;

  // Mode dictates whether to do inverse dynamics or just gravity compensation.
  const InverseDynamicsMode mode_;

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_force_{0};

  const int q_dim_{0};
  const int v_dim_{0};

  // Note: unused in gravity compensation mode.
  drake::systems::CacheIndex external_forces_cache_index_;

  drake::systems::CacheIndex multibody_plant_context_cache_index_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
