#pragma once

#include <memory>
#include <stdexcept>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {

/**
 * Solves inverse dynamics with no consideration for joint actuator force
 * limits.
 *
 * Computes the generalized force `τ_id` that needs to be applied so that the
 * multibody system undergoes a desired acceleration `vd_d`. That is, `τ_id`
 * is the result of an inverse dynamics computation according to:
 * <pre>
 *   τ_id = M(q)vd_d + C(q, v)v - τ_g(q) - τ_app
 * </pre>
 * where `M(q)` is the mass matrix, `C(q, v)v` is the bias term containing
 * Coriolis and gyroscopic effects, `τ_g(q)` is the vector of generalized
 * forces due to gravity and `τ_app` contains applied forces from force
 * elements added to the multibody model (this can include damping, springs,
 * etc. See MultibodyPlant::CalcForceElementsContribution()).
 *
 * The system also provides a pure gravity compensation mode via an option in
 * the constructor. In this case, the output is simply
 * <pre>
 *  τ_id = -τ_g(q).
 * </pre>
 *
 * InverseDynamicsController uses a PID controller to generate desired
 * acceleration and uses this class to compute generalized forces. Use this
 * class directly if desired acceleration is computed differently.
 *
 * @system
 * name: InverseDynamics
 * input_ports:
 * - u0 (estimated state)
 * - <span style="color:gray">u1</span> (desired accelerations)
 * output_ports:
 * - y0 (force)
 * @endsystem
 *
 * Port `u0` accepts system estimated state; port `y0` emits generalized
 * forces. Port `u1` is only present when the `mode` at construction is not
 * `kGravityCompensation`. When present, `u1` accepts desired accelerations.
 *
 * @tparam_default_scalar
 * @ingroup control_systems
 */
template <typename T>
class InverseDynamics final : public LeafSystem<T> {
 public:
  enum InverseDynamicsMode {
    /// Full inverse computation mode.
    kInverseDynamics,

    /// Purely gravity compensation mode.
    kGravityCompensation
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamics)

  /**
   * Constructs the InverseDynamics system.
   *
   * @param plant Pointer to the multibody plant model. The life span of @p
   * plant must be longer than that of this instance.
   * @param mode If set to kGravityCompensation, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`).
   */
  explicit InverseDynamics(const multibody::MultibodyPlant<T>* plant,
                           InverseDynamicsMode mode = kInverseDynamics);

  /**
   * Constructs the InverseDynamics system and takes the ownership of the
   * input `plant`.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound.}
   */
  explicit InverseDynamics(std::unique_ptr<multibody::MultibodyPlant<T>> plant,
                           InverseDynamicsMode mode = kInverseDynamics);

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit InverseDynamics(const InverseDynamics<U>& other);

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
  // Other constructors delegate to this private constructor.
  InverseDynamics(std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
                  const multibody::MultibodyPlant<T>* plant,
                  InverseDynamicsMode mode);

  template <typename> friend class InverseDynamics;

  // This is the calculator method for the output port.
  void CalcOutputForce(const Context<T>& context,
                       BasicVector<T>* force) const;

  // Methods for updating cache entries.
  void SetMultibodyContext(const Context<T>&, Context<T>*) const;
  void CalcMultibodyForces(const Context<T>&,
                           multibody::MultibodyForces<T>*) const;

  const std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_{};
  const multibody::MultibodyPlant<T>* const plant_;

  // Mode dictates whether to do inverse dynamics or just gravity compensation.
  const InverseDynamicsMode mode_;

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_force_{0};

  const int q_dim_{0};
  const int v_dim_{0};

  // Note: unused in gravity compensation mode.
  drake::systems::CacheIndex external_forces_cache_index_;

  drake::systems::CacheIndex plant_context_cache_index_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::InverseDynamics)
