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
 * @note As an alternative to adding a controller to your diagram, gravity
 * compensation can be modeled by disabling gravity for a given model instance,
 * see MultibodyPlant::set_gravity_enabled(), unless the gravity compensation
 * needs to be accounted for when evaluating effort limits.
 *
 * InverseDynamicsController uses a PID controller to generate desired
 * acceleration and uses this class to compute generalized forces. Use this
 * class directly if desired acceleration is computed differently.
 *
 * @system
 * name: InverseDynamics
 * input_ports:
 * - estimated_state
 * - <span style="color:gray">desired_acceleration</span>
 * output_ports:
 * - generalized_force
 * @endsystem
 *
 * The desired acceleration port shown in <span style="color:gray">gray</span>
 * is only present when the `mode` at construction is not
 * `kGravityCompensation`.
 *
 * @tparam_default_scalar
 * @ingroup control_systems
 */
template <typename T>
class InverseDynamics final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamics);

  enum InverseDynamicsMode {
    /// Full inverse computation mode.
    kInverseDynamics,

    /// Purely gravity compensation mode.
    kGravityCompensation
  };

  /**
   * Constructs the InverseDynamics system.
   *
   * @param plant Pointer to the multibody plant model. The life span of
   * `plant` must be longer than that of this instance.
   * @param mode If set to kGravityCompensation, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   * @param plant_context A specific context of `plant` to use for computing
   * inverse dynamics. For example, you can use this to pass in a context with
   * modified mass parameters.  If `nullptr`, the default context of the given
   * `plant` is used. Note that this will be copied at time of construction, so
   * there are no lifetime constraints.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`). Also, `plant_context`, if provided, must be compatible with
   * `plant`.
   */
  explicit InverseDynamics(const multibody::MultibodyPlant<T>* plant,
                           InverseDynamicsMode mode = kInverseDynamics,
                           const Context<T>* plant_context = nullptr);

  /**
   * Constructs the InverseDynamics system and takes the ownership of the
   * input `plant`.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound.}
   */
  explicit InverseDynamics(std::unique_ptr<multibody::MultibodyPlant<T>> plant,
                           InverseDynamicsMode mode = kInverseDynamics,
                           const Context<T>* plant_context = nullptr);

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit InverseDynamics(const InverseDynamics<U>& other);

  ~InverseDynamics() override;

  /**
   * Returns the input port for the estimated state.
   */
  const InputPort<T>& get_input_port_estimated_state() const {
    return this->get_input_port(estimated_state_);
  }

  /**
   * Returns the input port for the desired acceleration.
   */
  const InputPort<T>& get_input_port_desired_acceleration() const {
    DRAKE_THROW_UNLESS(!this->is_pure_gravity_compensation());
    return this->get_input_port(desired_acceleration_);
  }

  /**
   * Returns the output port for the generalized forces that realize the desired
   * acceleration. The dimension of that force vector will be identical to the
   * dimensionality of the generalized velocities.
   */
  const OutputPort<T>& get_output_port_generalized_force() const {
    return this->get_output_port(generalized_force_);
  }

  bool is_pure_gravity_compensation() const {
    return mode_ == InverseDynamicsMode::kGravityCompensation;
  }

 private:
  // Other constructors delegate to this private constructor.
  InverseDynamics(std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
                  const multibody::MultibodyPlant<T>* plant,
                  InverseDynamicsMode mode, const Context<T>* plant_context);

  // Helper data structure for scalar conversion.
  struct ScalarConversionData {
    std::unique_ptr<multibody::MultibodyPlant<T>> plant;
    InverseDynamicsMode mode{InverseDynamicsMode::kGravityCompensation};
    std::unique_ptr<Context<T>> plant_context;
  };

  // Helper function for the scalar conversion constructor that extracts the
  // plant context from `other` and scalar converts it to this scalar type, T.
  template <typename U>
  static ScalarConversionData ScalarConvertHelper(
      const InverseDynamics<U>& other);

  // Delegate constructor for scalar conversion.
  explicit InverseDynamics(ScalarConversionData&& data);

  template <typename>
  friend class InverseDynamics;

  // This is the calculator method for the output port.
  void CalcOutputForce(const Context<T>& context, BasicVector<T>* force) const;

  // Methods for updating cache entries.
  void SetMultibodyContext(const Context<T>&, Context<T>*) const;
  void CalcMultibodyForces(const Context<T>&,
                           multibody::MultibodyForces<T>*) const;

  const std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_{};
  const multibody::MultibodyPlant<T>* const plant_;

  // Mode dictates whether to do inverse dynamics or just gravity compensation.
  const InverseDynamicsMode mode_;

  InputPortIndex estimated_state_;
  InputPortIndex desired_acceleration_;
  OutputPortIndex generalized_force_;

  const int q_dim_;
  const int v_dim_;

  // Note: unused in gravity compensation mode.
  CacheIndex external_forces_cache_index_;

  CacheIndex plant_context_cache_index_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::InverseDynamics);
