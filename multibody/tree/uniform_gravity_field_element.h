#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"

namespace drake {
namespace multibody {

/// This ForceElement allows modeling the effect of a uniform gravity field as
/// felt by bodies on the surface of the Earth or elsewhere. This gravity
/// field acts on all bodies in the MultibodyPlant except those whose
/// ModelInstance has been explicitly excluded.
///
/// @tparam_default_scalar
template <typename T>
class UniformGravityFieldElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformGravityFieldElement);

  // TODO(sherm1) Switch to the NIST standard 9.80665 value but be sure that's
  //  used consistently throughout the code, SDFs, etc.
  /// The strength used by our class's default constructor (i.e., on the
  /// Earth's surface). This is an unsigned (positive) value in m/s².
  static constexpr double kDefaultStrength = 9.81;

  /// Constructs a uniform gravity field element with a default strength (on
  /// the Earth's surface) and direction (-z).
  UniformGravityFieldElement();

  /// Constructs a uniform gravity field element with a strength and direction
  /// given by the acceleration of gravity vector `g_W`, expressed in the
  /// world frame W.
  explicit UniformGravityFieldElement(Vector3<double> g_W);

  /// Constructs a uniform gravity field element with a strength and
  /// direction given by the acceleration of gravity vector `g_W`, expressed
  /// in the world frame W. Gravity is disabled for the set of model instances
  /// `disabled_model_instances`.
  /// @see set_enabled(), is_enabled()
  UniformGravityFieldElement(
      Vector3<double> g_W,
      std::set<ModelInstanceIndex> disabled_model_instances);

  ~UniformGravityFieldElement() override;

  /// Returns the acceleration of gravity vector in m/s², expressed in the
  /// world frame W.
  const Vector3<double>& gravity_vector() const { return g_W_; }

  /// Sets the acceleration of gravity vector in m/s², expressed in the world
  /// frame W.
  void set_gravity_vector(const Vector3<double>& g_W) { g_W_ = g_W; }

  /// Checks whether gravity is enabled for a given model instance.
  /// @returns `true` iff gravity is enabled for `model_instance`.
  /// @throws std::exception if the model instance is invalid.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  bool is_enabled(ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    if (model_instance >= this->get_parent_tree().num_model_instances()) {
      throw std::logic_error("Model instance index is invalid.");
    }
    return !disabled_model_instances_.contains(model_instance);
  }

  /// Enables or disables gravity for a given `model_instance`. Only permitted
  /// pre-Finalize().
  /// @throws if the parent model is finalized.
  /// @throws std::exception if the model instance is invalid.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  void set_enabled(ModelInstanceIndex model_instance, bool is_enabled);

  /// Computes the generalized forces `tau_g(q)` due to `this` gravity field
  /// element as a function of the generalized positions `q` stored in the input
  /// `context`, for the multibody model to which `this` element belongs.
  /// `tau_g(q)` is defined such that it appears on the right hand side of the
  /// equations of motion together with any other generalized forces, like so:
  /// <pre>
  ///   M(q)v̇ + C(q, v)v = tau_g(q) + tau_app
  /// </pre>
  /// where `tau_app` includes any other forces acting on the system
  /// (converted to generalized forces).
  ///
  /// @note Given the gravitational potential energy PE(q), the generalized
  /// force at the iᵗʰ joint degree of freedom is tau_gᵢ = −∂PE(q)/∂qᵢ.
  ///
  /// @param[in] context
  ///   The context storing the state of the multibody model to which this
  ///   element belongs.
  /// @returns tau_g
  ///   A vector containing the generalized forces due to this gravity field
  ///   force element. The ordering of the generalized forces is consistent
  ///   with the vector of generalized velocities `v` for the parent
  ///   MultibodyPlant so that the inner product `v⋅tau_g` is the
  ///   gravitational power acting on the mechanical system. That is,
  ///   `v⋅tau_g > 0` corresponds to potential energy going _into_ the system,
  ///   as either mechanical kinetic energy, some other potential energy, or
  ///   heat, and therefore to a decreasing of gravitational potential energy.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const;

  /// Computes the total gravitational potential energy of all bodies in the
  /// model in this uniform gravity field. The definition of potential energy
  /// allows to arbitrarily choose the zero energy height. This element takes
  /// the zero energy height to be the same as the World origin height. That
  /// is, a body will have zero potential energy when the height of its center
  /// of mass is at the height of the World origin.
  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const final;

  /// Returns gravitational power acting on the system. This is positive when
  /// gravitational potential energy is decreasing. In terms of the generalized
  /// gravity forces `tau_g` and the generalized velocities `v`, the
  /// gravitational power is `v⋅tau_g`.
  /// @see CalcGravityGeneralizedForces()
  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const final;

  /// Returns zero always since gravity is conservative.
  T CalcNonConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const final;

 protected:
  void DoCalcAndAddForceContribution(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<ForceElement<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  std::unique_ptr<ForceElement<T>> DoShallowClone() const override;

 private:
  // Accumulates gravity spatial forces on each mobod into F_Bo_W_array,
  // which must already be sized to num_mobods.
  void AccumulateGravitySpatialForces(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      std::vector<SpatialForce<T>>* F_Bo_W_array) const;

  Vector3<double> g_W_;
  // Set of model instances for which gravity is disabled.
  std::set<ModelInstanceIndex> disabled_model_instances_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::UniformGravityFieldElement);
