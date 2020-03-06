#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"

namespace drake {
namespace multibody {

/// This ForceElement allows modeling the effect of a uniform gravity field as
/// felt by bodies on the surface of the Earth.
/// This gravity field acts on all bodies in the MultibodyTree model.
///
/// @tparam_default_scalar
template <typename T>
class UniformGravityFieldElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformGravityFieldElement)

  // TODO(sherm1) Switch to the NIST standard 9.80665 value but be sure that's
  // used consistently throughout the code, SDFs, etc.
  /// The strength used by our class's default constructor (i.e., on the
  /// earth's surface).  This is an unsigned (positive) value in m/s².
  static constexpr double kDefaultStrength = 9.81;

  /// Constructs a uniform gravity field element with a default strength (on
  /// the earth's surface) and direction (-z).
  UniformGravityFieldElement();

  /// Constructs a uniform gravity field element with a strength given by the
  /// acceleration of gravity vector `g_W`, expressed in the world frame W.
  explicit UniformGravityFieldElement(Vector3<double> g_W);

  /// Returns the acceleration of the gravity vector in m/s², expressed in the
  /// world frame W.
  const Vector3<double>& gravity_vector() const { return g_W_; }

  /// Sets the acceleration of gravity vector, expressed in the world frame
  /// W in m/s².
  void set_gravity_vector(const Vector3<double>& g_W) {
    g_W_ = g_W;
  }

  /// Computes the generalized forces `tau_g(q)` due to `this` gravity field
  /// element as a function of the generalized positions `q` stored in the input
  /// `context`, for the multibody model to which `this` element belongs.
  /// `tau_g(q)` is defined such that it appears on the right hand side of the
  /// equations of motion together with any other generalized forces, like so:
  /// <pre>
  ///   Mv̇ + C(q, v)v = tau_g(q) + tau_app
  /// </pre>
  /// where `tau_app` includes any other generalized forces applied on the
  /// system.
  ///
  /// @param[in] context
  ///   The context storing the state of the multibody model to which this
  ///   element belongs.
  /// @returns tau_g
  ///   A vector containing the generalized forces due to this gravity field
  ///   force element. The generalized forces are consistent with the vector of
  ///   generalized velocities `v` for the parent MultibodyTree model so that
  ///   the inner product `v⋅tau_g` corresponds to the power applied by the
  ///   gravity forces on the mechanical system. That is, `v⋅tau_g > 0`
  ///   corresponds to potential energy going into the system, as either
  ///   mechanical kinetic energy, some other potential energy, or heat, and
  ///   therefore to a decrease of potential energy.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const;

  /// Computes the total potential energy of all bodies in the model in this
  /// uniform gravity field. The definition of potential energy allows to
  /// arbitrarily choose the zero energy height. This element takes the zero
  /// energy height to be the same as the world's height. That is, a body
  /// will have zero potential energy when its the height of its center of mass
  /// is at the world's origin.
  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const final;

  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const final;

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

 private:
  Vector3<double> g_W_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::UniformGravityFieldElement)
