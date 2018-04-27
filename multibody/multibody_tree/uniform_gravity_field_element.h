#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

/// This ForceElement allows modeling the effect of a uniform gravity field as
/// felt by bodies on the surface of the Earth.
/// This gravity fields acts on all bodies in the MultibodyTree model.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class UniformGravityFieldElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformGravityFieldElement)

  /// Constructs a uniform gravity field element with a strength given by the
  /// acceleration of gravity vector `g_W`, expressed in the world frame W.
  explicit UniformGravityFieldElement(Vector3<double> g_W);

  /// Returns the acceleration of gravity vector, expressed in the world frame
  /// W.
  const Vector3<double>& gravity_vector() const { return g_W_; }

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
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc) const final;

  T CalcConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

  T CalcNonConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

 protected:
  void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  Vector3<double> g_W_;
};

}  // namespace multibody
}  // namespace drake
