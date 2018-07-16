#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T> class Body;

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
class SpringDamper : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringDamper)

  /// blah...
  SpringDamper(
      const Body<T>& bodyA, const Vector3<double>& p_AP,
      const Body<T>& bodyB, const Vector3<double>& p_BQ,
      double rest_length, double stiffness, double damping);

  const Body<T>& bodyA() const { return bodyA_; }

  const Body<T>& bodyB() const { return bodyB_; }

  const Vector3<double> point_on_bodyA() const { return p_AP_; }

  const Vector3<double> point_on_bodyB() const { return p_BQ_; }

  double rest_length() const { return rest_length_; }

  double stiffness() const { return stiffness_; }

  double damping() const { return damping_; }

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
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
  const MultibodyTree<ToScalar>& tree_clone) const;

  // To avoid division by zero when the length of the spring approaches zero,
  // we use a "soft norm" defined by:
  //   ‖x‖ₛ = sqrt(xᵀ⋅x + ε²)
  // where ε is a small positive value so that it's effect is negligible for
  // non-zero x.
  T SoftNorm(const Vector3<T>& x) const;

  // Helper method to compute the rate of change of the separation length
  // between the two endpoints for this spring-damper.
  T CalcLengthTimeDerivative(
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const;

  const Body<T>& bodyA_;
  const Vector3<double> p_AP_;
  const Body<T>& bodyB_;
  const Vector3<double> p_BQ_;
  double rest_length_;
  double stiffness_;
  double damping_;
};

}  // namespace multibody
}  // namespace drake
