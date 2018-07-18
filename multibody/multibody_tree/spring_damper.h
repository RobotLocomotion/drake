#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T> class Body;

/// This %ForceElement models a spring-damper attached between two points on
/// two different bodies.
/// Given a point P on a body A and a point Q on a body B with positions
/// p_AP and p_BQ, respectively, this spring-damper applies equal and
/// opposite forces on bodies A and B according to: <pre>
///   f_AP = (k⋅(ℓ - ℓ₀) + c⋅dℓ/dt)⋅r̂
///   f_BQ = -f_AP
/// </pre>
/// where `ℓ = ‖p_WQ - p_WP‖` is the current length of the spring, dℓ/dt its
/// rate of change, `r̂ = (p_WQ - p_WP) / ℓ` is the normalized vector from P to
/// Q, ℓ₀ is the free length of the spring and k and c are the stiffness and
/// damping of the spring-damper, respectively.
/// Note that:
///   - The applied force is always along the line connecting points P and Q.
///   - Damping always dissipates energy.
///   - Forces on bodies A and B are equal and opposite according to Newton's
///     third law.
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
class LinearSpringDamper final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSpringDamper)

  /// Constructor for a spring-damper between a point P on `bodyA` and a
  /// point Q on `bodyB`. Point P is defined by its position `p_AP` as
  /// measured and expressed in the body frame A and similarly, point Q is
  /// defined by its position p_BQ as measured and expressed in body frame B.
  /// The remaining parameters define:
  /// @param[in] free_length
  ///   The free length of the spring ℓ₀, in meters, at which the spring
  ///   applies no forces. It must be non-negative.
  /// @param[in] stiffness
  ///   The stiffness k of the spring in N/m. It must be non-negative.
  /// @param[in] damping
  ///   The damping c of the damper in N⋅s/m. It must be non-negative.
  /// Refer to this class's documentation for further details.
  /// @throws std::exception if `free_length` is negative.
  /// @throws std::exception if `stiffness` is negative.
  /// @throws std::exception if `damping` is negative.
  LinearSpringDamper(
      const Body<T>& bodyA, const Vector3<double>& p_AP,
      const Body<T>& bodyB, const Vector3<double>& p_BQ,
      double free_length, double stiffness, double damping);

  const Body<T>& bodyA() const { return bodyA_; }

  const Body<T>& bodyB() const { return bodyB_; }

  /// The position p_AP of point P on body A as measured and expressed in body
  /// frame A.
  const Vector3<double> p_AP() const { return p_AP_; }

  /// The position p_BQ of point Q on body B as measured and expressed in body
  /// frame B.
  const Vector3<double> p_BQ() const { return p_BQ_; }

  double free_length() const { return free_length_; }

  double stiffness() const { return stiffness_; }

  double damping() const { return damping_; }

  T CalcPotentialEnergy(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc) const override;

  T CalcConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const override;

  T CalcNonConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const override;

 protected:
  void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // To avoid a division by zero when computing a normalized vector from point P
  // on body A to point Q on body B as length of the spring approaches zero,
  // we use a "soft norm" defined by:
  //   ‖x‖ₛ = sqrt(xᵀ⋅x + ε²)
  // where ε is a small positive value so that its effect is negligible for
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
  double free_length_;
  double stiffness_;
  double damping_;
};

}  // namespace multibody
}  // namespace drake
