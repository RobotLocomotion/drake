#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

template <typename T>
class Body;

/// This %ForceElement models a torsional spring attached to a RevoluteJoint
/// and applies a torque to that joint
/// <pre>
///   τ = -k⋅(θ - θ₀)
/// </pre>
/// where θ₀ is the nominal joint position. Note that joint damping exists
/// within the RevoluteJoint itself, and so is not included here.
///
/// @tparam_default_scalar
template <typename T>
class RevoluteSpring final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteSpring)

  /// Constructor for a spring attached to the given joint
  /// @param[in] nominal_angle
  ///   The nominal angle of the spring  θ₀, in radians, at which the spring
  ///   applies no moment.
  /// @param[in] stiffness
  ///   The stiffness k of the spring in N⋅m/rad.
  /// @throws std::exception if `stiffness` is negative.
  RevoluteSpring(const RevoluteJoint<T>& joint, double nominal_angle,
                 double stiffness);

  const RevoluteJoint<T>& joint() const;

  double nominal_angle() const { return nominal_angle_; }

  double stiffness() const { return stiffness_; }

  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  T CalcNonConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

 protected:
  void DoCalcAndAddForceContribution(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<ForceElement<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class RevoluteSpring;

  RevoluteSpring(ModelInstanceIndex model_instance, JointIndex joint_index,
                 double nominal_angle, double stiffness);

  // Helper method to make a clone templated on ToScalar().
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const JointIndex joint_index_;
  double nominal_angle_;
  double stiffness_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RevoluteSpring)
