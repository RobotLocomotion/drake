#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

template <typename T> class Body;

/// This %ForceElement models a torsional spring attached to a RevoluteJoint
/// to the two bodies attached to the joint,  applies equal and opposite torques
/// <pre>
///   tau = k⋅(free_length - q)
/// </pre>
/// where q_0 is the nominal joint position. Note that joint damping exists
/// within the RevoluteJoint itself, and so is not included here.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RevoluteSpring final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteSpring)

  /// Constructor for a spring attached to the given joint
  /// @param[in] free_length
  ///   The free length of the spring q_0, in radians, at which the spring
  ///   applies no moment.
  /// @param[in] stiffness
  ///   The stiffness k of the spring in N⋅m/rad. It must be non-negative.
  /// Refer to this class's documentation for further details.
  RevoluteSpring(const RevoluteJoint<T>& joint, double free_length,
                 double stiffness);

  const Joint<T>& joint() const { return joint_; }

  double free_length() const { return free_length_; }

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
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const RevoluteJoint<T>& joint_;
  double free_length_;
  double stiffness_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RevoluteSpring)
