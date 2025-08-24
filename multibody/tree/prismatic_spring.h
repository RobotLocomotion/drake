#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/prismatic_joint.h"

namespace drake {
namespace multibody {

/// This %ForceElement models a linear spring attached to a PrismaticJoint
/// and applies a force to that joint according to
/// <pre>
///   f = -k⋅(x - x₀)
/// </pre>
/// where x₀ is the nominal (zero spring force) position in meters,
/// x is the joint position in meters, f is the spring force in Newtons and
/// k is the spring constant in N/m.
/// Note that joint damping exists within the PrismaticJoint itself, and
/// so is not included here.
///
/// @note This is different from the LinearSpringDamper: this
/// %PrismaticSpring is associated with a joint, while the LinearSpringDamper
/// connects two bodies.
/// @tparam_default_scalar
template <typename T>
class PrismaticSpring final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticSpring);

  /// Constructor for a linear spring attached to the given prismatic joint.
  /// @param[in] nominal_position
  /// The nominal position of the spring x₀, in meters, at which the spring
  /// applies no force. This is measured the same way as the generalized
  /// position of the prismatic joint.
  /// @param[in] stiffness
  /// The stiffness k of the spring in N/m.
  /// @throws std::exception if `stiffness` is (strictly) negative.
  PrismaticSpring(const PrismaticJoint<T>& joint, double nominal_position,
                  double stiffness);

  ~PrismaticSpring() override;

  /// Returns the joint associated with this spring.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  const PrismaticJoint<T>& joint() const;

  double nominal_position() const { return nominal_position_; }

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

 private:
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

  std::unique_ptr<ForceElement<T>> DoShallowClone() const override;

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class PrismaticSpring;

  // Private constructor for internal use in TemplatedDoCloneToScalar()
  PrismaticSpring(ModelInstanceIndex model_instance, JointIndex joint_index,
                  double nominal_position, double stiffness);

  // Helper method to make a clone templated on ToScalar().
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const JointIndex joint_index_;
  double nominal_position_{0};
  double stiffness_{0};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticSpring);
