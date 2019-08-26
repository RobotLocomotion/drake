#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T> class Body;

/// This %ForceElement models a massless bushing that connects a frame Aʙ of a
/// body A to a frame Bᴀ of a body B.  The bushing has both rotational and
/// translational stiffness and damping properties.
///
/// The bushing's rotational stiffness depends on the RollPitchYaw angles
/// q₀, q₁, q₂ and its rotational damping depends on  q̇₀, q̇₁, q̇₂.  The angles
/// are calculated from the instantaneous orientation between frames Aʙ and Bᴀ
/// and have the range `-π < q₀ <= π`, `-π/2 < q₁ <= π/2`, `-π < q₂ <= π`.
///
/// The bushing's translational stiffness is linear in displacements x, y, z and
/// the translational damping is linear in ẋ, ẏ, ż.  The displacements x, y, z
/// are defined so the position vector from Aʙₒ (frame Aʙ's origin) to Bᴀₒ
/// (frame Bᴀ's origin), expressed in frame Aʙ is `x*Aʙx + y*Aʙy + z*Aʙz`.
///
/// The set of forces exerted on body A by the bushing are equivalent to a
/// torque τ on frame Aʙ and a force f on point Aʙₒ, as
/// <pre>
/// τ = (k₀q₀ + b₀q̇₀) Aʙz  +  (k₁q₁ + b₁q̇₁) Iy  + (k₂q₂ + b₂q̇₂) Bᴀx
/// f = (kx x + bx ẋ) Aʙx  +  (ky y + by ẏ) Aʙy + (kz z + bz ż) Aʙz
/// </pre>
/// where k₁, k₂, k₃ and b₁, b₂, b₃ are torque stiffness and damping constants,
/// kx, ky, kz and bx, by, bz are force stiffness and damping constants, and
/// Iy = -sin(q₀)*Aʙx + cos(q₀)*Aʙy
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see @ref RollPitchYaw for definitions of angles q₀, q₁, q₂.
template <typename T>
class LinearBushingRollPitchYaw final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearBushingRollPitchYaw)

  /// Constructor for a roll-pitch-yaw bushing that connects bodies `A` and `B`.
  /// @param[in] frameAb frame fixed to body `A` and connected to the bushing.
  /// @param[in] frameBa frame fixed to body `B` and connected to the bushing.
  /// @param[in] torque_stiffness k₁, k₂, k₃ for roll-pitch-yaw rotation.
  /// @param[in] torque_damping b₁, b₂, b₃ for roll-pitch-yaw rates.
  /// @param[in] force_stiffness kx, ky, kz for translational displacement.
  /// @param[in] force_damping bx, by, bz for translational motion.
  /// @note Refer to this class's documentation for further details.
  /// @note The stiffness and damping parameters are usually non-negative.
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  LinearBushingRollPitchYaw(const Frame<T>& frameAb,
                            const Frame<T>& frameBa,
                            const Vector3<T>& torque_stiffness,
                            const Vector3<T>& torque_damping,
                            const Vector3<T>& force_stiffness,
                            const Vector3<T>& force_damping);

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Body<T>& bodyA() const { return frameAb_.body(); }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Body<T>& bodyB() const { return frameBa_.body(); }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Frame<T>& frameAb() const { return frameAb_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Frame<T>& frameBa() const { return frameBa_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<T>& torque_stiffness() const { return torque_stiffness_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<T>& torque_damping() const { return torque_damping_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<T>& force_stiffness() const { return force_stiffness_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<T>& force_damping() const { return force_damping_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
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
#if 0
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // Helper method to compute the rate of change of the separation length
  // between the two endpoints for this spring-damper.
  T CalcLengthTimeDerivative(
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const;
#endif

  const Frame<T>& frameAb_;
  const Frame<T>& frameBa_;
  const Vector3<T> torque_stiffness_;
  const Vector3<T> torque_damping_;
  const Vector3<T> force_stiffness_;
  const Vector3<T> force_damping_;
};


}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
