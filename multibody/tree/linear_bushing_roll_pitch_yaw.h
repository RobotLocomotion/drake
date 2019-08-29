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
/// are calculated from the orientation between frames Aʙ and Bᴀ and have the
/// range `-π < q₀ <= π`, `-π/2 < q₁ <= π/2`, `-π < q₂ <= π`.
///
/// The bushing's translational stiffness is linear in displacements x, y, z and
/// the translational damping is linear in ẋ, ẏ, ż.  The displacements x, y, z
/// are defined so the position vector from Aʙₒ (frame Aʙ's origin) to Bᴀₒ
/// (frame Bᴀ's origin), expressed in frame Aʙ is `[x, y, z]`.
///
/// The set of forces on body A from the bushing is equivalent to a torque τ
/// on frame Aʙ and a force f applied to point Aʙₒ, as
/// <pre>
/// f = (kx x + bx ẋ) Aʙx  +  (ky y + by ẏ) Aʙy + (kz z + bz ż) Aʙz
/// τ = (k₀q₀ + b₀q̇₀) Aʙz  +  (k₁q₁ + b₁q̇₁) Iy  + (k₂q₂ + b₂q̇₂) Bᴀx
/// </pre>
/// where kx, ky, kz and bx, by, bz are force stiffness and damping constants,
/// k₀, k₁, k₂ and b₀, b₁, b₂, are torque stiffness and damping constants,
/// Aʙx, Aʙy, Aʙz are orthogonal unit vectors fixed in frame Aʙ,
/// Bᴀx, Bᴀy, Bᴀz are orthogonal unit vectors fixed in frame Bᴀ, and
/// Iy = -sin(q₀)*Aʙx + cos(q₀)*Aʙy.  Note: τ is expressed in a non-orthogonal
/// basis related to a RollPitchYaw sequence - which is a Body-fixed (intrinsic)
/// Z-Y-X rotation by angles `[yaw = q₀, pitch = q₁, roll = q₂]` or equivalently
/// a Space-fixed (extrinsic) X-Y-Z rotation by `[roll, pitch, yaw]`.
///
/// The set of forces on body B from the bushing is equivalent to a torque -τ
/// on frame Bᴀ and a force -f applied to the point of B coincident with Aʙₒ.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see @ref RollPitchYaw for definitions of yaw, pitch, roll = `[q₀, q₁, q₂]`.
template <typename T>
class LinearBushingRollPitchYaw final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearBushingRollPitchYaw)

  /// Constructor for a roll-pitch-yaw bushing that connects bodies `A` and `B`.
  /// @param[in] frameAb the frame of body `A` that connects to the bushing.
  /// @param[in] frameBa the frame of body `B` that connects to the bushing.
  /// @param[in] torque_stiffness_constants For torque τ, stiffness constants
  /// `[k₀, k₁, k₂]` associated with angles `[q₀, q₁, q₂]`.
  /// @param[in] torque_damping_constants For torque τ, damping constants
  /// `[b₀, b₁, b₂]` associated with angular rates `[q̇₀, q̇₁, q̇₂]`.
  /// @param[in] force_stiffness_constants For force f, stiffness constants
  /// `[kx, ky, kz]` associated with translational displacement `[x, y, z]`
  /// @param[in] force_damping_constants For force f, damping constants
  /// `[bx, by, bz]` associated with translational rates `[ẋ, ẏ, ż]`.
  /// @note Refer to this class's documentation for details about τ, f, q₀, etc.
  /// @note The stiffness and damping parameters are usually non-negative.
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  LinearBushingRollPitchYaw(const Frame<T>& frameAb,
                            const Frame<T>& frameBa,
                            const Vector3<double>& torque_stiffness_constants,
                            const Vector3<double>& torque_damping_constants,
                            const Vector3<double>& force_stiffness_constants,
                            const Vector3<double>& force_damping_constants);

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Body<T>& bodyA() const { return frameAb_.body(); }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Body<T>& bodyB() const { return frameBa_.body(); }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Frame<T>& frameAb() const { return frameAb_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Frame<T>& frameBa() const { return frameBa_; }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<double>& torque_stiffness_constants() const {
    return torque_stiffness_constants_;
  }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<double>& torque_damping_constants() const {
    return torque_damping_constants_;
  }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<double>& force_stiffness_constants() const {
    return force_stiffness_constants_;
  }

  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  const Vector3<double>& force_damping_constants() const {
    return force_damping_constants_;
  }

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

  // Returns X_AʙBᴀ, the rigid transform that relates frames Aʙ and Bᴀ.
  math::RigidTransform<T> CalcBushingRigidTransform(
      const systems::Context<T>& context) const {
    return frameBa().CalcPose(context, frameAb());
  }

  // Returns R_AʙBᴀ, the rotation matrix that relates frames Aʙ and Bᴀ.
  math::RotationMatrix<T> CalcBushingRotationMatrix(
      const systems::Context<T>& context) const {
    return CalcBushingRigidTransform(context).rotation();
  }

  // Calculates the roll-pitch-yaw angles q₀, q₁, q₂ from the rotation matrix
  // R_AʙBᴀ that relates frames Aʙ and Bᴀ.
  math::RollPitchYaw<T> CalcBushingRollPitchYawAngles(
      const systems::Context<T>& context) const {
    return math::RollPitchYaw<T>(CalcBushingRotationMatrix(context));
  }

  // Reverses roll-pitch-yaw order to return yaw-pitch-roll `[q₀, q₁, q₂]` which
  // are calculated from the rotation matrix that relates frames Aʙ and Bᴀ.
  Vector3<T> CalcBushingBodyZYXAngleSequenceYawRollPitch(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return Vector3<T>(rpy.yaw_angle(), rpy.pitch_angle(), rpy.roll_angle());
  }

  // Returns p_AʙBᴀ, the position vector from Aʙₒ to Bᴀₒ, expressed in frame Aʙ.
  Vector3<T> CalcBushingPositionVector(
      const systems::Context<T>& context) const {
    return CalcBushingRigidTransform(context).translation();
  }

  // Returns V_AʙBᴀ, frame Bᴀ's spatial velocity in frame Aʙ, expressed in Aʙ.
  SpatialVelocity<T> CalcBushingSpatialVelocity(
      const systems::Context<T>& context) const {
    return frameBa().CalcSpatialVelocity(context, frameAb(), frameAb());
  }

  // Returns v_AʙBᴀ, point Bᴀₒ's translational velocity in Aʙ, expressed in
  // frame Aʙ.
  Vector3<T> CalcBushingTranslationalVelocity(
      const systems::Context<T>& context) const {
     return CalcBushingSpatialVelocity(context).translational();
  }

  // Returns w_AʙBᴀ, frame Bᴀ's angular velocity in frame Aʙ, expressed in Aʙ.
  Vector3<T> CalcBushingAngularVelocity(
      const systems::Context<T>& context) const {
    return CalcBushingSpatialVelocity(context).rotational();
  }

  // Calculates roll-pitch-yaw angle rates q̇₀, q̇₁, q̇₂ for frame Bᴀ in frame Aʙ.
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T>& rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> w_AbBa = CalcBushingAngularVelocity(context);
    return rpy.CalcRpyDtFromAngularVelocityInParent(w_AbBa);
  }

  // Use torque stiffness together with the roll-pitch-yaw angles q₀, q₁, q₂
  // to form the bushing's resultant torque on frame Aʙ , expressed in frame Aʙ.
  Vector3<T> CalcBushingTorqueStiffnessOnAb(
      const systems::Context<T>& context) const {
    const Vector3<T> ypr = CalcBushingBodyZYXAngleSequenceYawRollPitch(context);
    const Vector3<T> t_Ab_Ab = torque_stiffness_constants().cwiseProduct(ypr);
    return t_Ab_Ab;  // TODO(Mitiguy) This is wrong.
  }

  // Use force damping together with v_AʙBᴀₒ (point Bᴀₒ's translational velocity
  // in frame Aʙ, expressed in frame Aʙ) to form the bushing's resultant damping
  // force on point Aʙₒ, expressed in frame Aʙ.
  Vector3<T> CalcBushingTorqueDampingOnAb(
      const systems::Context<T>& context) const {
    const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context);
    const Vector3<T> t_Ab_Ab = torque_damping_constants().cwiseProduct(rpyDt);
    return t_Ab_Ab;  // TODO(Mitiguy) This is wrong.
  }

  // Use force stiffness together with p_AʙBᴀ (the position vector from point
  // Aʙₒ to point Bᴀₒ, expressed in frame Aʙ) to form the bushing's resultant
  // stiffness force on point Aʙₒ, expressed in frame Aʙ.
  Vector3<T> CalcBushingForceStiffnessOnAbo(
      const systems::Context<T>& context) const {
    const Vector3<T> p_AbBa = CalcBushingPositionVector(context);
    return force_stiffness_constants().cwiseProduct(p_AbBa);
  }

  // Use force damping together with v_AʙBᴀₒ (point Bᴀₒ's translational velocity
  // in frame Aʙ, expressed in frame Aʙ) to form the bushing's resultant damping
  // force on point Aʙₒ, expressed in frame Aʙ.
  Vector3<T> CalcBushingForceDampingOnAbo(
      const systems::Context<T>& context) const {
    const Vector3<T> v_AbBa = CalcBushingTranslationalVelocity(context);
    return force_damping_constants().cwiseProduct(v_AbBa);
  }

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const Frame<T>& frameAb_;
  const Frame<T>& frameBa_;

  // TODO(Mitiguy) It would be better to instantiate all the data members in
  // this class on type <T>, not just specialize to <double>.  I tried this,
  // but there are problems with clone methods if on <T>, having to do with
  // converting a symbolic expression to type double.
  const Vector3<double> torque_stiffness_constants_;
  const Vector3<double> torque_damping_constants_;
  const Vector3<double> force_stiffness_constants_;
  const Vector3<double> force_damping_constants_;
};


}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
