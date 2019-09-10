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
/// The bushing's rotational stiffness depends on the yaw-pitch-roll angles
/// q₀, q₁, q₂ and its rotational damping depends on q̇₀, q̇₁, q̇₂.  The angles
/// q₀, q₁, q₂ are calculated from the orientation between frames Aʙ and Bᴀ and
/// have the range `-π < q₀ <= π`, `-π/2 < q₁ <= π/2`, `-π < q₂ <= π`.
///
/// The bushing's translational stiffness is linear in displacements x, y, z and
/// the translational damping is linear in ẋ, ẏ, ż.  The displacements x, y, z
/// are defined so the position vector from Aʙₒ (frame Aʙ's origin) to Bᴀₒ
/// (frame Bᴀ's origin) expressed in frame Aʙ is `[x, y, z]`.
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
/// Iy = -sin(q₀)*Aʙx + cos(q₀)*Aʙy.  Above, τ is expressed in a non-orthogonal
/// basis related to a RollPitchYaw sequence - which is a Body-fixed (intrinsic)
/// Z-Y-X rotation by angles `[yaw = q₀, pitch = q₁, roll = q₂]`.
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

  /// Constructor for a RollPitchYaw bushing that connects bodies `A` and `B`.
  /// @param[in] frameAb the frame Aʙ of body `A` that connects to the bushing.
  /// @param[in] frameBa the frame Bᴀ of body `B` that connects to the bushing.
  /// @param[in] torque_stiffness_constants For torque τ, the stiffness
  /// constants `[k₀, k₁, k₂]` associated with angles `[q₀, q₁, q₂]`.
  /// @param[in] torque_damping_constants For torque τ, the damping
  /// constants `[b₀, b₁, b₂]` associated with angular rates `[q̇₀, q̇₁, q̇₂]`.
  /// @param[in] force_stiffness_constants For force f, the stiffness constants
  /// `[kx, ky, kz]` associated with translational displacement `[x, y, z]`
  /// @param[in] force_damping_constants For force f, the damping constants
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

  /// Returns X_AʙBᴀ, the rigid transform that relates frames Aʙ and Bᴀ.
  /// @param[in] context The state of the multibody system.
  /// @note: `X_AʙBᴀ.rotation()` is the rotation matrix R_AʙBᴀ that relates
  /// frames Aʙ and Bᴀ whereas `X_AʙBᴀ.translation()` is `p_AʙBᴀ = [x, y, z]`
  /// (the position from Aʙₒ to Bᴀₒ expressed in Aʙ).
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  math::RigidTransform<T> CalcBushingRigidTransform(
      const systems::Context<T>& context) const {
    return frameBa().CalcPose(context, frameAb());
  }

  /// Calculates the bushing's RollPitchYaw from the rotation matrix R_AʙBᴀ that
  /// relates frames Aʙ and Bᴀ.  Note: `[q₀, q₁, q₂]` is `[yaw, pitch, roll]`.
  /// @param[in] context The state of the multibody system.
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  math::RollPitchYaw<T> CalcBushingRollPitchYawAngles(
      const systems::Context<T>& context) const {
    return math::RollPitchYaw<T>(CalcBushingRigidTransform(context).rotation());
  }

  /// Returns V_AʙBᴀ, frame Bᴀ's spatial velocity in frame Aʙ expressed in Aʙ.
  /// @param[in] context The state of the multibody system.
  /// @note: `V_AʙBᴀ.rotational()` is w_AʙBᴀ (Bᴀ's angular velocity in frame Aʙ
  /// expressed in Aʙ) whereas `V_AʙBᴀ.translational()` is `v_AʙBᴀₒ = [ẋ, ẏ, ż]`
  /// (point Bᴀₒ's translational velocity in Aʙ, expressed in frame Aʙ).
  /// Note: w_AʙBᴀ is not equal to either `[q̇₀, q̇₁, q̇₂]` or `[q̇₂, q̇₁, q̇₀]`.
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  SpatialVelocity<T> CalcBushingSpatialVelocity(
      const systems::Context<T>& context) const {
    return frameBa().CalcSpatialVelocity(context, frameAb(), frameAb());
  }

  /// Returns F_Ab_Ab, the spatial force on frame Aʙ expressed in frame Aʙ.
  /// @param[in] context The state of the multibody system.
  /// @note The set of forces exerted on body A by the bushing are replaced by
  /// a force f_Aʙₒ equal to the set's resultant applied to point Aʙₒ and
  /// a torque t_Aʙ Aequal to the moment of the set about point Aʙₒ.
  /// F_Ab_Ab.rotational() is t_Aʙ whereas F_Ab_Ab.translational() is f_Aʙₒ.
  SpatialForce<T> CalcBushingResultantSpatialForceOnAbo(
      const systems::Context<T>& context) const {
    // Form the torque that is equal to the moment of the bushing forces on
    // body A about point Aʙₒ expressed in frame Aʙ.
    const Vector3<T> t_Ab_Ab = CalcBushingTorqueStiffnessOnAb(context, true)
                             + CalcBushingTorqueDampingOnAb(context, true);

    // Form the bushing's resultant force on point Aʙₒ expressed in frame Aʙ.
    const Vector3<T> f_Abo_Ab = CalcBushingForceStiffnessOnAbo(context)
                              + CalcBushingForceDampingOnAbo(context);

    return SpatialForce<T>(t_Ab_Ab, f_Abo_Ab);
  }

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

  // Calculates the bushing's RollPitchYaw rates for frame Bᴀ in frame Aʙ.
  // @param[in] context The state of the multibody system.
  // @param[in] rpy RollPitchYaw orientation relating frames Aʙ and Bᴀ.
  // @param[in] is_return_in_yaw_pitch_roll_order is `true` if this method
  // is to return [q̇₀, q̇₁, q̇₂] = [ẏaw, ṗitch, ṙoll] whereas `false` returns
  // [q̇₂, q̇₁, q̇₀] = [ṙoll, ṗitch, ẏaw]
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context,
      const math::RollPitchYaw<T>& rpy,
      const bool is_return_in_yaw_pitch_roll_order) const {
    const Vector3<T> w_AbBa = CalcBushingSpatialVelocity(context).rotational();
    const Vector3<T> rpyDt = rpy.CalcRpyDtFromAngularVelocityInParent(w_AbBa);
    return is_return_in_yaw_pitch_roll_order == false ? rpyDt :
           Vector3<T>(rpyDt(2), rpyDt(1), rpyDt(0));
  }

  // Calculates the bushing's RollPitchYaw rates for frame Bᴀ in frame Aʙ.
  // @see related CalcBushingRollPitchYawAngleRates() method for information.
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context,
      const bool is_return_in_yaw_pitch_roll_order) const {
    const math::RollPitchYaw<T>& rpy = CalcBushingRollPitchYawAngles(context);
    return CalcBushingRollPitchYawAngleRates(context, rpy,
                                             is_return_in_yaw_pitch_roll_order);
  }

  // Convert a torque τ from a non-orthogonal basis related to RollPitchYaw of
  // τ = τ₀ Aʙz + τ₁ Iy  + τ₂ Bᴀx  to an orthogonal basis of the form
  // τ = τx Aʙx + τy Aʙy + τz Aʙz  where
  // Aʙx, Aʙy, Aʙz are orthogonal unit vectors fixed in frame Aʙ,
  // Bᴀx, Bᴀy, Bᴀz are orthogonal unit vectors fixed in frame Bᴀ, and
  // Iy = -sin(q₀)*Aʙx + cos(q₀)*Aʙy.  Note: The non-orthogonal basis is related
  // to a RollPitchYaw rotation sequence - which is a Body-fixed (intrinsic)
  // Z-Y-X rotation by angles `[yaw = q₀, pitch = q₁, roll = q₂]`.
  Vector3<T> ConvertTorqueFromNonOrthogonalBasisToFrameAb(
      const Vector3<T>& t_nonOrthogonal_basis,
      const math::RollPitchYaw<T>& rpy) const {
    const T t0 = t_nonOrthogonal_basis(0);
    const T t1 = t_nonOrthogonal_basis(1);
    const T t2 = t_nonOrthogonal_basis(2);
    const T q0 = rpy.yaw_angle();
    const T q1 = rpy.pitch_angle();
    const T cos0 = cos(q0), sin0 = sin(q0);
    const T cos1 = cos(q1), sin1 = sin(q1);
    const T tx = cos0 * cos1 * t2 - sin0 * t1;
    const T ty = cos0 * t1 + sin0 * cos1 * t2;
    const T tz = t0 - sin1 * t2;
    return Vector3<T>(tx, ty, tz);
  }

  // Use the bushing's torque stiffness constants and its roll-pitch-yaw angles
  // to form the bushing's stiffness torque τₛ on frame Aʙ.
  // @param[in] context The state of the multibody system.
  // @param[in] is_express_in_frame_Ab is `true` if this method is to return
  // τₛ expressed in frame Aʙ whereas `false` returns `[k₀q₀, k₁q₁, k₂q₂]`.
  // Note: `false` means τₛ is expressed in a non-orthogonal basis as
  // τₛ = k₀q₀ Aʙz  +  k₁q₁ Iy  + k₂q₂ Bᴀx,
  // where k₀, k₁, k₂ are torque stiffness constants,
  // Aʙx, Aʙy, Aʙz are orthogonal unit vectors fixed in frame Aʙ,
  // Bᴀx, Bᴀy, Bᴀz are orthogonal unit vectors fixed in frame Bᴀ, and
  // Iy = -sin(q₀)*Aʙx + cos(q₀)*Aʙy.  The non-orthogonal basis Aʙz, Iy, Bᴀx
  // arises from the RollPitchYaw sequence - which is a Body-fixed Z-Y-X
  // rotation by angles `[yaw = q₀, pitch = q₁, roll = q₂]`
  Vector3<T> CalcBushingTorqueStiffnessOnAb(
      const systems::Context<T>& context,
      const bool is_express_results_in_frame_Ab) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> ypr = rpy.ConvertAnglesToSequenceBodyZYXYawRollPitch();
    const Vector3<T> torque_kiqi =
        torque_stiffness_constants().cwiseProduct(ypr);
    return is_express_results_in_frame_Ab == false ? torque_kiqi :
           ConvertTorqueFromNonOrthogonalBasisToFrameAb(torque_kiqi, rpy);
  }

  // Use the bushing's torque damping constants and its roll-pitch-yaw rates
  // to form the bushing's damping torque τᵣ on frame Aʙ.
  // @param[in] context The state of the multibody system.
  // @param[in] is_express_in_frame_Ab is `true` if this method is to return
  // τᵣ expressed in frame Aʙ whereas `false` returns `[b₀q̇₀, b₁q̇₁, b₂q̇₂]`.
  // @see CalcBushingTorqueStiffnessOnAb() for non-orthogonal basis information.
  Vector3<T> CalcBushingTorqueDampingOnAb(
      const systems::Context<T>& context,
      const bool is_express_results_in_frame_Ab) const {
    const math::RollPitchYaw<T>& rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> yprDt =
        CalcBushingRollPitchYawAngleRates(context, rpy, true);
    const Vector3<T> torque_biqiDt =
        torque_damping_constants().cwiseProduct(yprDt);
    return is_express_results_in_frame_Ab == false ? torque_biqiDt :
           ConvertTorqueFromNonOrthogonalBasisToFrameAb(torque_biqiDt, rpy);
  }

  // Calculates [kx*x, ky*y, kz*z], the bushing's stiffness force on point Aʙₒ
  // expressed in frame Aʙ.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingForceStiffnessOnAbo(
      const systems::Context<T>& context) const {
    // Use the bushing's force stiffness constants [kx, ky, kz] and
    // `p_AʙBᴀ = [x, y, z]` (the position vector from point Aʙₒ to point Bᴀₒ
    // expressed in frame Aʙ) to form [kx*x, ky*y, kz*z].
    const Vector3<T> xyz = CalcBushingRigidTransform(context).translation();
    return force_stiffness_constants().cwiseProduct(xyz);
  }

  // Calculates [bx*ẋ, by*ẏ, bz*ż], the bushing's damping force on point Aʙₒ
  // expressed in frame Aʙ.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingForceDampingOnAbo(
      const systems::Context<T>& context) const {
    // Use the bushing's force damping constants [bx, by, bz] and
    // `v_AʙBᴀₒ = [ẋ, ẏ, ż]` (Bᴀₒ's translational velocity in frame Aʙ
    // expressed in frame Aʙ) to form [bx*ẋ, by*ẏ, bz*ż].
    const Vector3<T> xyzDt =
        CalcBushingSpatialVelocity(context).translational();
    return force_damping_constants().cwiseProduct(xyzDt);
  }

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const Frame<T>& frameAb_;
  const Frame<T>& frameBa_;

  // TODO(Mitiguy) Improve this class by upgrading all data members to type <T>
  //  -- not specialized to <double>.  There were problems with clone methods
  //  having to do with converting a symbolic expression to type double.
  const Vector3<double> torque_stiffness_constants_;
  const Vector3<double> torque_damping_constants_;
  const Vector3<double> force_stiffness_constants_;
  const Vector3<double> force_damping_constants_;
};


}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
