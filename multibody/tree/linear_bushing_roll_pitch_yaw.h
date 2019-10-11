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
/// body A to a frame Bᴀ of a body B.  The bushing can apply both a torque and
/// force due to stiffness and dissipation (spring/damper) properties.
///
/// The bushing's torque depends on roll-pitch-yaw angles q₀, q₁, q₂ and their
/// time derivatives q̇₀, q̇₁, q̇₂.  The roll-pitch-yaw angles q₀, q₁, q₂ are
/// calculated from the orientation between frames Aʙ and Bᴀ and have the range
/// `-π < q₀ <= π`, `-π/2 <= q₁ <= π/2`, `-π < q₂ <= π`.
///
/// The bushing's force depends on displacements x, y, z and their time
/// derivatives ẋ, ẏ, ż.  The displacements x, y, z are defined so the position
/// vector from Aʙₒ (frame Aʙ's origin) to Bᴀₒ (frame Bᴀ's origin) expressed
/// in frame Aʙ is `[x, y, z]`.
///
/// The set of forces on body B from the bushing is equivalent to a torque τ
/// on frame Bᴀ and a force f applied to a point Bc of B, as
/// <pre>
/// f = Fx Aʙx + Fy Aʙy + Fz Aʙz
/// τ = Tx Bᴀx + Ty Aʙy + Tz Aʙz
/// </pre>
/// where Aʙx, Aʙy, Aʙz are orthogonal unit vectors fixed in frame Aʙ,
/// and Bᴀx, Bᴀy, Bᴀz are orthogonal unit vectors fixed in frame Bᴀ.
/// The set of forces on body A from the bushing is equivalent to a torque -τ on
/// frame Aʙ and a force -f applied to a point Ac of A.  Point Ac of A and point
/// Bc of B are coincident and located halfway between origin points Aʙₒ and Bᴀₒ.
///
/// The bushing torque τ and force f are modeled as having a potential energy 𝖀
/// and a dissipation function 𝕱 of <pre>
/// `𝖀 = 1/2 (kx x² + ky y² + kz z²) + 1/2 (k₀q₀² + k₁q₁² + k₂q₂²)`
/// `𝕱 = 1/2 (bx ẋ² + by ẏ² + bz ż²) + 1/2 (b₀q̇₀² + b₁q̇₁² + b₂q̇₂²)`
/// </pre>
/// By equating the generalized forces associated with 𝖀 and 𝕱 with the
/// generalized forces produced by f and τ, one can show <pre>
/// Fx = (kx x + bx ẋ)
/// Fy = (ky y + by ẏ)
/// Fz = (kz z + bz ż)
/// Tx = (k₀q₀ + b₀q̇₀)
/// Ty = (k₁q₁ + b₁q̇₁)
/// Tz = (k₂q₂ + b₂q̇₂)
/// </pre>
/// where kx, ky, kz and bx, by, bz are force stiffness and damping constants,
/// k₀, k₁, k₂ and b₀, b₁, b₂, are torque stiffness and damping constants,
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see @ref RollPitchYaw for definitions of roll, pitch, yaw = `[q₀, q₁, q₂]`.
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
  LinearBushingRollPitchYaw(const Frame<T>& frameAb,
                            const Frame<T>& frameBa,
                            const Vector3<double>& torque_stiffness_constants,
                            const Vector3<double>& torque_damping_constants,
                            const Vector3<double>& force_stiffness_constants,
                            const Vector3<double>& force_damping_constants);

  const Body<T>& bodyA() const { return frameAb_.body(); }
  const Body<T>& bodyB() const { return frameBa_.body(); }
  const Frame<T>& frameAb() const { return frameAb_; }
  const Frame<T>& frameBa() const { return frameBa_; }

  // Returns the torque stiffness constants [k₀, k₁, k₂].
  const Vector3<double>& torque_stiffness_constants() const {
    return torque_stiffness_constants_;
  }

  // Returns the torque damping constants [b₀, b₁, b₂].
  const Vector3<double>& torque_damping_constants() const {
    return torque_damping_constants_;
  }

  // Returns the force stiffness constants [kx, ky, kz].
  const Vector3<double>& force_stiffness_constants() const {
    return force_stiffness_constants_;
  }

  // Returns the force damping constants [bx, by, bz].
  const Vector3<double>& force_damping_constants() const {
    return force_damping_constants_;
  }

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

  /// Returns X_AʙBᴀ, the rigid transform that relates frames Aʙ and Bᴀ.
  /// @param[in] context The state of the multibody system.
  /// @note: `X_AʙBᴀ.rotation()` is the rotation matrix R_AʙBᴀ that relates
  /// frames Aʙ and Bᴀ whereas `X_AʙBᴀ.translation()` is `p_AʙBᴀ = [x, y, z]`
  /// (the position from Aʙₒ to Bᴀₒ expressed in Aʙ).
  math::RigidTransform<T> CalcBushingRigidTransform(
      const systems::Context<T>& context) const {
    return frameBa().CalcPose(context, frameAb());
  }

  /// Calculates the bushing's RollPitchYaw from the rotation matrix R_AʙBᴀ that
  /// relates frames Aʙ and Bᴀ.  Note: `[roll, pitch, yaw]` = `[q₀, q₁, q₂]`.
  /// @param[in] context The state of the multibody system.
  math::RollPitchYaw<T> CalcBushingRollPitchYawAngles(
      const systems::Context<T>& context) const {
    return math::RollPitchYaw<T>(CalcBushingRigidTransform(context).rotation());
  }

  /// Calculates the bushing's RollPitchYaw rates for frame Bᴀ in frame Aʙ.
  /// @param[in] context The state of the multibody system.
  /// @retval [ṙoll, ṗitch, ẏaw] = [q̇₀, q̇₁, q̇₂].
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T>& rpy = CalcBushingRollPitchYawAngles(context);
    return CalcBushingRollPitchYawAngleRates(context, rpy);
  }

  /// Returns V_AʙBᴀ, frame Bᴀ's spatial velocity in frame Aʙ expressed in Aʙ.
  /// @param[in] context The state of the multibody system.
  /// @note: `V_AʙBᴀ.rotational()` is w_AʙBᴀ (Bᴀ's angular velocity in frame Aʙ
  /// expressed in Aʙ) whereas `V_AʙBᴀ.translational()` is `v_AʙBᴀₒ = [ẋ, ẏ, ż]`
  /// (point Bᴀₒ's translational velocity in Aʙ, expressed in frame Aʙ).
  /// Note: w_AʙBᴀ is not equal to `[q̇₀, q̇₁, q̇₂]`.
  /// @see CalcBushingRollPitchYawAngleRates() for `[q̇₀, q̇₁, q̇₂]`.
  SpatialVelocity<T> CalcBushingSpatialVelocity(
      const systems::Context<T>& context) const {
    return frameBa().CalcSpatialVelocity(context, frameAb(), frameAb());
  }

  /// Returns F_Ab_Ab, the spatial force on frame Aʙ expressed in frame Aʙ.
  /// @param[in] context The state of the multibody system.
  /// @note F_Ab_Ab is a force/torque representation of the set of all bushing
  /// forces exerted on body A. In other words, the bushing forces on body A
  /// are replaced by the set's resultant force f_Aʙₒ applied to point Aʙₒ
  /// and a torque t_Aʙ equal to the moment of the set about point Aʙₒ.
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
  // @retval `[ṙoll, ṗitch, ẏaw]` = `[q̇₀, q̇₁, q̇₂]`.
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context,
      const math::RollPitchYaw<T>& rpy) const {
    const Vector3<T> w_AbBa = CalcBushingSpatialVelocity(context).rotational();
    return rpy.CalcRpyDtFromAngularVelocityInParent(w_AbBa);
  }

  // Convert a torque τ from a non-orthogonal basis related to RollPitchYaw of
  // τ = τ₀ Bᴀx + τ₁  Iy + τ₂ Aʙz  to an orthogonal basis of the form
  // τ = τx Aʙx + τy Aʙy + τz Aʙz  where
  // Aʙx, Aʙy, Aʙz are orthogonal unit vectors fixed in frame Aʙ,
  // Bᴀx, Bᴀy, Bᴀz are orthogonal unit vectors fixed in frame Bᴀ, and
  // Iy = -sin(q₂)*Aʙx + cos(q₂)*Aʙy.  Note: The non-orthogonal basis is related
  // to a Space-fixed XYZ (intrinsic) RollPitchYaw sequence with angles
  // `[roll = q₀, pitch = q₁, yaw = q₂]`.
  Vector3<T> ConvertTorqueFromNonOrthogonalBasisToFrameAb(
      const Vector3<T>& t_nonOrthogonal_basis,
      const math::RollPitchYaw<T>& rpy) const {
    const T t0 = t_nonOrthogonal_basis(0);
    const T t1 = t_nonOrthogonal_basis(1);
    const T t2 = t_nonOrthogonal_basis(2);
    const T q1 = rpy.pitch_angle();
    const T q2 = rpy.yaw_angle();
    const T cos1 = cos(q1), sin1 = sin(q1);
    const T cos2 = cos(q2), sin2 = sin(q2);
    const T tx = cos1 * cos2 * t0 - sin2 * t1;
    const T ty = cos2 * t1 + sin2 * cos1 * t0;
    const T tz = t2 - sin1 * t0;
    return Vector3<T>(tx, ty, tz);
  }

  // Use the bushing's torque stiffness constants and its roll-pitch-yaw angles
  // to form the bushing's stiffness torque τₛ on frame Aʙ.
  // @param[in] context The state of the multibody system.
  // param[in] is_express_in_frameAb is `true` if this method is to return τₛ
  // expressed in frame Aʙ whereas `false` returns `[k₀q₀, k₁q₁, k₂q₂]`.
  // @see ConvertTorqueFromNonOrthogonalBasisToFrameAb() for more information.
  Vector3<T> CalcBushingTorqueStiffnessOnAb(
      const systems::Context<T>& context,
      const bool is_express_in_frameAb) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> ki_qi =
        torque_stiffness_constants().cwiseProduct(rpy.vector());
    return is_express_in_frameAb == false ? ki_qi :
           ConvertTorqueFromNonOrthogonalBasisToFrameAb(ki_qi, rpy);
  }

  // Use the bushing's torque damping constants and its roll-pitch-yaw rates
  // to form the bushing's damping torque τᵣ on frame Aʙ.
  // @param[in] context The state of the multibody system.
  // param[in] is_express_in_frameAb is `true` if this method is to return τᵣ
  // expressed in frame Aʙ whereas `false` returns `[b₀q̇₀, b₁q̇₁, b₂q̇₂]`.
  // @see ConvertTorqueFromNonOrthogonalBasisToFrameAb() for more information.
  Vector3<T> CalcBushingTorqueDampingOnAb(
      const systems::Context<T>& context,
      const bool is_express_in_frameAb) const {
    const math::RollPitchYaw<T>& rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context, rpy);
    const Vector3<T> bi_qiDt = torque_damping_constants().cwiseProduct(rpyDt);
    return is_express_in_frameAb == false ? bi_qiDt :
           ConvertTorqueFromNonOrthogonalBasisToFrameAb(bi_qiDt, rpy);
  }

  // Returns [kx*x, ky*y, kz*z], the bushing's stiffness force on point Aʙₒ
  // expressed in frame Aʙ.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingForceStiffnessOnAbo(
      const systems::Context<T>& context) const {
    const Vector3<T> xyz = CalcBushingRigidTransform(context).translation();
    return force_stiffness_constants().cwiseProduct(xyz);
  }

  // Returns [bx*ẋ, by*ẏ, bz*ż], the bushing's damping force on point Aʙₒ
  // expressed in frame Aʙ.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingForceDampingOnAbo(
      const systems::Context<T>& context) const {
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
