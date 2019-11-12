#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T> class Body;

/// This %ForceElement models a massless bushing that connects a frame Aʙ of a
/// body A to a frame Bᴀ of a body B.  The bushing can apply both a torque and
/// force due to stiffness (spring) and dissipation (damper) properties.
///
/// The set of forces on body B from the bushing is equivalent to a torque τ on
/// body B and a force f applied to a point Bc of B.
/// The set of forces on body A from the bushing is equivalent to a torque -τ on
/// body A and a force -f applied to the point Ac of A.
/// Points Bc and Ac are coincident and located halfway between point Aʙₒ
/// (the origin of frame Aʙ) and point Bᴀₒ (the origin of frame Bᴀ).
///
/// The torque τ on B and force f applied to point Bc of B are expressed in
/// terms of orthogonal unit vectors Aʙx, Aʙy, Aʙz fixed in frame Aʙ as <pre>
/// τ = Tx Aʙx + Ty Aʙy + Tz Aʙz
/// f = Fx Aʙx + Fy Aʙy + Fz Aʙz
/// </pre>
///
/// Torque τ and force f depend on the orientation and position of frames Bᴀ and
/// Aʙ.  %Frame Bᴀ's orientation relative to frame Aʙ determines roll-pitch-yaw
/// angles q₀, q₁, q₂ [`-π < q₀ <= π`, `-π/2 <= q₁ <= π/2`, `-π < q₂ <= π`].
/// The position of Bᴀₒ (frame Bᴀ's origin) from Aʙₒ (frame Aʙ's origin) is
/// `x Aʙx + y Aʙy + z Aʙz`.
/// Note: When frame Bᴀ's angular velocity in Aʙ is `ωx Aʙx + ωy Aʙy + ωz Aʙz`,
/// then q̇₀, q̇₁, q̇₂ are related to ωx, ωy, ωz by a matrix N as <pre>
/// ⌈ q̇₀ ⌉ = ⌈ cos(q₂)/cos(q₁)  sin(q₂)/cos(q₁)   0 ⌉ ⌈ ωx ⌉
/// | q̇₁ | = |   -sin(q2)            cos(q2)      0 | | ωy |
/// ⌊ q̇₂ ⌋ = ⌊ cos(q₂)*tan(q₁)   sin(q₂)*tan(q₁)  1 ⌋ ⌊ ωz ⌋
/// </pre>
///
/// Specifically, the model of the force f and torque τ depends on <pre>
/// Fx = -(kx x + bx ẋ)
/// Fy = -(ky y + by ẏ)
/// Fz = -(kz z + bz ż)
/// T₀ = -(k₀ q₀ + b₀ q̇₀)
/// T₁ = -(k₁ q₁ + b₁ q̇₁)
/// T₂ = -(k₂ q₂ + b₂ q̇₂)
/// ⌈ Tx ⌉ = ⌈ cos(q₂)/cos(q₁)  -sin(q2)   cos(q₂)*tan(q₁) ⌉ ⌈ T₀ ⌉
/// | Ty | = | sin(q₂)/cos(q₁)   cos(q2)   sin(q₂)*tan(q₁) | | T₁ |
/// ⌊ Tz ⌋ = ⌊        0             0             1        ⌋ ⌊ T₂ ⌋
/// </pre>
/// where kx, ky, kz and bx, by, bz are force stiffness/damping constants and
/// k₀, k₁, k₂ and b₀, b₁, b₂ are torque stiffness/damping constants.  Note:
/// Tx, Ty, Tz relate to T₀, T₁, T₂, by Nᵀ (the transpose of the N matrix).
///
/// The expressions for Tx, Ty, Tz in terms of T₀, T₁, T₂ were found by equating
/// the generalized forces produced by torque `τ = Tx Aʙx + Ty Aʙy + Tz Aʙz` to
/// the generalized forces produced by the three spring-damper torques
///  `T₀ Bᴀx`,  `T₁ Py`,  `T₂ Aʙz`  (each of these torques are associated with
/// a frame in the RollPitchYaw successive rotation sequence, where `Py` denotes
/// a unit vector of the pitch intermediate frame).
///
/// The torque τ stiffness and a portion of the force f stiffness have a
/// potential energy U.  The torque τ damping and a portion of force f damping
/// have a dissipation function D as <pre>
/// U = 1/2 (k₀ q₀² + k₁ q₁² + k₂ q₂²) + 1/2 (kx x² + ky y² + kz z²)
/// D = 1/2 (b₀ q̇₀² + b₁ q̇₁² + b₂ q̇₂²) + 1/2 (bx ẋ² + by ẏ² + bz ż²)
/// </pre>
///
/// The combined power of the torque τ and force f can be written as <pre>
/// Power = -U̇ - 2*D + 0.5*[Fx*(y*wz-z*wy) + Fy*(z*wx-x*wz) + Fz*(x*wy - y*wx)]
/// </pre>
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see RollPitchYaw for definitions of roll, pitch, yaw = `[q₀, q₁, q₂]`.
// TODO(Mitiguy) Improve efficiency with caching and references.
template <typename T>
class LinearBushingRollPitchYaw final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearBushingRollPitchYaw)

  /// Constructor for a RollPitchYaw bushing that connects bodies `A` and `B`.
  /// @param[in] frameAb the frame Aʙ of body `A` that connects to the bushing.
  /// @param[in] frameBa the frame Bᴀ of body `B` that connects to the bushing.
  /// @param[in] torque_stiffness_constants  the constants `[k₀, k₁, k₂]`
  /// associated with the rotational part `1/2 (k₀q₀² + k₁q₁² + k₂q₂²)` of the
  /// potential energy U, where `[q₀, q₁, q₂]` are the roll, pitch, yaw angles.
  /// @param[in] torque_damping_constants the constants `[b₀, b₁, b₂]`
  /// associated with the rotational part `1/2 (b₀q̇₀² + b₁q̇₁² + b₂q̇₂²)`
  /// of the dissipation function D.
  /// @param[in] force_stiffness_constants the constants `[kx, ky, kz]`
  /// associated with the translational part `1/2 (kx x² + ky y² + kz z²)` of
  /// the potential energy U, where [x, y, z] are the bushing's displacements.
  /// @param[in] force_damping_constants the constants `[bx, by, bz]`
  /// associated with the translational part `1/2 (bx ẋ² + by ẏ² + bz ż²)`
  /// of the dissipation function D.
  /// @note Refer to the class documentation for details about U, D, q₀, etc.
  /// @note The stiffness constants `[k₀, k₁, k₂]`, `[kx, ky, kz]` and the
  /// damping constants `[b₀, b₁, b₂]`, `[bx, by, bz]` are usually non-negative.
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

  /// Returns the torque stiffness constants [k₀, k₁, k₂].
  const Vector3<double>& torque_stiffness_constants() const {
    return torque_stiffness_constants_;
  }

  /// Returns the torque damping constants [b₀, b₁, b₂].
  const Vector3<double>& torque_damping_constants() const {
    return torque_damping_constants_;
  }

  /// Returns the force stiffness constants [kx, ky, kz].
  const Vector3<double>& force_stiffness_constants() const {
    return force_stiffness_constants_;
  }

  /// Returns the force damping constants [bx, by, bz].
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
  /// (the position vector from Aʙₒ to Bᴀₒ expressed in Aʙ).
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
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return CalcBushingRollPitchYawAngleRates(context, rpy);
  }

  /// Calculates the bushing's displacement `p_AʙBᴀ_Aʙ = [x, y, z]` which is the
  /// position vector from point Aʙₒ to point Bᴀₒ, expressed in frame Aʙ.
  /// @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingDisplacement(
      const systems::Context<T>& context) const {
    return CalcBushingRigidTransform(context).translation();
  }

  /// Calculates the bushing's displacement rate `v_AʙBᴀₒ_Aʙ = [ẋ, ẏ, ż]`,
  /// which is point Bᴀₒ's velocity in frame Aʙ, expressed in frame Aʙ.
  /// @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingDisplacementRate(
      const systems::Context<T>& context) const {
    return CalcBushingSpatialVelocity(context).translational();
  }

  /// Returns V_AʙBᴀ, frame Bᴀ's spatial velocity in frame Aʙ expressed in Aʙ.
  /// @param[in] context The state of the multibody system.
  /// @note: V_AʙBᴀ.rotational() is w_AʙBᴀ_Aʙ (Bᴀ's angular velocity in Aʙ,
  /// expressed in Aʙ) whereas V_AʙBᴀ.translational() is v_AʙBᴀₒ_Aʙ = [ẋ, ẏ, ż]
  /// (point Bᴀₒ's translational velocity in frame Aʙ, expressed in frame Aʙ).
  /// Note: w_AʙBᴀ_Aʙ ≠ `[q̇₀, q̇₁, q̇₂]`.
  /// @see CalcBushingRollPitchYawAngleRates() for `[q̇₀, q̇₁, q̇₂]`.
  SpatialVelocity<T> CalcBushingSpatialVelocity(
      const systems::Context<T>& context) const {
    return frameBa().CalcSpatialVelocity(context, frameAb(), frameAb());
  }

  /// Calculates F_Bc_Ab, the spatial force on point Bc of body B expressed in
  /// frame Aʙ (due to the bushing forces exerted on B).
  /// @param[in] context The state of the multibody system.
  /// @note The set of forces on body B from the bushing is equivalent to a
  /// torque τ on body B and a force f applied to a point Bc of B.
  /// @related CalcBushingSpatialForceOnBa(), CalcBushingSpatialForceOnAb().
  SpatialForce<T> CalcBushingSpatialForceOnBc(
      const systems::Context<T>& context) const;

  /// Calculates F_Bᴀ_Ab, the spatial force on point Bᴀₒ of body B expressed in
  /// frame Aʙ (due to the bushing forces exerted on B).
  /// @param[in] context The state of the multibody system.
  /// @related CalcBushingSpatialForceOnBc(), CalcBushingSpatialForceOnAb().
  SpatialForce<T> CalcBushingSpatialForceOnBa(
      const systems::Context<T>& context) const {
    const SpatialForce<T> F_Bc_Ab = CalcBushingSpatialForceOnBc(context);
    const Vector3<T> p_BcBa_Ab = 0.5 * CalcBushingDisplacement(context);
    const SpatialForce<T> F_Ba_Ab = F_Bc_Ab.Shift(p_BcBa_Ab);
    return F_Ba_Ab;
  }

  /// Calculates F_Aʙ_Aʙ, the spatial force on point Aʙₒ of body A expressed in
  /// frame Aʙ (due to the bushing forces exerted on A).
  /// @param[in] context The state of the multibody system.
  /// @related CalcBushingSpatialForceOnBc(), CalcBushingSpatialForceOnBa().
  SpatialForce<T> CalcBushingSpatialForceOnAb(
      const systems::Context<T>& context) const {
    const SpatialForce<T> F_Bc_Ab = CalcBushingSpatialForceOnBc(context);
    const Vector3<T> p_BcAb_Ab = -0.5 * CalcBushingDisplacement(context);
    const SpatialForce<T> F_Ab_Ab = -(F_Bc_Ab).Shift(p_BcAb_Ab);
    return F_Ab_Ab;
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

  // Returns [k₀q₀, k₁q₁, k₂q₂], element-wise multiplication of the torque
  // stiffness constants [k₀, k₁, k₂] and roll-pitch-yaw angles [q₀, q₁, q₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> TorqueStiffnessConstantsTimesAngles(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return torque_stiffness_constants().cwiseProduct(rpy.vector());
  }

  // Returns [b₀q̇₀, b₁q̇₁, b₂q̇₂], element-wise multiplication of the torque
  // damping constants [b₀, b₁, b₂] and roll-pitch-yaw rates [q̇₀, q̇₁, q̇₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> TorqueDampingConstantsTimesAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context, rpy);
    return torque_damping_constants().cwiseProduct(rpyDt);
  }

  // Returns [kx x, ky y, kz z], element-wise multiplication of the force
  // stiffness constants [kx, ky, kz] and displacements [x, y, z].
  // @param[in] context The state of the multibody system.
  Vector3<T> ForceStiffnessConstantsTimesDisplacement(
      const systems::Context<T>& context) const {
    const Vector3<T> xyz = CalcBushingDisplacement(context);
    return force_stiffness_constants().cwiseProduct(xyz);
  }

  // Returns [bx ẋ, by ẏ, bz ż], element-wise multiplication of the force
  // damping constants [bx, by, bz] and displacement rates [ẋ, ẏ, ż].
  // @param[in] context The state of the multibody system.
  Vector3<T> ForceDampingConstantsTimesDisplacementRate(
      const systems::Context<T>& context) const {
    const Vector3<T> xyzDt = CalcBushingDisplacementRate(context);
    return force_damping_constants().cwiseProduct(xyzDt);
  }

  // Calculates the resultant force f exerted by the bushing on body B.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingResultantForceOnB(
      const systems::Context<T>& context) const {
    // Calculate force `f = Fx Aʙx + Fy Aʙy + Fz Aʙz`.
    // Fx = -(kx x + bx ẋ)
    // Fy = -(ky y + by ẏ)
    // Fz = -(kz z + bz ż)
    return -(ForceStiffnessConstantsTimesDisplacement(context) +
             ForceDampingConstantsTimesDisplacementRate(context));
  }

  // Calculate the matrix that relates q̇₀, q̇₁, q̇₂ to ωx, ωy, ωz, where frame
  // Bᴀ's angular velocity in Aʙ is `ωx Aʙx + ωy Aʙy + ωz Aʙz`.
  // ⌈ q̇₀ ⌉ = ⌈ cos(q₂)/cos(q₁)  sin(q₂)/cos(q₁)   0 ⌉ ⌈ ωx ⌉
  // | q̇₁ | = |   -sin(q2)            cos(q2)      0 | | ωy |
  // ⌊ q̇₂ ⌋ = ⌊ cos(q₂)*tan(q₁)   sin(q₂)*tan(q₁)  1 ⌋ ⌊ ωz ⌋
  Matrix3<T> CalcQDt012ToWxyzMatrix(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const T q1 = rpy.pitch_angle();
    const T q2 = rpy.yaw_angle();
    const T c1 = cos(q1), s1 = sin(q1), oneOverc1 = 1/c1, tan1 = s1 * oneOverc1;
    const T c2 = cos(q2), s2 = sin(q2);
    Matrix3<T> mat33;
    // clang-format off
    mat33 << c2 * oneOverc1,  s2 * oneOverc1,  0,
                        -s2,              c2,  0,
                  c2 * tan1,       s2 * tan1,  1;
    // clang-format on
    return mat33;
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
