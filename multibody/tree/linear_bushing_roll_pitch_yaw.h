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
/// force due to stiffness (spring) and dissipation (damper) properties.
///
/// The set of forces on body B from the bushing is equivalent to a torque τ on
/// body B and a force f applied to a point Bᴀₒ of B (Bᴀₒ is frame Bᴀ's origin).
/// The set of forces on body A from the bushing is equivalent to a torque -τ on
/// body A and a force -f applied to the point of A coincident with Bᴀₒ.
///
/// The torque τ on B and force f applied to point Bᴀₒ of B are expressed in
/// terms of orthogonal unit vectors Aʙx, Aʙy, Aʙz fixed in frame Aʙ as <pre>
/// τ = Tx Aʙx + Ty Aʙy + Tz Aʙz
/// f = Fx Aʙx + Fy Aʙy + Fz Aʙz
/// </pre>
///
/// The bushing's potential energy 𝖀 is modeled as a function of the bushing's
/// roll-pitch-yaw angles q₀, q₁, q₂ and the bushing's displacements x, y, z.
/// Angles q₀, q₁, q₂ are determined from the orientation between frames Aʙ and
/// Bᴀ and have the range `-π < q₀ <= π`, `-π/2 <= q₁ <= π/2`, `-π < q₂ <= π`.
/// Displacements x, y, z are determined from the position vector from Aʙₒ
/// to Bᴀₒ expressed in frame Aʙ.  The bushing's dissipation function 𝕱 depends
/// on q̇₀, q̇₁, q̇₂ and ẋ, ẏ, ż (time-derivatives of q₀, q₁, q₂ and x, y, z).<pre>
/// `𝖀 = 1/2 (k₀ q₀² + k₁ q₁² + k₂ q₂²) + 1/2 (kx x² + ky y² + kz z²)`
/// `𝕱 = 1/2 (b₀ q̇₀² + b₁ q̇₁² + b₂ q̇₂²) + 1/2 (bx ẋ² + by ẏ² + bz ż²)`
/// </pre>
/// @note k₀, k₁, k₂ and b₀, b₁, b₂ are torque stiffness and damping constants
/// and kx, ky, kz and bx, by, bz are force stiffness and damping constants.
///
/// By equating the generalized forces produced by 𝖀 and 𝕱 to the generalized
/// forces produced by τ and f, one can show [Mitiguy 2016, §26.9] <pre>
/// Fx = -(kx x + bx ẋ)
/// Fy = -(ky y + by ẏ)
/// Fz = -(kz z + bz ż)
/// T₀ = -(k₀ q₀ + b₀ q̇₀)
/// T₁ = -(k₁ q₁ + b₁ q̇₁)
/// T₂ = -(k₂ q₂ + b₂ q̇₂)
/// Tx = cos(q₂)/cos(q₁) T₀ - sin(q2) T₁ + cos(q₂) tan(q₁) T₂
/// Ty = sin(q₂)/cos(q₁) T₀ + cos(q2) T₁ + sin(q₂) tan(q₁) T₂
/// Tz =                                                   T₂
/// </pre>
///
/// [Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see @ref RollPitchYaw for definitions of roll, pitch, yaw = `[q₀, q₁, q₂]`.
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
  /// potential energy 𝖀 where `[q₀, q₁, q₂]` are the roll, pitch, yaw angles.
  /// @param[in] torque_damping_constants the constants `[b₀, b₁, b₂]`
  /// associated with the rotational part `1/2 (b₀q̇₀² + b₁q̇₁² + b₂q̇₂²)`
  /// of the dissipation function 𝕱.
  /// @param[in] force_stiffness_constants the constants `[kx, ky, kz]`
  /// associated with the translational part `1/2 (kx x² + ky y² + kz z²)` of
  /// the potential energy 𝖀, where [x, y, z] are the bushing's displacements.
  /// @param[in] force_damping_constants the constants `[bx, by, bz]`
  /// associated with the translational part `1/2 (bx ẋ² + by ẏ² + bz ż²)`
  /// of the dissipation function 𝕱.
  /// @note Refer to the class documentation for details about 𝖀, 𝕱, q₀, etc.
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
  /// (the position vector from Aʙₒ to Bᴀₒ, expressed in Aʙ).
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

  /// Calculates F_Aʙ_Aʙ, the spatial force on point Aʙₒ of body A expressed in
  /// frame Aʙ (due to the bushing forces exerted on A).
  /// @param[in] context The state of the multibody system.
  SpatialForce<T> CalcBushingSpatialForceOnAb(
      const systems::Context<T>& context) const {
    const SpatialForce<T> F_Ba_Ab = CalcBushingSpatialForceOnBa(context);
    const Vector3<T> p_BaAb_Ab = -CalcBushingDisplacement(context);
    const SpatialForce<T> F_Ab_Ab = -(F_Ba_Ab).Shift(p_BaAb_Ab);
    return F_Ab_Ab;
  }

  // Calculates F_Bᴀₒ_Ab, the spatial force on point Bᴀₒ of body B expressed in
  // frame Aʙ, equivalent to the set of forces exerted by the bushing on B.
  // @param[in] context The state of the multibody system.
  // @note F_Bᴀₒ_Ab is equivalent to a torque τ applied to body B and a force f
  // applied to point Bᴀₒ of B (Bᴀₒ is the origin of frame Bᴀ).
  SpatialForce<T> CalcBushingSpatialForceOnBa(
      const systems::Context<T>& context) const;

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
