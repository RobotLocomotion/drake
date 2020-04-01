#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

using math::RigidTransform;
using math::RotationMatrix;

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
    const Frame<T>& frameA, const Frame<T>& frameC,
    const Vector3<double>& torque_stiffness_constants,
    const Vector3<double>& torque_damping_constants,
    const Vector3<double>& force_stiffness_constants,
    const Vector3<double>& force_damping_constants)
    : LinearBushingRollPitchYaw(frameA.model_instance(),
                                frameA.index(), frameC.index(),
                                torque_stiffness_constants,
                                torque_damping_constants,
                                force_stiffness_constants,
                                force_damping_constants) {
  DRAKE_THROW_UNLESS(frameA.model_instance() == world_model_instance() ||
                     frameC.model_instance() == world_model_instance() ||
                     frameA.model_instance() == frameC.model_instance());
}

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
    ModelInstanceIndex model_instance,
    FrameIndex frameA_index, FrameIndex frameC_index,
    const Vector3<double>& torque_stiffness_constants,
    const Vector3<double>& torque_damping_constants,
    const Vector3<double>& force_stiffness_constants,
    const Vector3<double>& force_damping_constants)
    : ForceElement<T>(model_instance),
      frameA_index_(frameA_index),
      frameC_index_(frameC_index),
      torque_stiffness_constants_(torque_stiffness_constants),
      torque_damping_constants_(torque_damping_constants),
      force_stiffness_constants_(force_stiffness_constants),
      force_damping_constants_(force_damping_constants) {
  DRAKE_THROW_UNLESS(torque_stiffness_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(torque_damping_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(force_stiffness_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(force_damping_constants.minCoeff() >= 0);
}

template <typename T>
void LinearBushingRollPitchYaw<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */,
    MultibodyForces<T>* forces) const {

  // Form F_Ao_A, the spatial force at point Ao of frame A due to the bushing.
  const SpatialForce<T> F_Ao_A = CalcBushingSpatialForceOnFrameA(context);

  // Form F_Ao_W by expressing F_Ao_A in the world frame W.
  const RotationMatrix<T> R_WA = frameA().CalcRotationMatrixInWorld(context);
  const SpatialForce<T> F_Ao_W = R_WA * F_Ao_A;

  // The next calculation needs the position from Aₒ (frame A's origin) to L0ₒ
  // (the origin of link L0), expressed in the world frame W.
  const RigidTransform<T>& X_WL0 = link0().EvalPoseInWorld(context);
  const RotationMatrix<T>& R_WL0 = X_WL0.rotation();
  const Vector3<T> p_L0Ao_L0 =
      frameA().CalcPoseInBodyFrame(context).translation();
  const Vector3<T> p_AoL0_W = -(R_WL0 * p_L0Ao_L0);

  // Form the spatial force F_L0_W by shifting the spatial force F_Ao_W from
  // Aₒ (A's origin) to L0ₒ (link L0's origin).
  const SpatialForce<T> F_L0_W = F_Ao_W.Shift(p_AoL0_W);

  // The next calculation needs the position from L0 to L1 expressed in world.
  const Vector3<T>& p_WoL0_W = X_WL0.translation();
  const Vector3<T>& p_WoL1_W = link1().EvalPoseInWorld(context).translation();
  const Vector3<T> p_L0L1_W = p_WoL1_W - p_WoL0_W;

  // Form the spatial force F_L1_W by shifting the spatial force F_L0_W from L0ₒ
  // (link L0's origin) to L1ₒ (link L1's origin) and negating the result.
  const SpatialForce<T> F_L1_W = -(F_L0_W.Shift(p_L0L1_W));

  // Alias to the array of spatial forces applied to each link (body).
  std::vector<SpatialForce<T>>& F_BodyOrigin_W_array =
      forces->mutable_body_forces();

  // Apply a torque to link L0 and apply the force −𝐟 to L0ₒ.
  // Apply a torque to link L1 and apply the force +𝐟 to L1ₒ.
  F_BodyOrigin_W_array[link0().node_index()] += F_L0_W;
  F_BodyOrigin_W_array[link1().node_index()] += F_L1_W;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */) const {
  // Note: The LinearBushingRollPitchYaw class documentation describes the
  // calculation of power (conservative/nonconservative) and potential energy.
  // TODO(Mitiguy) Per issue #12752, update ForceElement class to improve return
  //  values and LinearBushingRollPitchYaw class documentation.
  // ----------------------------------------------------------------------
  // Use the torque stiffness constants [k₀ k₁ k₂] and roll-pitch-yaw angles
  // [q₀ q₁ q₂] to form torque potential energy 0.5*(k₀*q₀² + k₁*q₁² + k₂*q₂²).
  const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
  const Vector3<T>& q012 = rpy.vector();
  const Vector3<T> q012Squared = q012.cwiseProduct(q012);
  const T torque_potential_energy =
      0.5 * (torque_stiffness_constants().dot(q012Squared));

  // Use the force stiffness constants [kx, ky, kz] and `p_AₒCₒ_B = [x y z]ʙ`
  // (the position vector from Aₒ to Bₒ expressed in frame B) to form the
  // analytical part of the force potential energy 0.5*(kx*x² + ky*y² + kz*z²).
  const Vector3<T> xyz = Calcp_AoCo_B(context);  // [x y z]ʙ
  const Vector3<T> xyzSquared = xyz.cwiseProduct(xyz);
  const T force_potential_energy =
      0.5 * (force_stiffness_constants().dot(xyzSquared));

  return torque_potential_energy + force_potential_energy;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePowerAnalytical(
    const systems::Context<T>& context) const {
  // ----------------------------------------------------------------------
  // Analytical conservative power Pcᴀ is related to analytical potential energy
  // Uᴀ by `Pcᴀ = −U̇ᴀ`.
  // Uᴀ  = 1/2 (k₀ q₀² + k₁ q₁² + k₂ q₂²  +  kx x² + ky y² + kz z²)
  // Pcᴀ = −(k₀ q₀ q̇₀ + k₁ q₁ q̇₁ + k₂ q₂ q̇₂  +  kx x ẋ + ky y ẏ + kz z ż)
  // ----------------------------------------------------------------------
  // Calculate Pcᴀ_torque = τᴋ ⋅ q̇ = −(k₀ q₀ q̇₀ + k₁ q₁ q̇₁ + k₂ q₂ q̇₂).
  const Vector3<T> kQ = TorqueStiffnessConstantsTimesAngles(context);
  const Vector3<T> q012Dt = CalcBushingRollPitchYawAngleRates(context);
  const T PcA_torque = -(kQ.dot(q012Dt));

  // Calculate Pcᴀ_force = 𝐟ᴋ ⋅ Ẋ = −(kx x ẋ + ky y ẏ + kz z ż).
  const Vector3<T> kX = ForceStiffnessConstantsTimesDisplacement(context);
  const Vector3<T> xyzDt = CalcBushing_xyzDt(context);
  const T PcA_force = -(kX.dot(xyzDt));

  return PcA_torque + PcA_force;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePowerNumerical(
    const systems::Context<T>& context) const {
  // Calculate the part of conservative power due to Pcɪ = w_AC ⋅ (p_AoCo × 𝐟ᴋ),
  // where Pcɪ is the part of conservative power that is numerically integrated
  // to calculate Uɪ (the non-analytical part of potential energy).
  const Vector3<T> spring_force_B =
      -ForceStiffnessConstantsTimesDisplacement(context);
  const T PcI = CalcPowerHelperMethod(context, spring_force_B);
  return PcI;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // Note: The LinearBushingRollPitchYaw class documentation describes the
  // calculation of power (conservative/nonconservative) and potential energy.
  // TODO(Mitiguy) Per issue #12752, update ForceElement class to improve return
  //  values and LinearBushingRollPitchYaw class documentation.
  return CalcConservativePowerAnalytical(context);
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcNonConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // Note: The LinearBushingRollPitchYaw class documentation describes the
  // calculation of power (conservative/nonconservative) and potential energy.
  // TODO(Mitiguy) Per issue #12752, update ForceElement class to improve return
  //  values and LinearBushingRollPitchYaw class documentation.
  // ----------------------------------------------------------------------
  // Calculate the part of nonconservative power due to torque damping.
  // Pɴᴄ_torque = τᴅ ⋅ q̇ = −(k₀ q̇₀ q̇₀ + k₁ q̇₁ q̇₁ + k₂ q̇₂ q̇₂)
  const Vector3<T> bQDt = TorqueDampingConstantsTimesAngleRates(context);
  const Vector3<T> q012Dt = CalcBushingRollPitchYawAngleRates(context);
  const T Pnc_torque = -(bQDt.dot(q012Dt));

  // Calculate a part of nonconservative power that is due to force damping.
  // Pɴᴄ_force = 𝐟ᴅ ⋅ X = −(kx ẋ ẋ  + ky ẏ ẏ  + kz ż ż)
  const Vector3<T> bXDt = ForceDampingConstantsTimesDisplacementRate(context);
  const Vector3<T> xyzDt = CalcBushing_xyzDt(context);
  const T Pnc_force0 = -(bXDt.dot(xyzDt));

  // Calculate the part of nonconservative power due to w_AC ⋅ (p_AoCo × 𝐟ᴅ).
  const Vector3<T> damping_force_B =
      -ForceDampingConstantsTimesDisplacementRate(context);
  const T Pnc_force_extra = CalcPowerHelperMethod(context, damping_force_B);

  // Calculate the part of conservative power due to Pcɪ = w_AC ⋅ (p_AoCo × 𝐟ᴋ),
  const T PcI = CalcConservativePowerNumerical(context);

  return Pnc_torque + Pnc_force0 + Pnc_force_extra + PcI;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPowerHelperMethod(
    const systems::Context<T>& context, const Vector3<T>& force_B) const {
  // Helper method to calculate a part of power due to w_AC ⋅ (p_AoCo × 𝐟ᴅ).
  // @param[in] context The state of the multibody system.
  const Vector3<T> p_AoCo_B = Calcp_AoCo_B(context);
  const Vector3<T> pCrossf_B = p_AoCo_B.cross(force_B);
  const Vector3<T> w_AC_A = Calcw_AC_A(context);
  const math::RotationMatrix<T> R_BA = CalcR_AB(context).inverse();
  const Vector3<T> w_AC_B = R_BA * w_AC_A;
  const T power_force_extra = w_AC_B.dot(pCrossf_B);
  return power_force_extra;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearBushingRollPitchYaw<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& ) const {
  // const Frame<ToScalar>& frameA_clone = tree_clone.get_variant(frameA());
  // const Frame<ToScalar>& frameC_clone = tree_clone.get_variant(frameC());
  const Vector3<double>& k012 = torque_stiffness_constants();
  const Vector3<double>& d012 = torque_damping_constants();
  const Vector3<double>& kxyz = force_stiffness_constants();
  const Vector3<double>& dxyz = force_damping_constants();

  // TODO(mitiguy) Why my kludge for the constructor used below to be public?
  std::unique_ptr<LinearBushingRollPitchYaw<ToScalar>> bushing_clone(
      new LinearBushingRollPitchYaw<ToScalar>(this->model_instance(),
          frameA_index_, frameC_index_,
          k012, d012, kxyz, dxyz));

  return bushing_clone;
  //  TODO ask Alejandro/Sherm if need return std::move(bushing_clone).
  //   std::move(door_hing_clone) was not used in door_hinge.cc
  //   However, return std::move(joint_clone) was used in revolute_joine.cc
}

template <typename T>
std::unique_ptr<ForceElement<double>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)

