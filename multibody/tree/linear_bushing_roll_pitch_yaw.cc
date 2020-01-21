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
                          const Frame<T>& frameA,
                          const Frame<T>& frameC,
                          const Vector3<double>& torque_stiffness_constants,
                          const Vector3<double>& torque_damping_constants,
                          const Vector3<double>& force_stiffness_constants,
                          const Vector3<double>& force_damping_constants) :
    ForceElement<T>(frameA.body().model_instance()),
    frameA_(frameA),
    frameC_(frameC),
    torque_stiffness_constants_(torque_stiffness_constants),
    torque_damping_constants_(torque_damping_constants),
    force_stiffness_constants_(force_stiffness_constants),
    force_damping_constants_(force_damping_constants) {
}

template <typename T>
void LinearBushingRollPitchYaw<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */,
    MultibodyForces<T>* forces) const {

  // Form spatial force F_Abo_Ab which contains both the moment of all forces
  // exerted by the bushing on frame Aʙ about Aʙₒ and the net force exerted by
  // the bushing on frame Aʙ (the moment and force are both expressed in Aʙ).
  const SpatialForce<T> F_Abo_Ab = CalcBushingSpatialForceOnAb(context);

  // Form spatial force F_Abo_W by expressing F_Ab_Ab in the world frame W.
  const RigidTransform<T> X_WAb = frameA().CalcPoseInWorld(context);
  const RotationMatrix<T>& R_WAb = X_WAb.rotation();
  const SpatialForce<T> F_Abo_W = R_WAb * F_Abo_Ab;

  // The next calculation needs the position from Aʙₒ to Ao expressed in world.
  const RigidTransform<T> X_WA = bodyA().EvalPoseInWorld(context);
  const RotationMatrix<T>& R_WA = X_WA.rotation();
  const Vector3<T> p_AoAbo_A =
      frameA().CalcPoseInBodyFrame(context).translation();
  const Vector3<T> p_AboAo_W = -(R_WA * p_AoAbo_A);

  // Form the spatial force F_Ao_W by shifting the spatial force F_Abo_W from
  // Aʙₒ (Aʙ's origin) to Ao (body A's origin).
  const SpatialForce<T> F_Ao_W = F_Abo_W.Shift(p_AboAo_W);

  // The next calculation needs the position from Ao to Bo expressed in world.
  const Vector3<T>& p_WoAo_W = X_WA.translation();
  const Vector3<T> p_WoBo_W = bodyB().EvalPoseInWorld(context).translation();
  const Vector3<T> p_AoBo_W = p_WoBo_W - p_WoAo_W;

  // Form the spatial force F_Bo_W by shifting the spatial force F_Ao_W from
  // Aₒ (body A's origin) to Bₒ (body B's origin).
  const SpatialForce<T> F_Bo_W = F_Ao_W.Shift(p_AoBo_W);

  // Alias to the array of spatial forces applied to each body.
  std::vector<SpatialForce<T>>& F_BodyOrigin_W_array =
      forces->mutable_body_forces();

  // Action:   Apply torque Tᴀ to body A and force  f to Ao (body A's origin).
  // Reaction: Apply torque Tʙ to body B and force -f to Bo (body B's origin).
  F_BodyOrigin_W_array[bodyA().node_index()] += F_Ao_W;
  F_BodyOrigin_W_array[bodyB().node_index()] -= F_Bo_W;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */) const {
  // Use the torque stiffness constants [k₀, k₁, k₂] and roll-pitch-yaw angles
  // [q₀, q₁, q₂] to form torque potential energy 0.5*[k₀*q₀², k₁*q₁², k₂*q₂²].
  const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
  const Vector3<T>& q012 = rpy.vector();
  const Vector3<T> q012Squared = q012.cwiseProduct(q012);
  const T torque_potential_energy =
      0.5 * (torque_stiffness_constants().dot(q012Squared));

  // Use the force stiffness constants [kx, ky, kz] and `p_AʙBᴀ = [x, y, z]`
  // (the position vector from point Aʙₒ to point Bᴀₒ expressed in frame Aʙ)
  // to form force potential energy 0.5 * [kx*x², ky*y², kz*z²].
  const Vector3<T> xyz = CalcBushingRigidTransform(context).translation();
  const Vector3<T> xyzSquared = xyz.cwiseProduct(xyz);
  const T force_potential_energy =
      0.5 * (force_stiffness_constants().dot(xyzSquared));

  return torque_potential_energy + force_potential_energy;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // One way to calculate conservative power Pc is from potential energy V.
  // V = 1/2 k₀ q₀² + 1/2 k₁ q₁² + 1/2 k₂ q₂²
  //   + 1/2 kx x²  + 1/2 ky y²  + 1/2 kz z²
  // Pc = -dV/dt = -(k₀ q₀ q̇₀ + k₁ q₁ q̇₁ + k₂ q₂ q̇₂
  //               + kx x ẋ   + ky y ẏ   + kz z ż)
  // Calculate Pc_torque = -(k₀ q₀ q̇₀ + k₁ q₁ q̇₁ + k₂ q₂ q̇₂).
  const Vector3<T> KQ = TorqueStiffnessConstantsTimesAngles(context);
  const Vector3<T> q012Dt = CalcBushingRollPitchYawAngleRates(context);
  const T Pc_torque = -(KQ.dot(q012Dt));

  // Calculate Pc_force = -(kx x ẋ + ky y ẏ + kz z ż).
  const Vector3<T> KX = ForceStiffnessConstantsTimesDisplacement(context);
  const Vector3<T> xyzDt = CalcBushingDisplacementRate(context);
  const T Pc_force = -(KX.dot(xyzDt));

  return Pc_torque + Pc_force;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcNonConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // Calculate the part of nonconservative power due to torque damping.
  // Pn_torque = -(k₀ q̇₀ q̇₀ + k₁ q̇₁ q̇₁ + k₂ q̇₂ q̇₂)
  const Vector3<T> BQDt = TorqueDampingConstantsTimesAngleRates(context);
  const Vector3<T> q012Dt = CalcBushingRollPitchYawAngleRates(context);
  const T Pn_torque = -(BQDt.dot(q012Dt));

  // Calculate the part of nonconservative power due to force damping.
  // Pn_force = -(kx ẋ ẋ  + ky ẏ ẏ  + kz ż ż)
  const Vector3<T> BXDt = ForceDampingConstantsTimesDisplacementRate(context);
  const Vector3<T> xyzDt = CalcBushingDisplacementRate(context);
  const T Pn_force = -(BXDt.dot(xyzDt));

  // Calculate the part of nonconservative power that is due to additional terms
  // in the calculation of power.
  // Extra = -0.5*kx*x*(y*wz-z*wy) - 0.5*ky*y*(z*wx-x*wz) - 0.5*kz*z*(x*wy-y*wx)
  //       -  0.5*bx*ẋ*(y*wz-z*wy) - 0.5*by*ẏ*(z*wx-x*wz) - 0.5*bz*ż*(x*wy-y*wx)
  const Vector3<T> xyz = 0.5 * CalcBushingRigidTransform(context).translation();
  const Vector3<T> w_AbBa = CalcBushingSpatialVelocity(context).rotational();
  const Vector3<T> w_cross_xyz = w_AbBa.cross(xyz);
  const Vector3<T> force = CalcBushingResultantForceOnB(context);
  const T power_extra = force.dot(w_cross_xyz);

  return Pn_torque + Pn_force + power_extra;
}

template <typename T>
SpatialForce<T> LinearBushingRollPitchYaw<T>::CalcBushingSpatialForceOnBc(
    const systems::Context<T>& context) const {
  // The set of forces applied by the bushing to body B are replaced by the
  // set's resultant force f applied to point Bc of B together with a torque τ
  // equal to the moment of the set about point Bc.

  // Calculate force `f = Fx Aʙx + Fy Aʙy + Fz Aʙz` applied to point Bc of B.
  const Vector3<T> f = CalcBushingResultantForceOnB(context);

  // Intermediate calculates in preparation for torque τ.
  // T₀ = -(k₀ q₀ + b₀ q̇₀)
  // T₁ = -(k₁ q₁ + b₁ q̇₁)
  // T₂ = -(k₂ q₂ + b₂ q̇₂)
  const Vector3<T> t012 = -(TorqueStiffnessConstantsTimesAngles(context) +
                            TorqueDampingConstantsTimesAngleRates(context));

  // Calculate torque `τ = Tx Aʙx + Ty Aʙy + Tz Aʙz` applied to body B.
  // Tx = cos(q₂)/cos(q₁) T₀ - sin(q2) T₁ + cos(q₂)*tan(q₁) T₂
  // Ty = sin(q₂)/cos(q₁) T₀ + cos(q2) T₁ + sin(q₂)*tan(q₁) T₂
  // Tz =                                                   T₂
  const Matrix3<T> mat33 = CalcQDt012ToWxyzMatrix(context);
  const Vector3<T> Txyz = mat33.transpose() * t012;

  return SpatialForce<T>(Txyz, f);
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearBushingRollPitchYaw<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frameA_clone = tree_clone.get_variant(frameA());
  const Frame<ToScalar>& frameC_clone = tree_clone.get_variant(frameC());
  const Vector3<double>& k012 = torque_stiffness_constants();
  const Vector3<double>& b012 = torque_damping_constants();
  const Vector3<double>& kxyz = force_stiffness_constants();
  const Vector3<double>& bxyz = force_damping_constants();

  // Make the Joint<T> clone.
  auto bushing_clone =
      std::make_unique<LinearBushingRollPitchYaw<ToScalar>>(
          frameA_clone, frameC_clone, k012, b012, kxyz, bxyz);

  return std::move(bushing_clone);
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

