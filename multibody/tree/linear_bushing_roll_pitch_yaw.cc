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
                          const Frame<T>& frameAb,
                          const Frame<T>& frameBa,
                          const Vector3<double>& torque_stiffness_constants,
                          const Vector3<double>& torque_damping_constants,
                          const Vector3<double>& force_stiffness_constants,
                          const Vector3<double>& force_damping_constants) :
    ForceElement<T>(frameAb.body().model_instance()),
    frameAb_(frameAb),
    frameBa_(frameBa),
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
  // Form the bushing's resultant torque on frame Aʙ , expressed in frame Aʙ.
  const Vector3<T> t_Ab_Ab = CalcBushingTorqueStiffnessOnAb(context)
                           + CalcBushingTorqueDampingOnAb(context);

  // Form the bushing's resultant force on point Aʙₒ, expressed in frame Aʙ.
  const Vector3<T> f_Abo_Ab = CalcBushingForceStiffnessOnAbo(context)
                            + CalcBushingForceDampingOnAbo(context);

  // Re-express the bushing's resultant torque in the world frame W.
  // Re-express the bushing's resultant force in the world frame W.
  const RigidTransform<T> X_WAb = frameAb().CalcPoseInWorld(context);
  const RotationMatrix<T>& R_WAb = X_WAb.rotation();
  const Vector3<T> t_Ab_W = R_WAb * t_Ab_Ab;
  const Vector3<T> f_Abo_W = R_WAb * f_Abo_Ab;

  // The next calculation needs the position from Aʙₒ to Ao, expressed in world.
  const Vector3<T> p_AoAbo_A =
      frameAb().CalcPoseInBodyFrame(context).translation();
  const RotationMatrix<T>& R_WA = bodyA().EvalPoseInWorld(context).rotation();
  const Vector3<T> p_AboAo_W = -(R_WA * p_AoAbo_A);

  // Shift the force from Aʙₒ (frame Aʙ's origin) to Ao (body A's origin).
  const SpatialForce<T> F_Ao_W =
      SpatialForce<T>(t_Ab_W, f_Abo_W).Shift(p_AboAo_W);

  // The next calculation needs the position from Ao to Bo, expressed in world.
  const Vector3<T>& p_WoAo_W = bodyA().EvalPoseInWorld(context).translation();
  const Vector3<T>& p_WoBo_W = bodyB().EvalPoseInWorld(context).translation();
  const Vector3<T> p_AoBo_W = p_WoBo_W - p_WoAo_W;

  // Shift the force from Aₒ (body A's origin) to Bₒ (body B's origin).
  const SpatialForce<T> F_Bo_W = F_Ao_W.Shift(p_AoBo_W);

  // Alias to the array of spatial forces applied to each body.
  std::vector<SpatialForce<T>>& F_BodyOrigin_W_array =
      forces->mutable_body_forces();

  // Action:   Apply torque to body A and a force to Ao (body A's origin).
  // Reaction: Apply torque to body B and a force to Bo (body B's origin).
  F_BodyOrigin_W_array[bodyA().node_index()] += F_Ao_W;
  F_BodyOrigin_W_array[bodyB().node_index()] -= F_Bo_W;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */) const {
  // Calculate the roll-pitch-yaw angles q₀, q₁, q₂ and use them together with
  // torsional stiffness to form the torque potential energy.
  const Vector3<T> ypr = CalcBushingBodyZYXAngleSequenceYawRollPitch(context);
  const Vector3<T> q012Squared = ypr.cwiseProduct(ypr);
  const T torque_potential_energy =
      0.5 * torque_stiffness_constants().dot(q012Squared);

  // Calculate p_AʙBᴀ (the position from point Aʙₒ to point Bᴀₒ, expressed in
  // frame Aʙ) and use it and stiffness to form the force potential energy.
  const Vector3<T> p_AbBa = CalcBushingPositionVector(context);
  const Vector3<T> xyzSquared = p_AbBa.cwiseProduct(p_AbBa);
  const T force_potential_energy =
      0.5 * force_stiffness_constants().dot(xyzSquared);

  return torque_potential_energy + force_potential_energy;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // One way to calculate conservative power Pc is from potential energy V,
  // V = 1/2⋅k₀⋅q₀² + 1/2⋅k₁⋅q₁²  + 1/2⋅k₂⋅q₂²
  //   + 1/2⋅kx⋅x²  + 1/2⋅ky⋅y² + 1/2⋅kz⋅z²
  // Pc = -dV/dt = k₀⋅q₀⋅q̇₀ + k₁⋅q₁⋅q̇₁ + k₂⋅q₂⋅q̇₂
  //             + kx⋅x⋅ẋ   + ky⋅y⋅ẏ   + kz⋅z⋅ż
  const Vector3<T> torque_stiffness = CalcBushingTorqueDampingOnAb(context);
  const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context);
  const T Pc_torque = torque_stiffness.dot(rpyDt);

  const Vector3<T> force_stiffness = CalcBushingForceDampingOnAbo(context);
  const Vector3<T> xyzDt = CalcBushingTranslationalVelocity(context);
  const T Pc_force = force_stiffness.dot(xyzDt);

  return Pc_torque + Pc_force;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcNonConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // Nonconservative power Pn comes from damping.
  // Pn = k₀⋅q̇₀⋅q̇₀ + k₁⋅q̇₁⋅q̇₁ + k₂⋅q̇₂⋅q̇₂
  //    + kx⋅ẋ⋅ẋ   + ky⋅ẏ⋅ẏ   + kz⋅ż⋅ż
  const Vector3<T> torque_damping = CalcBushingTorqueDampingOnAb(context);
  const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context);
  const T Pn_torque = torque_damping.dot(rpyDt);

  const Vector3<T> force_damping = CalcBushingForceDampingOnAbo(context);
  const Vector3<T> xyzDt = CalcBushingTranslationalVelocity(context);
  const T Pn_force = force_damping.dot(xyzDt);

  return Pn_torque + Pn_force;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearBushingRollPitchYaw<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frameAb_clone = tree_clone.get_variant(frameAb());
  const Frame<ToScalar>& frameBa_clone = tree_clone.get_variant(frameBa());
  const Vector3<double>& k012 = torque_stiffness_constants();
  const Vector3<double>& b012 = torque_damping_constants();
  const Vector3<double>& kxyz = force_stiffness_constants();
  const Vector3<double>& bxyz = force_damping_constants();

  // Make the Joint<T> clone.
  auto bushing_clone =
      std::make_unique<LinearBushingRollPitchYaw<ToScalar>>(
          frameAb_clone, frameBa_clone, k012, b012, kxyz, bxyz);

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

