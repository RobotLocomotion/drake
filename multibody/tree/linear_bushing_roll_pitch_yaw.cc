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
                          const Vector3<T>& torque_stiffness_constants,
                          const Vector3<T>& torque_damping_constants,
                          const Vector3<T>& force_stiffness_constants,
                          const Vector3<T>& force_damping_constants) :
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
  const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
  const Vector3<T> q012Squared = rpy.vector().cwiseProduct(rpy.vector());
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

#if 0
template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearBushingRollPitchYaw<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Body<ToScalar>& bodyA_clone =
      tree_clone.get_body(bodyA().index());
  const Body<ToScalar>& bodyB_clone =
      tree_clone.get_body(bodyB().index());

  // Make the Joint<T> clone.
  auto spring_damper_clone =
      std::make_unique<LinearBushingRollPitchYaw<ToScalar>>(
          bodyA_clone, p_AP(), bodyB_clone, p_BQ(), free_length(), stiffness(),
          damping());

  return std::move(spring_damper_clone);
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

template <typename T>
T LinearBushingRollPitchYaw<T>::SafeSoftNorm(const Vector3<T> &x) const {
  using std::sqrt;
  const double epsilon_length =
      std::numeric_limits<double>::epsilon() * free_length();
  const double epsilon_length_squared = epsilon_length * epsilon_length;
  const T x2 = x.squaredNorm();
  if (scalar_predicate<T>::is_bool && (x2 < epsilon_length_squared)) {
    throw std::runtime_error("The length of the spring became nearly zero. "
                                 "Revisit your model to avoid this situation.");
  }
  return sqrt(x2 + epsilon_length_squared);
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcLengthTimeDerivative(
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>& vc) const {
  const math::RigidTransform<T>& X_WA = pc.get_X_WB(bodyA().node_index());
  const math::RigidTransform<T>& X_WB = pc.get_X_WB(bodyB().node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // Using a "soft" norm we define a "soft length" as ℓₛ = ‖p_PQ‖ₛ.
  const T length_soft = SafeSoftNorm(p_PQ_W);

  const Vector3<T> r_PQ_W = p_PQ_W / length_soft;

  // Compute the velocities of P and Q so that we can add the damping force.
  // p_PAo = p_WAo - p_WP
  const Vector3<T> p_PAo_W = X_WA.translation() - p_WP;
  const SpatialVelocity<T>& V_WP =
      vc.get_V_WB(bodyA().node_index()).Shift(-p_PAo_W);

  // p_QBo = p_WBo - p_WQ
  const Vector3<T> p_QBo_W = X_WB.translation() - p_WQ;
  const SpatialVelocity<T>& V_WQ =
      vc.get_V_WB(bodyB().node_index()).Shift(-p_QBo_W);

  // Relative velocity of P in Q, expressed in the world frame.
  const Vector3<T> v_PQ_W = V_WQ.translational() - V_WP.translational();
  // The rate at which the length of the spring changes.
  const T length_dot = v_PQ_W.dot(r_PQ_W);

  return length_dot;
}
#endif

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)

