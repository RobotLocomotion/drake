#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
                          const Frame<T>& frameAb,
                          const Frame<T>& frameBa,
                          const Vector3<T>& torque_stiffness,
                          const Vector3<T>& torque_damping,
                          const Vector3<T>& force_stiffness,
                          const Vector3<T>& force_damping) :
    ForceElement<T>(frameAb.body().model_instance()),
    frameAb_(frameAb),
    frameBa_(frameBa),
    torque_stiffness_(torque_stiffness),
    torque_damping_(torque_damping),
    force_stiffness_(force_stiffness),
    force_damping_(force_damping) {
}

template <typename T>
void LinearBushingRollPitchYaw<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */,
    MultibodyForces<T>* /* forces */) const {
#if 0
  // Get the bushing frame's relative transform and relative spatial velocity.
  math::RigidTransform<T> X_AbBa = frameBa().CalcPose(context, frameAb());
  const SpatialVelocity<T> V_AbBa =
      frameBa().CalcSpatialVelocity(context, frameAb(), frameAb());

  // Extract the bushing frame's relative position and translational velocity.
  const Vector3<T>& p_AbBa = X_AbBa.translation();
  const Vector3<T>& v_AbBa = V_AbBa.translational();

  // Calculate the bushing force on Ab, expressed in frame Ab.
  const Vector3<T> f_Ab = force_stiffness.dot(p_AbBa)
                        + force_damping.dot(v_AbBa);

  // Express the bushing force on Ab in the world frame.
  const Frame<T>& world_frame = frameAb().get_parent_tree().world_frame();

  // Calculate the bushing's roll-pitch-yaw angles q₀, q₁, q₂.
  const math::RollPitchYaw<T> rpy(X_AbBa.rotation());
  const<T> q0 = rpy(0),  q1 = rpy(1),  q2 = rpy(2);

  // Calculate the bushing's roll-pitch-yaw rates  q̇₀, q̇₁, q̇₂
  const T c0 = cos(q0), s0 = sin(q0);
  const T c1 = cos(q1), s1 = sin(q1);

  // Calculate the bushing torque on Ab, expressed in frame Ab.
  const T k0 = torque_stiffness(0);
  const T k1 = torque_stiffness(1);
  const T k2 = torque_stiffness(2);
  const T b0 = torque_damping(0);
  const T b1 = torque_damping(1);
  const T b2 = torque_damping(2);

  const T T0 = k0*q0 + b0*q0Dt;
  const T T1 = k1*q1 + b1*q1Dt;
  const T T2 = k2*q2 + b2*q2Dt;

  const T tx = c0*c1*TB
  const Vector3<T> t_Ab =

  // Shift to the corresponding body frame and add spring-damper contribution
  // to the output forces.
  // Alias to the array of applied body forces:
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();

  // p_PAo = p_WAo - p_WP
  const Vector3<T> p_PAo_W = X_WA.translation() - p_WP;
  F_Bo_W_array[bodyA().node_index()] +=
      SpatialForce<T>(Vector3<T>::Zero(), f_AP_W).Shift(p_PAo_W);

  // p_QBo = p_WBo - p_WQ
  const Vector3<T> p_QBo_W = X_WB.translation() - p_WQ;
  F_Bo_W_array[bodyB().node_index()] +=
      SpatialForce<T>(Vector3<T>::Zero(), -f_AP_W).Shift(p_QBo_W);
#endif
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPotentialEnergy(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */) const {
#if 0
  const math::RigidTransform<T>& X_WA = pc.get_X_WB(bodyA().node_index());
  const math::RigidTransform<T>& X_WB = pc.get_X_WB(bodyB().node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // We use the same "soft" norm used in the force computation for consistency.
  const T delta_length = SafeSoftNorm(p_PQ_W) - free_length();

  return 0.5 * stiffness() * delta_length * delta_length;
#endif
  return 0;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
#if 0
  // Since the potential energy is:
  //  V = 1/2⋅k⋅(ℓ-ℓ₀)²
  // The conservative power is defined as:
  //  Pc = -d(V)/dt
  // being positive when the potential energy decreases.

  const math::RigidTransform<T>& X_WA = pc.get_X_WB(bodyA().node_index());
  const math::RigidTransform<T>& X_WB = pc.get_X_WB(bodyB().node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // We use the same "soft" norm used in the force computation for consistency.
  const T delta_length = SafeSoftNorm(p_PQ_W) - free_length();

  // The rate at which the length of the spring changes.
  const T length_dot = CalcLengthTimeDerivative(pc, vc);

  // Since V = 1/2⋅k⋅(ℓ-ℓ₀)² we have that, from its definition:
  // Pc = -d(V)/dt = -k⋅(ℓ-ℓ₀)⋅dℓ/dt
  const T Pc = -stiffness() * delta_length * length_dot;
  return Pc;
#endif
  return 0;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcNonConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
#if 0
  // The rate at which the length of the spring changes.
  const T length_dot = CalcLengthTimeDerivative(pc, vc);
  // Energy is dissipated at rate Pnc = -d⋅(dℓ/dt)²:
  return -damping() * length_dot * length_dot;
#endif
  return 0;
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

