#include "drake/multibody/tree/linear_spring_damper.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
LinearSpringDamper<T>::LinearSpringDamper(
    const Body<T>& bodyA, const Vector3<double>& p_AP,
    const Body<T>& bodyB, const Vector3<double>& p_BQ,
    double free_length, double stiffness, double damping) :
    ForceElement<T>(bodyA.model_instance()),
    bodyA_(bodyA),
    p_AP_(p_AP),
    bodyB_(bodyB),
    p_BQ_(p_BQ),
    free_length_(free_length),
    stiffness_(stiffness), damping_(damping) {
  DRAKE_THROW_UNLESS(free_length > 0);
  DRAKE_THROW_UNLESS(stiffness >= 0);
  DRAKE_THROW_UNLESS(damping >= 0);
}

template <typename T>
void LinearSpringDamper<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>& vc,
    MultibodyForces<T>* forces) const {
  using std::sqrt;

  const math::RigidTransform<T>& X_WA = pc.get_X_WB(bodyA().node_index());
  const math::RigidTransform<T>& X_WB = pc.get_X_WB(bodyB().node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // Using a "soft" norm we define a "soft length" as ℓₛ = ‖p_PQ‖ₛ.
  const T length_soft = SafeSoftNorm(p_PQ_W);

  const Vector3<T> r_PQ_W = p_PQ_W / length_soft;

  // Force on A, applied at P, expressed in the world frame.
  Vector3<T> f_AP_W =
      stiffness() * (length_soft - free_length()) * r_PQ_W;

  // The rate at which the length of the spring changes.
  const T length_dot = CalcLengthTimeDerivative(pc, vc);

  // Add the damping force on A at P.
  f_AP_W += damping() * length_dot * r_PQ_W;

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
}

template <typename T>
T LinearSpringDamper<T>::CalcPotentialEnergy(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& pc) const {
  const math::RigidTransform<T>& X_WA = pc.get_X_WB(bodyA().node_index());
  const math::RigidTransform<T>& X_WB = pc.get_X_WB(bodyB().node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // We use the same "soft" norm used in the force computation for consistency.
  const T delta_length = SafeSoftNorm(p_PQ_W) - free_length();

  return 0.5 * stiffness() * delta_length * delta_length;
}

template <typename T>
T LinearSpringDamper<T>::CalcConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>& vc) const {
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
}

template <typename T>
T LinearSpringDamper<T>::CalcNonConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>& vc) const {
  // The rate at which the length of the spring changes.
  const T length_dot = CalcLengthTimeDerivative(pc, vc);
  // Energy is dissipated at rate Pnc = -d⋅(dℓ/dt)²:
  return -damping() * length_dot * length_dot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearSpringDamper<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Body<ToScalar>& bodyA_clone =
      tree_clone.get_body(bodyA().index());
  const Body<ToScalar>& bodyB_clone =
      tree_clone.get_body(bodyB().index());

  // Make the LinearSpringDamper<T> clone.
  auto spring_damper_clone = std::make_unique<LinearSpringDamper<ToScalar>>(
      bodyA_clone, p_AP(), bodyB_clone, p_BQ(),
      free_length(), stiffness(), damping());

  return spring_damper_clone;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
LinearSpringDamper<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
LinearSpringDamper<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
LinearSpringDamper<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
T LinearSpringDamper<T>::SafeSoftNorm(const Vector3<T> &x) const {
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
T LinearSpringDamper<T>::CalcLengthTimeDerivative(
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

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearSpringDamper)
