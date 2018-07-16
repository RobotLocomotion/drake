#include "drake/multibody/multibody_tree/spring_damper.h"

#include <vector>

#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
SpringDamper<T>::SpringDamper(
    const Body<T>& bodyA, const Vector3<double>& p_AP,
    const Body<T>& bodyB, const Vector3<double>& p_BQ,
    double rest_length, double stiffness, double damping) :
    ForceElement<T>(bodyA.model_instance()),
    bodyA_(bodyA),
    p_AP_(p_AP),
    bodyB_(bodyB),
    p_BQ_(p_BQ),
    rest_length_(rest_length),
    stiffness_(stiffness), damping_(damping) {}

template <typename T>
void SpringDamper<T>::DoCalcAndAddForceContribution(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    MultibodyForces<T>* forces) const {
  using std::sqrt;

  // Alias to the array of applied body forces:
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();

  const Isometry3<T>& X_WA = pc.get_X_WB(bodyA().template node_index());
  const Isometry3<T>& X_WB = pc.get_X_WB(bodyB().template node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // Using a "soft" norm we define a "soft length" as ℓₛ = ‖p_PQ‖ₛ.
  const T length_soft = SoftNorm(p_PQ_W);

  const Vector3<T> r_PQ_W = p_PQ_W / length_soft;

  // Force on A, applied at P, expressed in the world frame.
  Vector3<T> f_AP_W =
      stiffness() * (length_soft - rest_length()) * r_PQ_W;

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

  // Add the damping force on A at P.
  f_AP_W += damping() * length_dot * r_PQ_W;

  // Shift to the corresponding body frame and add spring-damper contribution
  // to the output forces.
  F_Bo_W_array[bodyA().node_index()] +=
      SpatialForce<T>(Vector3<T>::Zero(), f_AP_W).Shift(p_PAo_W);
  F_Bo_W_array[bodyB().node_index()] +=
      SpatialForce<T>(Vector3<T>::Zero(), -f_AP_W).Shift(p_QBo_W);
}

template <typename T>
T SpringDamper<T>::CalcPotentialEnergy(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc) const {
  const Isometry3<T>& X_WA =   pc.get_X_WB(bodyA().template node_index());
  const Isometry3<T>& X_WB = pc.get_X_WB(bodyB().template node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  // Vector from P to Q. It's length is the current length of the spring.
  const Vector3<T> p_PQ_W = p_WQ - p_WP;

  // We use the same "soft" norm used in the force computation for consistency.
  const T delta_length = SoftNorm(p_PQ_W) - rest_length();

  return 0.5 * stiffness() * delta_length * delta_length;
}

template <typename T>
T SpringDamper<T>::CalcConservativePower(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
#if 0
  const SpatialVelocity<T>& V_WA = vc.get_V_WB(bodyA().node_index());
  const SpatialVelocity<T>& V_WB = vc.get_V_WB(bodyB().node_index());

  const Vector3<T> v_AB_W = V_WA.translational() - V_WB.translational();

  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  const MultibodyTree<T>& model = this->get_parent_tree();
  const int num_bodies = model.num_bodies();
  T TotalConservativePower = 0.0;
  // Skip the "world" body.
  for (BodyIndex body_index(1); body_index < num_bodies; ++body_index) {
    const Body<T>& body = model.get_body(body_index);

    // TODO(amcastro-tri): Replace this CalcXXX() calls by GetXXX() calls once
    // caching is in place.
    const T mass = body.get_mass(context);
    const Vector3<T> p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Isometry3<T>& X_WB = pc.get_X_WB(body.node_index());
    const Matrix3<T> R_WB = X_WB.linear();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const SpatialVelocity<T>& V_WB = vc.get_V_WB(body.node_index());
    const SpatialVelocity<T> V_WBcm = V_WB.Shift(p_BoBcm_W);
    const Vector3<T>& v_WBcm = V_WBcm.translational();

    // The conservative power is defined to be positive when the potential
    // energy decreases.
    TotalConservativePower += (mass * v_WBcm.dot(gravity_vector()));
  }
  return TotalConservativePower;
#endif
  return T(0);
}

template <typename T>
T SpringDamper<T>::CalcNonConservativePower(
    const MultibodyTreeContext<T>&,
    const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&) const {
  // A uniform gravity field is conservative. Therefore return zero power.
  return 0.0;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
SpringDamper<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Body<ToScalar>& bodyA_clone =
      tree_clone.get_body(bodyA().index());
  const Body<ToScalar>& bodyB_clone =
      tree_clone.get_body(bodyB().index());

  // Make the Joint<T> clone.
  auto spring_damper_clone = std::make_unique<SpringDamper<ToScalar>>(
      bodyA_clone, point_on_bodyA(), bodyB_clone, point_on_bodyB(),
      rest_length(), stiffness(), damping());

  return std::move(spring_damper_clone);
}

template <typename T>
std::unique_ptr<ForceElement<double>>
SpringDamper<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
SpringDamper<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
T SpringDamper<T>::SoftNorm(const Vector3<T>& x) const {
  using std::sqrt;
  const T epsilon_squared = std::numeric_limits<double>::epsilon();
  return sqrt(x.squaredNorm() + epsilon_squared);
}

// Explicitly instantiates on the most common scalar types.
template class SpringDamper<double>;
template class SpringDamper<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
