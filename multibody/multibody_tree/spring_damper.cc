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
    const VelocityKinematicsCache<T>&,
    MultibodyForces<T>* forces) const {
  // Alias to the array of applied body forces:
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();

  const Isometry3<T>& X_WA = pc.get_X_WB(bodyA().template node_index());
  const Isometry3<T>& X_WB = pc.get_X_WB(bodyB().template node_index());

  const Vector3<T> p_WP = X_WA * p_AP_.template cast<T>();
  const Vector3<T> p_WQ = X_WB * p_BQ_.template cast<T>();

  const Vector3<T> p_PQ_W = p_WQ - p_WP;
  const T length = p_PQ_W.norm();

  // TODO: Check for zero norm or use "soft" norms as defined by Sherm.
  const Vector3<T> r_PQ_W = p_PQ_W.normalized();

  // Force on A, applied at P, expressed in the world frame.
  const Vector3<T> f_AP_W = stiffness() * (length - rest_length_) * r_PQ_W;

  // Force on B, applied at Q, expressed in the world frame.
  const Vector3<T> f_BQ_W = -f_AP_W;

  F_Bo_W_array[bodyA().node_index()] +=
      SpatialForce<T>(Vector3<T>::Zero(), f_AP_W);

  F_Bo_W_array[bodyB().node_index()] +=
      SpatialForce<T>(Vector3<T>::Zero(), f_BQ_W);
}

template <typename T>
T SpringDamper<T>::CalcPotentialEnergy(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc) const {
#if 0
  const Isometry3<T>& X_WA = pc.get_X_WB(bodyA().node_index());
  const Isometry3<T>& X_WB = pc.get_X_WB(bodyB().node_index());

  const Vector3<T> p_WP = X_WA * p_AP_;
  const Vector3<T> p_WQ = X_WB * p_BQ_;

  const Vector3<T> p_PQ_W = p_WQ - p_WP;
  const T length = p_PQ_W.norm();

  return 0.5 * stiffness() * length * length;
#endif
  return T(0);
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

// Explicitly instantiates on the most common scalar types.
template class SpringDamper<double>;
template class SpringDamper<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
