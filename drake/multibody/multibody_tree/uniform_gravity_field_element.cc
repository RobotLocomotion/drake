#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
UniformGravityFieldElement<T>::UniformGravityFieldElement(Vector3<double> g_W) :
    g_W_(g_W) {}

template <typename T>
void UniformGravityFieldElement<T>::DoCalcAndAddForceContribution(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>&,
    std::vector<SpatialForce<T>>* F_Bo_W_array,
    EigenPtr<VectorX<T>>) const {
  // Add the force of gravity contribution for each body in the model.
  // Skip the world.
  const MultibodyTree<T>& model = this->get_parent_tree();
  const int num_bodies = model.get_num_bodies();
  // Skip the "world" body.
  for (BodyIndex body_index(1); body_index < num_bodies; ++body_index) {
    const Body<T>& body = model.get_body(body_index);
    BodyNodeIndex node_index = body.get_node_index();

    // TODO(amcastro-tri): Replace this CalcXXX() calls by GetXXX() calls once
    // caching is in place.
    const T mass = body.get_mass(context);
    const Vector3<T> p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Matrix3<T> R_WB = pc.get_X_WB(node_index).linear();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const Vector3<T> f_Bcm_W = mass * gravity_vector();
    const SpatialForce<T> F_Bo_W(p_BoBcm_W.cross(f_Bcm_W), f_Bcm_W);
    F_Bo_W_array->at(node_index) += F_Bo_W;
  }
}

template <typename T>
T UniformGravityFieldElement<T>::CalcPotentialEnergy(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc) const {
  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  const MultibodyTree<T>& model = this->get_parent_tree();
  const int num_bodies = model.get_num_bodies();
  T TotalPotentialEnergy = 0.0;
  // Skip the "world" body.
  for (BodyIndex body_index(1); body_index < num_bodies; ++body_index) {
    const Body<T>& body = model.get_body(body_index);

    // TODO(amcastro-tri): Replace this CalcXXX() calls by GetXXX() calls once
    // caching is in place.
    const T mass = body.get_mass(context);
    const Vector3<T> p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Isometry3<T>& X_WB = pc.get_X_WB(body.get_node_index());
    const Matrix3<T> R_WB = X_WB.linear();
    const Vector3<T> p_WBo = X_WB.translation();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W and/or p_WBcm.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;
    const Vector3<T> p_WBcm = p_WBo + p_BoBcm_W;

    TotalPotentialEnergy -= (mass * p_WBcm.dot(gravity_vector()));
  }
  return TotalPotentialEnergy;
}

template <typename T>
T UniformGravityFieldElement<T>::CalcConservativePower(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  const MultibodyTree<T>& model = this->get_parent_tree();
  const int num_bodies = model.get_num_bodies();
  T TotalConservativePower = 0.0;
  // Skip the "world" body.
  for (BodyIndex body_index(1); body_index < num_bodies; ++body_index) {
    const Body<T>& body = model.get_body(body_index);

    // TODO(amcastro-tri): Replace this CalcXXX() calls by GetXXX() calls once
    // caching is in place.
    const T mass = body.get_mass(context);
    const Vector3<T> p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Isometry3<T>& X_WB = pc.get_X_WB(body.get_node_index());
    const Matrix3<T> R_WB = X_WB.linear();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const SpatialVelocity<T>& V_WB = vc.get_V_WB(body.get_node_index());
    const SpatialVelocity<T> V_WBcm = V_WB.Shift(p_BoBcm_W);
    const Vector3<T>& v_WBcm = V_WBcm.translational();

    // The conservative power is defined to be positive when the potential
    // energy decreases.
    TotalConservativePower += (mass * v_WBcm.dot(gravity_vector()));
  }
  return TotalConservativePower;
}

template <typename T>
T UniformGravityFieldElement<T>::CalcNonConservativePower(
    const MultibodyTreeContext<T>&,
    const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&) const {
  // A uniform gravity field is conservative. Therefore return zero power.
  return 0.0;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const MultibodyTree<double>&) const {
  return std::make_unique<UniformGravityFieldElement<double>>(gravity_vector());
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>&) const {
  return std::make_unique<UniformGravityFieldElement<AutoDiffXd>>(
      gravity_vector());
}

// Explicitly instantiates on the most common scalar types.
template class UniformGravityFieldElement<double>;
template class UniformGravityFieldElement<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
