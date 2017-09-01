#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

#include "drake/common/eigen_autodiff_types.h"
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
    const VelocityKinematicsCache<T>& vc,
    std::vector<SpatialForce<T>>* F_Bo_W_array,
    Eigen::Ref<VectorX<T>> tau) const {
  // Add the force of gravity contribution for each body in the model.
  // Skip the world.
  const MultibodyTree<T>& model = this->get_parent_tree();
  const int num_bodies = model.get_num_bodies();
  for (BodyNodeIndex node_index(1); node_index < num_bodies; ++node_index) {
    const Body<T>& body = model.get_body(node_index);

    // TODO(amcastro-tri): Use methods get_mass(context) and get_com(context)
    // once caching is in place so that we can retrieve these from the
    // parameters.
    const double mass = body.get_default_mass();
    const Vector3<double>& p_BoBcm_B = body.get_default_com();
    const Matrix3<T> R_WB = pc.get_X_WB(node_index).rotation();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const Vector3<T> f_Bcm_W = mass * g_W();
    const SpatialForce<T> F_Bo_W(p_BoBcm_W.cross(f_Bcm_W), f_Bcm_W);
    F_Bo_W_array->at(node_index) += F_Bo_W;
  }
}

template <typename T>
T UniformGravityFieldElement<T>::CalcPotentialEnergy(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  const MultibodyTree<T>& model = this->get_parent_tree();
  const int num_bodies = model.get_num_bodies();
  T TotalPotentialEnergy = 0.0;
  for (BodyNodeIndex node_index(1); node_index < num_bodies; ++node_index) {
    const Body<T>& body = model.get_body(node_index);

    // TODO(amcastro-tri): Use methods get_mass(context) and get_com(context)
    // once caching is in place so that we can retrieve these from the
    // parameters.
    const double mass = body.get_default_mass();
    const Vector3<double>& p_BoBcm_B = body.get_default_com();
    const Isometry3<T>& X_WB = pc.get_X_WB(node_index);
    const Matrix3<T> R_WB = X_WB.rotation();
    const Vector3<T> p_WBo = X_WB.translation();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W and/or p_WBcm.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;
    const Vector3<T> p_WBcm = p_WBo + p_BoBcm_W;

    TotalPotentialEnergy -= (mass * p_WBcm.dot(g_W()));
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
  for (BodyNodeIndex node_index(1); node_index < num_bodies; ++node_index) {
    const Body<T>& body = model.get_body(node_index);

    // TODO(amcastro-tri): Use methods get_mass(context) and get_com(context)
    // once caching is in place so that we can retrieve these from the
    // parameters.
    const double mass = body.get_default_mass();
    const Vector3<double>& p_BoBcm_B = body.get_default_com();
    const Isometry3<T>& X_WB = pc.get_X_WB(node_index);
    const Matrix3<T> R_WB = X_WB.rotation();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const SpatialVelocity<T>& V_WB = vc.get_V_WB(node_index);
    const SpatialVelocity<T> V_WBcm = V_WB.Shift(p_BoBcm_W);
    const Vector3<T>& v_WBcm = V_WBcm.translational();

    // The conservative power is defined to be positive when the potential
    // energy decreases.
    TotalConservativePower += (mass * v_WBcm.dot(g_W()));
  }
  return TotalConservativePower;
}

template <typename T>
T UniformGravityFieldElement<T>::CalcNonConservativePower(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  // A uniform gravity field is conservative. Therefore return zero power.
  return 0.0;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return std::make_unique<UniformGravityFieldElement<double>>(g_W());
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return std::make_unique<UniformGravityFieldElement<AutoDiffXd>>(g_W());
}

// Explicitly instantiates on the most common scalar types.
template class UniformGravityFieldElement<double>;
template class UniformGravityFieldElement<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
