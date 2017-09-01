#include "drake/multibody/multibody_tree/force_element.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
UniformGravityElement<T>::UniformGravityElement(Vector3<double> g_W) :
    g_W_(g_W) {}

template <typename T>
void UniformGravityElement<T>::DoCalcAndAddForceContribution(
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
    const T& mass = body.get_default_mass();
    const Vector3<double>& p_BoBcm_B = body.get_default_com();
    const Matrix3<T> R_WB = pc.get_X_WB(node_index).rotation();
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const Vector3<T> f_Bcm_W = mass * g_W();
    const SpatialForce<T> F_Bo_W(p_BoBcm_W.cross(f_Bcm_W), f_Bcm_W);
    F_Bo_W_array->at(node_index) += F_Bo_W;
  }
}

// Explicitly instantiates on the most common scalar types.
template class UniformGravityElement<double>;
template class UniformGravityElement<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
