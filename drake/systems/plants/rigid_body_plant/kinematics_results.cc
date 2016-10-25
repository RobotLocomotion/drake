#include "drake/systems/plants/rigid_body_plant/kinematics_results.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/plants/KinematicsCache.h"

namespace drake {
namespace systems {

template<typename T>
KinematicsResults<T>::KinematicsResults(const RigidBodyTree* tree) :
    tree_(tree), kinematics_cache_(tree_->bodies) {
}

template<typename T>
int KinematicsResults<T>::get_num_bodies() const {
  return tree_->get_num_bodies();
}

template<typename T>
int KinematicsResults<T>::get_num_positions() const {
  return kinematics_cache_.get_num_positions();
}

template<typename T>
int KinematicsResults<T>::get_num_velocities() const {
  return kinematics_cache_.get_num_velocities();
}

template<typename T>
Quaternion<T> KinematicsResults<T>::get_body_orientation(int body_index) const {
  Isometry3<T>
      pose = tree_->relativeTransform(kinematics_cache_, 0, body_index);
  Vector4<T> quat_vector = drake::math::rotmat2quat(pose.linear());
  // Note that Eigen quaternion elements are not laid out in memory in the
  // same way Drake currently aligns them. See issue #3470.
  return Quaternion<T>(
      quat_vector[0], quat_vector[1], quat_vector[2], quat_vector[3]);
}

template<typename T>
Vector3<T> KinematicsResults<T>::get_body_position(int body_index) const {
  Isometry3<T>
      pose = tree_->relativeTransform(kinematics_cache_, 0, body_index);
  return pose.translation();
}

template<typename T>
void KinematicsResults<T>::UpdateFromContext(const Context<T> &context) {
  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_continuous_state_vector()).get_value();

  const int nq = tree_->get_num_positions();
  const int nv = tree_->get_num_velocities();

  // TODO(amcastro-tri): We would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not explicitly instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);

  kinematics_cache_.initialize(q, v);
  tree_->doKinematics(kinematics_cache_, false);
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_EXPORT KinematicsResults<double>;

}  // namespace systems
}  // namespace drake

