#include "drake/systems/rendering/pose_bundle.h"

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace drake {
namespace systems {
namespace rendering {

using math::RigidTransform;

template <typename T>
PoseBundle<T>::PoseBundle(int num_poses)
    : poses_(num_poses),
      velocities_(num_poses),
      names_(num_poses),
      ids_(num_poses) {}

template <typename T>
PoseBundle<T>::~PoseBundle() {}

template <typename T>
int PoseBundle<T>::get_num_poses() const {
  return static_cast<int>(poses_.size());
}

template <typename T>
const RigidTransform<T>& PoseBundle<T>::get_transform(int index) const {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  return poses_[index];
}

template <typename T>
void PoseBundle<T>::set_transform(int index, const RigidTransform<T>& pose) {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  poses_[index] = pose;
}

template <typename T>
const FrameVelocity<T>& PoseBundle<T>::get_velocity(int index) const {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  return velocities_[index];
}

template <typename T>
void PoseBundle<T>::set_velocity(int index, const FrameVelocity<T>& velocity) {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  velocities_[index] = velocity;
}

template <typename T>
const std::string& PoseBundle<T>::get_name(int index) const {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  return names_[index];
}

template <typename T>
void PoseBundle<T>::set_name(int index, const std::string& name) {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  names_[index] = name;
}
template <typename T>
int PoseBundle<T>::get_model_instance_id(int index) const {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  return ids_[index];
}

template <typename T>
void PoseBundle<T>::set_model_instance_id(int index, int id) {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  DRAKE_DEMAND(id >= 0);
  ids_[index] = id;
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::rendering::PoseBundle)

#pragma GCC diagnostic pop
