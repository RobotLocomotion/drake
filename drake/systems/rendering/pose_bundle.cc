#include "drake/systems/rendering/pose_bundle.h"

#include <Eigen/Dense>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
PoseBundle<T>::PoseBundle(int num_poses)
    : poses_(num_poses), names_(num_poses), ids_(num_poses) {
}

template <typename T>
PoseBundle<T>::~PoseBundle() {}

template <typename T>
int PoseBundle<T>::get_num_poses() const {
  return static_cast<int>(poses_.size());
}

template <typename T>
const Isometry3<T>& PoseBundle<T>::get_pose(int index) const {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  return poses_[index];
}

template <typename T>
void PoseBundle<T>::set_pose(int index, const Isometry3<T>& pose) {
  DRAKE_DEMAND(index >= 0 && index < get_num_poses());
  poses_[index] = pose;
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


template class PoseBundle<double>;
template class PoseBundle<AutoDiffXd>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
