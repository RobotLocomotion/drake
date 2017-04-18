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

template <typename T>
std::unique_ptr<PoseBundle<AutoDiffXd>> PoseBundle<T>::ToAutoDiffXd() const {
  // TODO(sherm1): Consider changing this to overload a default implementation
  // in AbstractValue, as discussed in
  // https://github.com/RobotLocomotion/drake/issues/5454

  auto bundle = std::make_unique<PoseBundle<AutoDiffXd>>(get_num_poses());
  for (int pose_index = 0; pose_index < get_num_poses(); pose_index++) {
    Isometry3<AutoDiffXd> pose(get_pose(pose_index));
    bundle->set_pose(pose_index, pose);
    FrameVelocity<AutoDiffXd> velocity;
    velocity.set_velocity(multibody::SpatialVelocity<AutoDiffXd>(
        get_velocity(pose_index).get_velocity().get_coeffs()));
    bundle->set_velocity(pose_index, velocity);
    bundle->set_name(pose_index, get_name(pose_index));
    bundle->set_model_instance_id(pose_index,
                                  get_model_instance_id(pose_index));
  }
  return bundle;
}

template class PoseBundle<double>;
template class PoseBundle<AutoDiffXd>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
