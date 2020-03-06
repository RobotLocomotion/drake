#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/rendering/frame_velocity.h"

namespace drake {
namespace systems {
namespace rendering {

// TODO(david-german-tri, SeanCurtis-TRI): Subsume this functionality into
// SceneGraph when it becomes available.

// TODO(david-german-tri): Consider renaming this to FrameKinematicsBundle,
// since it contains both poses and velocities.

/// PoseBundle is a container for a set of poses, represented by an Isometry3,
/// and corresponding velocities, represented by a FrameVelocity. The poses and
/// velocities are expressed in the world frame: X_WFi, V_WFi. Each pose has a
/// name and a model instance ID.  If two poses in the bundle have the same
/// model instance ID, they must not have the same name.
///
/// @tparam_default_scalar
template <typename T>
class PoseBundle {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PoseBundle)

  explicit PoseBundle(int num_poses = 0);
  ~PoseBundle();

  int get_num_poses() const;
  const Isometry3<T>& get_pose(int index) const;
  void set_pose(int index, const Isometry3<T>& pose);

  const FrameVelocity<T>& get_velocity(int index) const;
  void set_velocity(int index, const FrameVelocity<T>& velocity);

  const std::string& get_name(int index) const;
  void set_name(int index, const std::string& name);

  int get_model_instance_id(int index) const;
  void set_model_instance_id(int index, int id);

 private:
  std::vector<Isometry3<T>> poses_;
  std::vector<FrameVelocity<T>> velocities_;
  std::vector<std::string> names_;
  std::vector<int> ids_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
