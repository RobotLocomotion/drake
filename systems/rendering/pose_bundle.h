#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/math/rigid_transform.h"
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
class DRAKE_DEPRECATED("2021-12-01",
                       "PoseBundle is no longer in use. Visualizers typically "
                       "connect to SceneGraph's QueryObject port.")
PoseBundle {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PoseBundle)

  explicit PoseBundle(int num_poses = 0);
  ~PoseBundle();

  int get_num_poses() const;
  const math::RigidTransform<T>& get_transform(int index) const;
  void set_transform(int index, const math::RigidTransform<T>& pose);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const FrameVelocity<T>& get_velocity(int index) const;
  void set_velocity(int index, const FrameVelocity<T>& velocity);
#pragma GCC diagnostic pop

  const std::string& get_name(int index) const;
  void set_name(int index, const std::string& name);

  int get_model_instance_id(int index) const;
  void set_model_instance_id(int index, int id);

 private:
  std::vector<math::RigidTransform<T>> poses_;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<FrameVelocity<T>> velocities_;
#pragma GCC diagnostic pop

  std::vector<std::string> names_;
  std::vector<int> ids_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
