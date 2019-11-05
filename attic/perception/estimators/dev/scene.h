#pragma once

#include <Eigen/Core>

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace perception {
namespace estimators {

DRAKE_DEPRECATED("2020-02-01", "The attic perception package is being removed.")
typedef RigidBodyFrame<double> RigidBodyFramed;
DRAKE_DEPRECATED("2020-02-01", "The attic perception package is being removed.")
typedef KinematicsCache<double> KinematicsCached;

DRAKE_DEPRECATED("2020-02-01", "The attic perception package is being removed.")
typedef int BodyIndex;
DRAKE_DEPRECATED("2020-02-01", "The attic perception package is being removed.")
typedef int FrameIndex;

// TODO(eric.cousineau): Replace with SceneGraph when it becomes available
// Presently, RigidBodyTree<> can function as a (dense) scene graph of sorts.
/**
 * A scene describes a group of objects (stored within a rigid body tree
 * structure), the world frame, and the camera frame. This is used to for
 * posing things such as an ICP formulation.
 */
class DRAKE_DEPRECATED("2020-02-01",
    "The attic perception package is being removed.")
Scene {
 public:
  Scene(const RigidBodyTreed* tree, const FrameIndex frame_world,
        const FrameIndex frame_camera)
      : tree_(tree), frame_world_(frame_world), frame_camera_(frame_camera) {}
  const RigidBodyTreed& tree() const { return *tree_; }
  /** @brief World frame, shared between model and the camera. */
  FrameIndex frame_world() const { return frame_world_; }
  /** @brief Camera frame. */
  FrameIndex frame_camera() const { return frame_camera_; }

 private:
  const RigidBodyTreed* tree_;
  FrameIndex frame_world_;
  FrameIndex frame_camera_;
};

/**
 * Stored state of a scene. Presently, only incorporates position configuration.
 */
class DRAKE_DEPRECATED("2020-02-01",
    "The attic perception package is being removed.")
SceneState {
 public:
  explicit SceneState(const Scene* scene)
      : scene_(scene), tree_cache_(scene->tree().CreateKinematicsCache()) {}

  /**
   * Update the state of the scene given the position kinematics.
   * @param q State of scene().tree().
   */
  void Update(const Eigen::VectorXd& q);

  const Scene& scene() const { return *scene_; }
  KinematicsCached& tree_cache() { return tree_cache_; }
  const KinematicsCached& tree_cache() const { return tree_cache_; }
  const Eigen::VectorXd& q() const { return tree_cache_.getQ(); }

 private:
  const Scene* scene_;
  KinematicsCached tree_cache_;
};

}  // namespace estimators
}  // namespace perception
}  // namespace drake
