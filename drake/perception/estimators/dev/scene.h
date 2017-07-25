#pragma once

#include <Eigen/Core>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace perception {
namespace estimators {

typedef RigidBodyFrame<double> RigidBodyFramed;
typedef KinematicsCache<double> KinematicsCached;

typedef int BodyIndex;
typedef int FrameIndex;

/**
 * A scene describes a group of objects (stored within a rigid body tree
 * structure), the world frame, and the camera frame. This is used to for
 * posing things such as an ICP formulation.
 */
// TODO(eric.cousineau): Replace with GeometryWorld when it becomes available
// Presently, RigidBodyTree<> can function as a (dense) scene graph of sorts.
class Scene {
 public:
  Scene(const RigidBodyTreed* tree, const FrameIndex frame_W,
        const FrameIndex frame_C)
      : tree_(tree), frame_W_(frame_W), frame_C_(frame_C) {}
  const RigidBodyTreed& tree() const { return *tree_; }
  /** @brief World frame, shared between model and the camera. */
  FrameIndex frame_W() const { return frame_W_; }
  /** @brief Camera frame. */
  FrameIndex frame_C() const { return frame_C_; }

 private:
  const RigidBodyTreed* tree_;
  FrameIndex frame_W_;
  FrameIndex frame_C_;
};

/**
 * Stored state of a scene.
 */
class SceneState {
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
