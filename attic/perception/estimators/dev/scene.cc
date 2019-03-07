#include "drake/perception/estimators/dev/scene.h"

namespace drake {
namespace perception {
namespace estimators {

void SceneState::Update(const Eigen::VectorXd& q) {
  tree_cache_.initialize(q);
  scene_->tree().doKinematics(tree_cache_);
}

}  // namespace estimators
}  // namespace perception
}  // namespace drake
