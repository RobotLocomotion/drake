#include "drake/multibody/rigid_body_plant/frame_visualizer.h"

namespace drake {
namespace systems {

FrameVisualizer::FrameVisualizer(
    const RigidBodyTree<double>* tree,
    const std::vector<RigidBodyFrame<double>>& local_transforms,
    drake::lcm::DrakeLcmInterface* lcm)
    : tree_(*tree), lcm_(lcm), local_transforms_(local_transforms) {
  DeclareInputPort(kVectorValued,
                   tree_.get_num_positions() + tree_.get_num_velocities());
  set_name("frame_visualizer");

  // This ensures that an explicit call to System::Publish() will produce
  // a message to the visualizer.
  this->DeclareForcedPublishEvent(&FrameVisualizer::PublishFramePose);

  // This is disabled if a publish period is set.
  DeclarePerStepPublishEvent(&FrameVisualizer::PerStepPublishFramePose);

  default_msg_.num_links = static_cast<int>(local_transforms_.size());
  default_msg_.link_name.resize(default_msg_.num_links);
  // The robot num is not relevant here.
  default_msg_.robot_num.resize(default_msg_.num_links, 0);
  std::vector<float> pos = {0, 0, 0};
  std::vector<float> quaternion = {1, 0, 0, 0};
  default_msg_.position.resize(default_msg_.num_links, pos);
  default_msg_.quaternion.resize(default_msg_.num_links, quaternion);
  for (size_t i = 0; i < local_transforms_.size(); ++i) {
    default_msg_.link_name[i] = local_transforms_[i].get_name();
  }
}

EventStatus FrameVisualizer::PublishFramePose(
    const systems::Context<double>& context) const {
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();

  auto state = EvalEigenVectorInput(context, 0);
  cache.initialize(state.head(tree_.get_num_positions()),
                   state.tail(tree_.get_num_velocities()));
  tree_.doKinematics(cache);

  drake::lcmt_viewer_draw msg = default_msg_;
  msg.timestamp = static_cast<int64_t>(context.get_time() * 1e3);

  Isometry3<double> X_WF;
  for (size_t i = 0; i < local_transforms_.size(); ++i) {
    X_WF = tree_.CalcFramePoseInWorldFrame(cache, local_transforms_[i]);

    for (int j = 0; j < 3; j++)
      msg.position[i][j] = static_cast<float>(X_WF.translation()[j]);
    Quaternion<double> quat(X_WF.linear());
    msg.quaternion[i][0] = static_cast<float>(quat.w());
    msg.quaternion[i][1] = static_cast<float>(quat.x());
    msg.quaternion[i][2] = static_cast<float>(quat.y());
    msg.quaternion[i][3] = static_cast<float>(quat.z());
  }

  drake::lcm::Publish(lcm_, lcm_channel_, msg, context.get_time());
  return EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace drake
