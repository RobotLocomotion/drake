#include "drake/multibody/plant/frame_visualizer.h"

#include <utility>

namespace drake {
namespace multibody {

FrameVisualizer::FrameVisualizer(
    const MultibodyPlant<double>* plant,
    const std::vector<const Frame<double>*>& frames)
    : plant_(*plant), frames_(frames) {
  auto context = plant_.CreateDefaultContext();
  Value<systems::Context<double>> val{std::move(context)};
  this->DeclareAbstractInputPort("kinematics", val);
  this->DeclareAbstractOutputPort("visualization_message", lcmt_viewer_draw{},
      &FrameVisualizer::CalcOutputMsg);
  this->set_name("frame_visualizer");
}

void FrameVisualizer::CalcOutputMsg(const systems::Context<double>& context,
                                    lcmt_viewer_draw* msg) const {
  const auto& kinematics =
      get_input_port(0).Eval<systems::Context<double>>(context);

  msg->num_links = static_cast<int>(frames_.size());
  msg->link_name.resize(msg->num_links);
  // The robot num is not relevant here.
  msg->robot_num.resize(msg->num_links, 0);

  msg->position.resize(msg->num_links);
  msg->quaternion.resize(msg->num_links);

  for (size_t i = 0; i < frames_.size(); ++i) {
    msg->link_name[i] = frames_[i]->name();
    msg->position[i].resize(3);
    msg->quaternion[i].resize(4);
    auto X_WF = plant_.CalcRelativeTransform(kinematics, plant_.world_frame(),
                                             *frames_[i]);
    for (int j = 0; j < 3; j++)
      msg->position[i][j] = static_cast<float>(X_WF.translation()[j]);
    const auto quat = X_WF.rotation().ToQuaternion();
    msg->quaternion[i][0] = static_cast<float>(quat.w());
    msg->quaternion[i][1] = static_cast<float>(quat.x());
    msg->quaternion[i][2] = static_cast<float>(quat.y());
    msg->quaternion[i][3] = static_cast<float>(quat.z());
  }
}

}  // namespace multibody
}  // namespace drake
