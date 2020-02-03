#include "drake/systems/rendering/frame_visualizer.h"

#include <utility>

#include "drake/common/extract_double.h"

namespace drake {
namespace systems {
namespace rendering {

using multibody::Frame;
using multibody::MultibodyPlant;

template <typename T>
FrameVisualizer<T>::FrameVisualizer(const MultibodyPlant<T>* plant,
                                    const std::vector<const Frame<T>*>& frames)
    : plant_(*plant), frames_(frames) {
  auto context = plant_.CreateDefaultContext();
  Value<Context<T>> val{std::move(context)};
  this->DeclareAbstractInputPort("kinematics", val);
  this->DeclareAbstractOutputPort("visualization_message", lcmt_viewer_draw{},
                                  &FrameVisualizer::CalcOutputMsg);
  this->set_name("frame_visualizer");
}

template <typename T>
void FrameVisualizer<T>::CalcOutputMsg(const Context<T>& context,
                                       lcmt_viewer_draw* msg) const {
  const auto& kinematics =
      this->get_input_port(0).template Eval<Context<T>>(context);

  msg->num_links = static_cast<int>(frames_.size());
  msg->link_name.resize(msg->num_links);
  msg->robot_num.resize(msg->num_links);

  msg->position.resize(msg->num_links);
  msg->quaternion.resize(msg->num_links);

  for (size_t i = 0; i < frames_.size(); ++i) {
    msg->link_name[i] = frames_[i]->name();
    msg->robot_num[i] = frames_[i]->model_instance();
    auto X_WF = plant_.CalcRelativeTransform(kinematics, plant_.world_frame(),
                                             *frames_[i]);
    msg->position[i].resize(3);
    for (int j = 0; j < 3; j++) {
      msg->position[i][j] =
          static_cast<float>(ExtractDoubleOrThrow(X_WF.translation()[j]));
    }
    const auto quat = X_WF.rotation().ToQuaternion();
    msg->quaternion[i].resize(4);
    msg->quaternion[i][0] = static_cast<float>(ExtractDoubleOrThrow(quat.w()));
    msg->quaternion[i][1] = static_cast<float>(ExtractDoubleOrThrow(quat.x()));
    msg->quaternion[i][2] = static_cast<float>(ExtractDoubleOrThrow(quat.y()));
    msg->quaternion[i][3] = static_cast<float>(ExtractDoubleOrThrow(quat.z()));
  }
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::rendering::FrameVisualizer)
