#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace systems {
namespace rendering {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

PoseBundleToDrawMessage::PoseBundleToDrawMessage() {
  this->DeclareAbstractInputPort(
      kUseDefaultName, Value<PoseBundle<double>>());
  this->DeclareAbstractOutputPort(
      kUseDefaultName, &PoseBundleToDrawMessage::CalcViewerDrawMessage);
}

PoseBundleToDrawMessage::~PoseBundleToDrawMessage() {}

void PoseBundleToDrawMessage::CalcViewerDrawMessage(
    const Context<double>& context, lcmt_viewer_draw* output) const {
  const PoseBundle<double>& poses =
      this->get_input_port(0).Eval<PoseBundle<double>>(context);

  lcmt_viewer_draw& message = *output;

  const int n = poses.get_num_poses();

  message.timestamp = static_cast<int64_t>(context.get_time() * 1000.0);
  message.num_links = n;
  message.link_name.resize(n);
  message.robot_num.resize(n);
  message.position.resize(n);
  message.quaternion.resize(n);

  for (int i = 0; i < n; ++i) {
    message.robot_num[i] = poses.get_model_instance_id(i);

    message.link_name[i] = poses.get_name(i);

    Eigen::Translation<double, 3> t(poses.get_transform(i).translation());
    message.position[i].resize(3);
    message.position[i][0] = t.x();
    message.position[i][1] = t.y();
    message.position[i][2] = t.z();

    Eigen::Quaternion<double> q(
        poses.get_transform(i).rotation().ToQuaternion());
    message.quaternion[i].resize(4);
    message.quaternion[i][0] = q.w();
    message.quaternion[i][1] = q.x();
    message.quaternion[i][2] = q.y();
    message.quaternion[i][3] = q.z();
  }
}

#pragma GCC diagnostic pop

}  // namespace rendering
}  // namespace systems
}  // namespace drake
