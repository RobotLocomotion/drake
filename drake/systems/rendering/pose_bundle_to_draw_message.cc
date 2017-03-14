#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace systems {
namespace rendering {

PoseBundleToDrawMessage::PoseBundleToDrawMessage() {
  this->DeclareAbstractInputPort();
  this->DeclareAbstractOutputPort();
}

PoseBundleToDrawMessage::~PoseBundleToDrawMessage() {}

void PoseBundleToDrawMessage::DoCalcOutput(const Context<double>& context,
                                           SystemOutput<double>* output) const {
  const PoseBundle<double>& poses =
      this->EvalAbstractInput(context, 0)
          ->template GetValue<PoseBundle<double>>();
  lcmt_viewer_draw& message =
      output->GetMutableData(0)->template GetMutableValue<lcmt_viewer_draw>();

  const int n = poses.get_num_poses();

  message.timestamp = static_cast<int64_t>(context.get_time() * 1000.0);
  message.num_links = n;
  message.link_name.resize(n);
  message.robot_num.resize(n);
  message.position.resize(n);
  message.quaternion.resize(n);

  for (int i = 0; i < n; ++i) {
    // TODO(david-german-tri): Support non-unique link names by populating
    // robot_num. Will require changes in PoseAggregator and PoseBundle.
    message.robot_num[i] = 0;

    message.link_name[i] = poses.get_name(i);

    Eigen::Translation<double, 3> t(poses.get_pose(i).translation());
    message.position[i].resize(3);
    message.position[i][0] = t.x();
    message.position[i][1] = t.y();
    message.position[i][2] = t.z();

    Eigen::Quaternion<double> q(poses.get_pose(i).rotation());
    message.quaternion[i].resize(4);
    message.quaternion[i][0] = q.w();
    message.quaternion[i][1] = q.x();
    message.quaternion[i][2] = q.y();
    message.quaternion[i][3] = q.z();
  }
}

std::unique_ptr<AbstractValue>
PoseBundleToDrawMessage::AllocateOutputAbstract(
    const OutputPortDescriptor<double>& descriptor) const {
  return AbstractValue::Make<lcmt_viewer_draw>(lcmt_viewer_draw());
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake
