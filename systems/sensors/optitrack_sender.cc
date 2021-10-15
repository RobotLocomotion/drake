#include "drake/systems/sensors/optitrack_sender.h"

#include <vector>

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/geometry/frame_kinematics_vector.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {
void PopulateRigidBody(const math::RigidTransformd& T_WF,
                       optitrack::optitrack_rigid_body_t* msg) {
    const Eigen::Vector3d trans = T_WF.translation();
    const Eigen::Quaterniond rot = T_WF.rotation().ToQuaternion();

    msg->xyz[0] = static_cast<float>(trans[0]);
    msg->xyz[1] = static_cast<float>(trans[1]);
    msg->xyz[2] = static_cast<float>(trans[2]);

    msg->quat[0] = static_cast<float>(rot.x());
    msg->quat[1] = static_cast<float>(rot.y());
    msg->quat[2] = static_cast<float>(rot.z());
    msg->quat[3] = static_cast<float>(rot.w());
}
}  // namespace

OptitrackLcmFrameSender::OptitrackLcmFrameSender(
    const std::map<geometry::FrameId,
    std::pair<std::string, int>>& frame_map)
    : num_rigid_bodies_(frame_map.size()),
      frame_map_(frame_map) {
  pose_input_port_index_ = this->DeclareAbstractInputPort(
      kUseDefaultName,
      Value<geometry::FramePoseVector<double>>()).get_index();
  this->DeclareAbstractOutputPort(
      kUseDefaultName,
      optitrack::optitrack_frame_t(),
      &OptitrackLcmFrameSender::PopulatePoseMessage);
}

void OptitrackLcmFrameSender::PopulatePoseMessage(
    const Context<double>& context,
    optitrack::optitrack_frame_t* output) const {
  output->utime = static_cast<int64_t >(context.get_time() * 1e6);
  output->num_rigid_bodies = num_rigid_bodies_;
  output->rigid_bodies.resize(static_cast<size_t>(num_rigid_bodies_));

  const auto& poses = get_optitrack_input_port().
      Eval<geometry::FramePoseVector<double>>(context);

  int output_index = 0;
  for (const auto& frame : frame_map_) {
    const math::RigidTransformd& pose = poses.value(frame.first);
    output->rigid_bodies[output_index].id = frame.second.second;
    PopulateRigidBody(pose, &(output->rigid_bodies[output_index++]));
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
