#include "drake/systems/sensors/optitrack_sender.h"

#include <vector>

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/systems/sensors/optitrack_encoder.h"

namespace drake {
namespace systems {
namespace sensors {

using systems::sensors::TrackedBody;

OptitrackLCMFrameSender::OptitrackLCMFrameSender(int num_rigid_bodies)
    : num_rigid_bodies_(num_rigid_bodies) {
  DRAKE_DEMAND(num_rigid_bodies >= 0);
  this->DeclareAbstractInputPort();
  this->DeclareAbstractOutputPort(&OptitrackLCMFrameSender::CreateNewMessage,
                                  &OptitrackLCMFrameSender::PopulateMessage);
}

optitrack::optitrack_frame_t OptitrackLCMFrameSender::CreateNewMessage() const {
  optitrack::optitrack_frame_t msg{};

  msg.num_rigid_bodies = num_rigid_bodies_;
  msg.rigid_bodies.resize(static_cast<size_t>(num_rigid_bodies_));

  return msg;
}

void OptitrackLCMFrameSender::PopulateMessage(
    const Context<double>& context,
    optitrack::optitrack_frame_t* output) const {

  optitrack::optitrack_frame_t& status = *output;

  status.utime = static_cast<int64_t >(context.get_time() * 1e6);

  const std::vector<TrackedBody>* mocap_objects =
      this->EvalInputValue<std::vector<TrackedBody>>(context, 0);

  for (size_t i = 0; i < mocap_objects->size(); ++i) {
    status.rigid_bodies[i].id = (*mocap_objects)[i].id;

    const Eigen::Vector3d trans = (*mocap_objects)[i].T_WF.translation();
    const Eigen::Quaterniond rot = Eigen::Quaterniond(
        (*mocap_objects)[i].T_WF.linear());

    status.rigid_bodies[i].xyz[0] = static_cast<float>(trans[0]);
    status.rigid_bodies[i].xyz[1] = static_cast<float>(trans[1]);
    status.rigid_bodies[i].xyz[2] = static_cast<float>(trans[2]);

    status.rigid_bodies[i].quat[0] = static_cast<float>(rot.x());
    status.rigid_bodies[i].quat[1] = static_cast<float>(rot.y());
    status.rigid_bodies[i].quat[2] = static_cast<float>(rot.z());
    status.rigid_bodies[i].quat[3] = static_cast<float>(rot.w());
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
