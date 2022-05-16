#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/** Converts LCM optitrack_frame_t message data on an input port to
RigidTransform data on per-body output ports.

@system
name: OptitrackReceiver
input_ports:
- optitrack_frame_t
output_ports:
- <em style="color:gray">...</em>
- <em style="color:gray">...</em>
@endsystem
*/
class OptitrackReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackReceiver)

  /** Constructs an OptitrackReceiver.
  @param frame_map Mapping from optitrack body id to output port name.
    All ids must always be present in the input message.
  @param X_WO Pose of the optitrack frame O in the world frame W.
    Defaults to the Identity transform.
  */
  explicit OptitrackReceiver(
      const std::map<int, std::string>& frame_map,
      const math::RigidTransformd& X_WO = {});

 private:
  void CalcOutput(const Context<double>&, int, math::RigidTransformd*) const;

  const math::RigidTransformd X_WO_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
