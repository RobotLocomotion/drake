#include "drake/examples/valkyrie/robot_state_decoder.h"

#include <utility>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace systems {

RobotStateDecoder::RobotStateDecoder(const RigidBodyTree<double>& tree)
    : translator_(tree),
      robot_state_message_port_index_(DeclareAbstractInputPort(
          kUseDefaultName,
          Value<bot_core::robot_state_t>{}).get_index()) {
  DeclareAbstractOutputPort(
      KinematicsCache<double>(translator_.get_robot().CreateKinematicsCache()),
      &RobotStateDecoder::OutputKinematics);
  set_name("RobotStateDecoder");
}

void RobotStateDecoder::OutputKinematics(
    const Context<double>& context, KinematicsCache<double>* output) const {
  // Input: robot_state_t message.
  const auto& message = get_input_port(robot_state_message_port_index_).
      Eval<bot_core::robot_state_t>(context);

  // Output: KinematicsCache.
  auto& kinematics_cache = *output;

  VectorX<double> q(translator_.get_robot().get_num_positions());
  VectorX<double> v(translator_.get_robot().get_num_velocities());
  translator_.DecodeMessageKinematics(message, q, v);

  kinematics_cache.initialize(q, v);
  translator_.get_robot().doKinematics(kinematics_cache, true);
}

}  // namespace systems
}  // namespace drake
