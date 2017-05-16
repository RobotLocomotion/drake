#include "drake/examples/Valkyrie/robot_state_decoder.h"

#include <utility>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace systems {

RobotStateDecoder::RobotStateDecoder(const RigidBodyTree<double>& tree)
    : translator_(tree),
      robot_state_message_port_index_(DeclareAbstractInputPort().get_index()),
      kinematics_cache_port_index_(DeclareAbstractOutputPort().get_index()) {
  set_name("RobotStateDecoder");
}

void RobotStateDecoder::DoCalcOutput(const Context<double>& context,
                                     SystemOutput<double>* output) const {
  // Input: robot_state_t message.
  const auto& message =
      EvalAbstractInput(context, robot_state_message_port_index_)
          ->GetValue<bot_core::robot_state_t>();

  // Output: KinematicsCache.
  auto& kinematics_cache = output->GetMutableData(kinematics_cache_port_index_)
                               ->GetMutableValue<KinematicsCache<double>>();

  VectorX<double> q(translator_.get_robot().get_num_positions());
  VectorX<double> v(translator_.get_robot().get_num_velocities());
  translator_.DecodeMessageKinematics(message, q, v);

  kinematics_cache.initialize(q, v);
  translator_.get_robot().doKinematics(kinematics_cache, true);
}

std::unique_ptr<AbstractValue> RobotStateDecoder::AllocateOutputAbstract(
    const OutputPortDescriptor<double>&) const {
  return std::make_unique<Value<KinematicsCache<double>>>(
      translator_.get_robot().CreateKinematicsCache());
}

}  // namespace systems
}  // namespace drake
