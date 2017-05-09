#include "drake/examples/Valkyrie/robot_state_decoder.h"

#include <utility>

#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

#include "lcmtypes/bot_core/robot_state_t.hpp"

namespace drake {
namespace systems {

RobotStateDecoder::RobotStateDecoder(const RigidBodyTree<double>& tree)
    : translator_(tree),
      robot_state_message_port_index_(DeclareAbstractInputPort().get_index()),
      joint_name_to_body_(CreateJointNameToBodyMap(tree)) {
  DeclareAbstractOutputPort(
      KinematicsCache<double>(tree_.CreateKinematicsCache()),
      &RobotStateDecoder::OutputKinematics);
  set_name("RobotStateDecoder");
}

void RobotStateDecoder::OutputKinematics(const Context<double>& context,
                                     KinematicsCache<double>* output) const {
  // Input: robot_state_t message.
  const auto& message =
      EvalAbstractInput(context, robot_state_message_port_index_)
          ->GetValue<bot_core::robot_state_t>();

  // Output: KinematicsCache.
  auto& kinematics_cache = *output;

  VectorX<double> q(translator_.get_robot().get_num_positions());
  VectorX<double> v(translator_.get_robot().get_num_velocities());
  translator_.DecodeMessageKinematics(message, q, v);

  kinematics_cache.initialize(q, v);
  translator_.get_robot().doKinematics(kinematics_cache, true);
}

std::map<std::string, const RigidBody<double>*>
RobotStateDecoder::CreateJointNameToBodyMap(const RigidBodyTree<double>& tree) {
  map<string, const RigidBody<double>*> ret;
  for (const auto& body : tree.bodies) {
    if (body->has_parent_body()) {
      const auto& joint = body->getJoint();
      if (!joint.is_fixed() && !joint.is_floating()) {
        // To match usage of robot_state_t throughout OpenHumanoids code, use
        // position coordinate name as joint name.
        int position_index = body->get_position_start_index();
        ret[tree_.get_position_name(position_index)] = body.get();
      }
    }
  }
  return ret;
}

}  // namespace systems
}  // namespace drake
