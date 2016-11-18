#include "drake/examples/Valkyrie/robot_state_lcmtype_util.h"

namespace drake {
namespace systems {

const RigidBodyTree<double>& CheckTreeIsRobotStateLcmTypeCompatible(
    const RigidBodyTree<double>& tree) {
  if (tree.get_num_bodies() < 2) {
    DRAKE_ABORT_MSG("This class assumes at least one non-world body.");
  }

  bool floating_joint_found = false;
  for (const auto& body_ptr : tree.bodies) {
    if (body_ptr->has_parent_body()) {
      const auto& joint = body_ptr->getJoint();
      if (joint.is_floating()) {
        if (floating_joint_found) {
          DRAKE_ABORT_MSG("robot_state_t assumes at most one floating joint.");
        }
        floating_joint_found = true;

        if (body_ptr != tree.bodies[1]) {
          DRAKE_ABORT_MSG(
              "This class assumes that the first non-world body is the "
              "floating body.");
        }

        if (body_ptr->get_position_start_index() != 0 ||
            body_ptr->get_velocity_start_index() != 0) {
          DRAKE_ABORT_MSG(
              "This class assumes that floating joint positions and are at the "
              "head of the position and velocity vectors.");
        }
      } else {
        if (joint.get_num_positions() > 1 || joint.get_num_velocities() > 1) {
          DRAKE_ABORT_MSG(
              "robot_state_t assumes non-floating joints to be "
              "1-DoF or fixed.");
        }
      }
    }
  }

  return tree;
}

}  // namespace systems
}  // namespace drake
