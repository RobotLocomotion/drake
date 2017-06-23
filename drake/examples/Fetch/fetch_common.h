#pragma once
/// @file
///
/// This file assumes a 21-degree of freedom Fetch mobile robot. Joints are
/// numbered sequentially starting from the base with the following joint
/// index descriptions:
/// 0: base x-translation
/// 1: base y-translation
/// 2: base z-translation
/// 3: quaternion element q0
/// 4: quaternion element q1
/// 5: quaternion element q2
/// 6: quaternion element q3
/// 7: r_wheel_joint
/// 8: l_wheel_joint
/// 9: torso_lift_joint
/// 10: head_pan_joint
/// 11: head_tilt_joint
/// 12: shoulder_pan_joint
/// 13: shoulder_lift_joint
/// 14: shoulder_lift_joint
/// 15: upperarm_roll_joint
/// 16: elbow_flex_joint
/// 17: forearm_roll_joint
/// 18: wrist_flex_joint
/// 19: wrist_roll_joint
/// 20: r_gripper_finger_joint
/// 21: l_gripper_finger_joint
///
/// Rotational position/velocity units are in rad and rad/s, respectively.
/// Linear position units are in meters and m/s, respectively.

#include <string>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace Fetch {

constexpr int kNumDofs =
    21;  // degrees of freedom available for the Fetch robot

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyFetchTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreeFromFloatingModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());

}  // namespace Fetch
}  // namespace examples
}  // namespace drake
