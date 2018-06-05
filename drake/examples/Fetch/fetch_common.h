#pragma once
/// @file
///
/// This file assumes a 20-degree of freedom Fetch mobile robot. Joints are
/// numbered sequentially starting from the base with the joint index
/// descriptions specified below.
/// Note: The four element base quaternion vector specifies the rotation of the
/// base and must be set accordingly. The individual quaternion vector elements
/// do not correspond to individual joints. Also, since a quaternion vector
/// contains one more element than the angular velocity vector, the numbering of
/// the position and velocity indices differ.
/// ---- Position indexing ---          ---- Velocity Indexing ----
/// 0: base x-translation               0: base x-translation
/// 1: base y-translation               1: base y-translation
/// 2: base z-translation               2: base z-translation
/// 3: base quaternion element w        3: base omega_x
/// 4: base quaternion element x        4: base omega_y
/// 5: base quaternion element y        5: base omega_z
/// 6: base quaternion element z        6: r_wheel_joint
/// 7: r_wheel_joint                    7: l_wheel_joint
/// 8: l_wheel_joint                    8: torso_lift_joint
/// 9: torso_lift_joint                 9: head_pan_joint
/// 10: head_pan_joint                  10: head_tilt_joint
/// 11: head_tilt_joint                 11: shoulder_pan_joint
/// 12: shoulder_pan_joint              12: shoulder_lift_joint
/// 13: shoulder_lift_joint             13: upperarm_roll_joint
/// 14: upperarm_roll_joint             14: elbow_flex_joint
/// 15: elbow_flex_joint                15: forearm_roll_joint
/// 16: forearm_roll_joint              16: wrist_flex_joint
/// 17: wrist_flex_joint                17: wrist_roll_joint
/// 18: wrist_roll_joint                18: r_gripper_finger_joint
/// 19: r_gripper_finger_joint          19: l_gripper_finger_joint
/// 20: l_gripper_finger_joint
///
/// Rotational position/velocity units are in rad and rad/s, respectively.
/// Linear position units are in meters and m/s, respectively.

#include <string>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace Fetch {

constexpr int kNumPosIndices = 21;  // Position indices for the Fetch robot
constexpr int kNumVelIndices = 20;  // Velocity indices for the Fetch robot

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyFetchTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreeFromFloatingModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Isometry3d& pose = Eigen::Isometry3d::Identity());

}  // namespace Fetch
}  // namespace examples
}  // namespace drake
