#pragma once
/// @file
///
/// This file assumes a 21-degree of freedom Fetch mobile robot. Joints are
/// numbered sequentially starting from the base with the following joint
/// index descriptions:
/// 0: base x-translation
/// 1: base y-translation
/// 2: base z-translation
/// 3: quaternion element w
/// 4: quaternion element x
/// 5: quaternion element y
/// 6: quaternion element z
/// 7: r_wheel_joint
/// 8: l_wheel_joint
/// 9: torso_lift_joint
/// 10: head_pan_joint
/// 11: head_tilt_joint
/// 12: shoulder_pan_joint
/// 13: shoulder_lift_joint
/// 14: upperarm_roll_joint
/// 15: elbow_flex_joint
/// 16: forearm_roll_joint
/// 17: wrist_flex_joint
/// 18: wrist_roll_joint
/// 19: r_gripper_finger_joint
/// 20: l_gripper_finger_joint
///
/// Rotational position/velocity units are in rad and rad/s, respectively.
/// Linear position units are in meters and m/s, respectively.

#include <string>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace Fetch {

constexpr int kNumDofs = 21;  // DOFs available for the Fetch robot

constexpr int kBasexTranslationIdx = 0;
constexpr int kBaseyTranslationIdx = 1;
constexpr int kBasezTranslationIdx = 2;
constexpr int kQuatwElementIdx = 3;
constexpr int kQuatxElementIdx = 4;
constexpr int kQuatyElementIdx = 5;
constexpr int kQuatzElementIdx = 6;
constexpr int kRWheelJointIdx = 7;
constexpr int kLWheelJointIdx = 8;
constexpr int kTorsoLiftJointIdx = 9;
constexpr int kHeadPanJointIdx = 10;
constexpr int kHeadTiltJointIdx = 11;
constexpr int kShoulderPanJointIdx = 12;
constexpr int kShoulderLiftJointIdx = 13;
constexpr int kUpperarmRollJointIdx = 14;
constexpr int kElbowFlexJointIdx = 15;
constexpr int kForearmRollJointIdx = 16;
constexpr int kWristFlexJointIdx = 17;
constexpr int kWristRollJointIdx = 18;
constexpr int kRGripperFingerJointIdx = 19;
constexpr int kLGripperFingerJointx = 20;


/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyFetchTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreeFromFloatingModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Isometry3d& orientation = Eigen::Isometry3d::Identity());

}  // namespace Fetch
}  // namespace examples
}  // namespace drake
