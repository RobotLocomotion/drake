#pragma once
/// @file
///
/// This file assumes a 6-degree of freedom Kinova Jaco arm with a three
/// finger gripper. Joints are numbered sequentially starting from the base
/// with the following joint index descriptions:
/// 0: shoulder roll
/// 1: shoulder fore/aft
/// 2: elbow fore/aft
/// 3: forearm roll
/// 4: wrist yaw
/// 5: wrist roll
/// 6: finger 1 bend/extend
/// 7: finger 2 bend/extend
/// 8: finger 3 bend/extend
///
/// Position units are in radians, velocity units are in radians per second

#include <string>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

constexpr int kNumDofs = 9;  // degrees of freedom available for the jaco arm

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyJacoTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreeFromFixedModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());

/// Sets the feedback gains for the simulated position controlled Jaco arm.
void SetPositionControlledJacoGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
