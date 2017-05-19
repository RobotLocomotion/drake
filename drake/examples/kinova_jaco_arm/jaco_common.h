#pragma once

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyJacoTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreedFromFixedModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());

/// Used to set the feedback gains for the simulated position controlled KUKA.
void SetPositionControlledJacoGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
