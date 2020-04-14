#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

// These details have moved to files under drake/manipulation/kuka_iiwa.
// These forwarding aliases are placed here for compatibility purposes.
using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using manipulation::kuka_iiwa::get_iiwa_max_joint_velocities;

/// Computes the lumped inertia parameters of the gripper and the end effector
/// link expressed in the end effector frame.
/// @param world_tree The RigidBodyTree that contains the arm and the gripper
/// models.
/// @param iiwa_instance Identifier for the arm in @p world_tree.
/// @param end_effector_link_name Link name of the end effector.
/// @param wsg_instance Identifier for the gripper in @p world_tree.
/// @return Lumped inertia parameters.
template <typename T>
DRAKE_DEPRECATED("2020-05-01", "This function is being removed.")
Matrix6<T> ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<T>& world_tree,
    int iiwa_instance, const std::string& end_effector_link_name,
    int wsg_instance);

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
DRAKE_DEPRECATED("2020-06-01", "This function is being removed.")
void VerifyIiwaTree(const RigidBodyTree<double>& tree);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
DRAKE_DEPRECATED("2020-05-01", "This function is being removed.")
void CreateTreedFromFixedModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());

/// Used to set the feedback gains for the simulated position controlled KUKA.
void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

/// Used to set the feedback gains for the simulated torque controlled KUKA.
void SetTorqueControlledIiwaGains(Eigen::VectorXd* stiffness,
                                  Eigen::VectorXd* damping_ratio);

/// Scales a plan so that no step exceeds the robot's maximum joint velocities.
/// The number of columns in @p keyframes must match the size of @p time.  Times
/// must be in strictly increasing order.
/// @see get_iiwa_max_joint_velocities
DRAKE_DEPRECATED("2020-07-01",
                 "This function is being moved to manipulation::util.")
void ApplyJointVelocityLimits(const MatrixX<double>& keyframes,
                              std::vector<double>* time);

/// Makes a robotlocomotion::robot_plan_t message.  The number of
/// columns in @p keyframes must match the size of @p time.  Times
/// must be in strictly increasing order.
DRAKE_DEPRECATED("2020-06-01", "This function is being removed.")
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot, const std::vector<double>& time,
    const std::vector<int>& info, const MatrixX<double>& keyframes);

/// Makes a robotlocomotion::robot_plan_t message.  The number of rows in @p
/// keyframes must match the size of @p joint_names.  The number of columns in
/// @p keyframes must match the size of @p time.  Times must be in strictly
/// increasing order.
DRAKE_DEPRECATED("2020-07-01",
                 "This function is being moved to manipulation::util.")
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& time, const std::vector<int>& info,
    const MatrixX<double>& keyframes);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
