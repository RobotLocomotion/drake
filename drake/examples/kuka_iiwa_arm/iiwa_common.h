#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Computes the lumped inertia parameters of the gripper and the end effector
/// link expressed in the end effector frame.
/// @param world_tree The RigidBodyTree that contains the arm and the gripper
/// models.
/// @param iiwa_instance Identifier for the arm in @p world_tree.
/// @param end_effector_link_name Link name of the end effector.
/// @param wsg_instance Identifier for the gripper in @p world_tree.
/// @return Lumped inertia parameters.
template <typename T>
Matrix6<T> ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<T>& world_tree,
    int iiwa_instance, const std::string& end_effector_link_name,
    int wsg_instance);

/// Verifies that @p tree matches assumptions about joint indices.
/// Aborts if the tree isn't as expected.
void VerifyIiwaTree(const RigidBodyTree<double>& tree);

/// This method generates a simple trajectory plan for a robot using Cartesian
/// (end effector) way points. The robot is specified by @p tree. The
/// way-points must be supplied in the world frame.
/// This method wraps a call to the `inverseKinPointwise` method.
/// @see inverseKinPointwise.
std::unique_ptr<PiecewisePolynomialTrajectory> SimpleCartesianWayPointPlanner(
    const RigidBodyTreed& tree, const std::string& link_to_constrain,
    const std::vector<Eigen::Vector3d>& way_point_list,
    const std::vector<double>& time_stamps);

/// Builds a vector of time window points distributed about the given time
/// stamps. Time window positions do not overlap. The @p lower_ratio
/// and @p upper_ratio variables can be used to proportionally compute
/// the lower and upper window points between adjacent
/// time stamps. The logic ensures that the windows never overlap for a
/// monotonically increasing time stamp vector.
/// For instance, for a lower_ratio = 0.4, and upper_ratio = 0.5, and time
/// stamp t(k), the window is located at
/// [t(k-1) + 0.4 * ( t(k) - t(k-1)), t(k) + 0.5 * (t(k+1) - t(k))]
/// i.e. between 40% of the previous time step to 50% of the subsequent time
/// step, with the boundary conditions [0, t(f) + 0.5*(t(f) - t(f-1))].
/// The method requires @p upper_ratio > @p lower_ratio.
std::vector<Eigen::Vector2d> TimeWindowBuilder(
    const std::vector<double>& time_stamps, double lower_ratio = 0.4,
    double upper_ratio = 0.5);

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convinience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreedFromFixedModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());

/// Used to set the feedback gains for the simulated position controlled KUKA.
void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

/// Makes a robotlocomotion::robot_plan_t message.
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot, const std::vector<double>& time,
    const std::vector<int>& info, const MatrixX<double>& keyframes);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
