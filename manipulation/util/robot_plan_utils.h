#pragma once

/// @file Functions to help with the creation of lcmt_robot_plan messages.

#include <string>
#include <vector>

#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace util {

/// @return A vector of joint names corresponding to the positions in @p plant
/// in the order of the joint indices.  If joints with duplicate names exist
/// in different model instance in the plant, the names will be duplicated in
/// the output.
template <typename T>
std::vector<std::string> GetJointNames(
    const multibody::MultibodyPlant<T>& plant);

/// Scales a plan so that no step exceeds the robot's maximum joint
/// velocities.  The size of @p keyframes must match the size of @p times.
/// Times must be in strictly increasing order and start with zero.  Per-joint
/// velocity limits are specified by @p limits, which much be the same size ad
/// the number of joints in each element of @p keyframes. Assumes that
/// velocity limits are equal regardless of direction.  If any step does
/// exceed the maximum velocities in @p limits, @p times will be modified to
/// reduce the velocity.
void ApplyJointVelocityLimits(
    const std::vector<Eigen::VectorXd>& keyframes,
    const Eigen::VectorXd& limits,
    std::vector<double>* times);

/// Makes an lcmt_robot_plan message.  The entries in @p
/// joint_names should be unique, though the behavior if names are duplicated
/// depends on how the returned plan is evaluated.  The size of each vector in
/// @p keyframes must match the size of @p joint_names.  The size of @p
/// keyframes must match the size of @p times.  Times must be in strictly
/// increasing order.
lcmt_robot_plan EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& times,
    const std::vector<Eigen::VectorXd>& keyframes);

}  // namespace util
}  // namespace manipulation
}  // namespace drake
