#pragma once

/// @file Functions to help with the creation of robot_plan_t messages.

#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace util {

/// @return A vector of joint names corresponding to the positions in @plant
/// in the order of the joint indices.
template <typename T>
std::vector<std::string> GetJointNames(const multibody::MultibodyPlant<T>&);

/// Scales a plan so that no step exceeds the robot's maximum joint
/// velocities.  The size of @p keyframes must match the size of @p time.
/// Times must be in strictly increasing order.  Assumes that velocity limits
/// are equal regardless of direction.
void ApplyJointVelocityLimits(
    const std::vector<Eigen::VectorXd>& keyframes,
    const Eigen::VectorXd& limit,
    std::vector<double>* time);

/// Makes a robotlocomotion::robot_plan_t message.  The size of each vector in
/// @p keyframes must match the size of @p joint_names.  The size of @p
/// keyframes must match the size of @p time.  Times must be in strictly
/// increasing order.
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& time, const std::vector<int>& info,
    const std::vector<Eigen::VectorXd>& keyframes);

}  // namespace util
}  // namespace manipulation
}  // namespace drake
