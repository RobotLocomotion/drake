#pragma once

#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

// These details have moved to files under drake/manipulation/kuka_iiwa.
// These forwarding aliases are placed here for compatibility purposes.
using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using manipulation::kuka_iiwa::get_iiwa_max_joint_velocities;

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
