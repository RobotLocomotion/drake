#pragma once

#include "drake/common/eigen_types.h"
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

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
